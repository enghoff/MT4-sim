"""Drive the simulated arm in the model angles the real stack speaks.

Callers work in the same units and frame as the MT4 control repo: model angles
in degrees, TCP in robot-frame millimetres, gripper as a firmware S value. The
translation to the URDF chain's joint positions lives in :mod:`mt4_sim.chain`.

This deliberately does not reimplement the firmware's motion planner. The real
``mp``/``mq`` commands interpolate straight world-frame lines, route around the
keep-out cylinder and validate every segment; a position drive here jumps the
joints toward a target instead. What it does give is a pose whose FK agrees with
the real arm's to floating-point, so anything measured off the simulated camera
lands in coordinates the real arm would accept.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

from mt4_sim.chain import (
    ARM_JOINT_NAMES,
    DESK_Z_MM,
    FINGER_JOINT_NAMES,
    GRIPPER_S_CLOSED,
    GRIPPER_S_OPEN,
    JointAnglesDeg,
    Q_LIMITS,
    finger_positions_for_s,
    model_from_urdf,
    park_pose,
    urdf_from_model,
)
from mt4_sim.urdf import JOINTS
from mt4_jog.kinematics import fk_tcp, ik_position

ARM_PRIM_PATH = "/World/MT4"
# The URDF importer nests link prims exactly as the chain nests them.
GEOMETRY_SCOPE = "Geometry"


def link_prim_path(link: str, arm_prim_path: str = ARM_PRIM_PATH) -> str:
    """Where a URDF link ended up on the stage, walked up the joint tree."""
    parents = {j.child: j.parent for j in JOINTS}
    names = [link]
    while names[-1] in parents:
        names.append(parents[names[-1]])
    return "/".join([arm_prim_path, GEOMETRY_SCOPE, *reversed(names)])


# The wrist-roll frame is the TCP: the roll axis passes through it, which is
# why `fk_tcp` has no q4 term.
TCP_PRIM_PATH = link_prim_path("gripper_base")


@dataclass(frozen=True)
class ArmState:
    q: JointAnglesDeg
    tcp_mm: tuple[float, float, float]
    gripper_s: float


class SimArm:
    """An Isaac Sim articulation wrapper that speaks MT4 model angles.

    Construct after the world has been reset, so the physics view exists and
    joint indices can be resolved.
    """

    def __init__(self, prim_path: str = ARM_PRIM_PATH) -> None:
        from isaacsim.core.prims import SingleArticulation

        self._art = SingleArticulation(prim_path=prim_path, name="mt4")
        self._art.initialize()

        names = list(self._art.dof_names)
        missing = [n for n in (*ARM_JOINT_NAMES, *FINGER_JOINT_NAMES) if n not in names]
        if missing:
            raise RuntimeError(
                f"articulation at {prim_path} has no DOFs named {missing}; found {names}"
            )
        self._arm_dofs = [names.index(n) for n in ARM_JOINT_NAMES]
        self._finger_dofs = [names.index(n) for n in FINGER_JOINT_NAMES]
        self._gripper_s = float(GRIPPER_S_CLOSED)
        self._commanded: JointAnglesDeg | None = None

    # -- commanding -------------------------------------------------------

    def _drive(self, positions: np.ndarray, dof_indices: list[int]) -> None:
        from isaacsim.core.utils.types import ArticulationAction

        self._art.apply_action(
            ArticulationAction(joint_positions=positions, joint_indices=dof_indices)
        )

    def set_model_angles(self, q: JointAnglesDeg, *, teleport: bool = False) -> None:
        """Command the arm to model angles q1..q4 (degrees).

        ``teleport`` writes the joint positions directly instead of driving to
        them -- for placing the arm at a known pose before stepping, where a
        drive would swing it there through whatever lies between.
        """
        for index, limit in zip(("j1", "j2", "j3", "j4"), Q_LIMITS):
            value = getattr(q, index)
            if not limit.lower_deg - 1e-6 <= value <= limit.upper_deg + 1e-6:
                raise ValueError(
                    f"{index}={value:.3f} deg is outside the arm's soft limits "
                    f"[{limit.lower_deg:.3f}, {limit.upper_deg:.3f}]"
                )

        positions = np.asarray(urdf_from_model(q), dtype=float)
        if teleport:
            self._art.set_joint_positions(positions, joint_indices=self._arm_dofs)
            self._art.set_joint_velocities(
                np.zeros(len(self._arm_dofs)), joint_indices=self._arm_dofs
            )
        self._commanded = q
        self._drive(positions, self._arm_dofs)

    def set_gripper_s(self, s: float, *, teleport: bool = False) -> None:
        """Command the gripper by firmware S value (120 open .. 285 closed)."""
        if not GRIPPER_S_OPEN <= s <= GRIPPER_S_CLOSED:
            raise ValueError(f"gripper S must be {GRIPPER_S_OPEN}-{GRIPPER_S_CLOSED} (got {s})")
        self._gripper_s = float(s)
        positions = np.asarray(finger_positions_for_s(s), dtype=float)
        if teleport:
            self._art.set_joint_positions(positions, joint_indices=self._finger_dofs)
        self._drive(positions, self._finger_dofs)

    def move_to_tcp(self, x_mm: float, y_mm: float, z_mm: float, *, j4_deg: float | None = None):
        """Solve the control repo's own position IK and drive there.

        Returns the commanded model angles, or None when the point is out of the
        two-link reach -- the same answer ``mt4_jog.kinematics.ik_position``
        gives the real client.
        """
        near = self.state().q
        q = ik_position(x_mm, y_mm, z_mm, near=near, hold_orientation=j4_deg is None)
        if q is None:
            return None
        if j4_deg is not None:
            q = JointAnglesDeg(q.j1, q.j2, q.j3, j4_deg)
        self.set_model_angles(q)
        return q

    def park(self, *, teleport: bool = True) -> JointAnglesDeg:
        """Put the arm in the pose it holds after homing."""
        q = park_pose()
        self.set_model_angles(q, teleport=teleport)
        self.set_gripper_s(GRIPPER_S_CLOSED, teleport=teleport)
        return q

    # -- reading ----------------------------------------------------------

    def urdf_joint_positions(self) -> tuple[float, float, float, float, float]:
        """The five arm joint positions as physics has them (rad), chain order."""
        positions = self._art.get_joint_positions(joint_indices=self._arm_dofs)
        return tuple(float(v) for v in positions)  # type: ignore[return-value]

    def head_tilt_deg(self) -> float:
        """How far off level the head platform is.

        On the real arm the link rods hold it level mechanically. Here it is a
        driven joint, so it gives up the drive's steady-state error to the weight
        of the gripper -- and a tilted head swings the TCP through
        ``HEAD_OFFSET``, which is the one way the simulated TCP can differ from
        ``fk_tcp`` at rest.
        """
        _q1, q2, q3_rel, level, _q4 = self.urdf_joint_positions()
        return math.degrees(level + (q2 + q3_rel))

    def state(self) -> ArmState:
        q = model_from_urdf(self.urdf_joint_positions())
        tcp = fk_tcp(q)
        return ArmState(q, (tcp.x, tcp.y, tcp.z), self._gripper_s)

    def tracking_error_deg(self) -> float | None:
        """How far the joints are from the last pose they were commanded to.

        A stepper is where its pulse train left it; a position drive is still
        catching up. Anything that wants "the move has finished" to mean the
        same thing here as on the bench has to wait for this to fall.
        Returns None before the first command.
        """
        if self._commanded is None:
            return None
        measured = model_from_urdf(self.urdf_joint_positions())
        return max(
            abs(getattr(self._commanded, name) - getattr(measured, name))
            for name in ("j1", "j2", "j3", "j4")
        )

    def measured_tcp_mm(self) -> tuple[float, float, float]:
        """Where the simulated TCP frame actually is, in robot mm.

        ``state().tcp_mm`` runs the control repo's FK on the joint angles. This
        reads the body pose out of the scene graph instead, so the two
        disagreeing means the URDF chain does not match the kinematic model --
        which is what ``tests``/``scripts/check.py`` assert against.
        """
        from isaacsim.core.utils.prims import get_prim_at_path
        from pxr import UsdGeom

        prim = get_prim_at_path(TCP_PRIM_PATH)
        if not prim.IsValid():
            raise RuntimeError(f"no TCP prim at {TCP_PRIM_PATH}")
        world = UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(0.0)
        t = world.ExtractTranslation()
        return (t[0] * 1000.0, t[1] * 1000.0, t[2] * 1000.0)


def desk_z_mm() -> float:
    return DESK_Z_MM


def tcp_above_desk_mm(state: ArmState) -> float:
    return state.tcp_mm[2] - DESK_Z_MM


def deg(radians: float) -> float:
    return math.degrees(radians)
