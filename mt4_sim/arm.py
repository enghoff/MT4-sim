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
    TCP_GRIP_Z_MM,
    FINGER_ARMATURE_KG,
    FINGER_COUPLING_DAMPING,
    FINGER_COUPLING_NAME,
    FINGER_COUPLING_STIFFNESS,
    FINGER_DAMPING_N_S_PER_M,
    FINGER_DYNAMIC_FRICTION,
    FINGER_EFFORT_N,
    FINGER_JOINT_NAMES,
    FINGER_MAX_SPEED_M_S,
    FINGER_STATIC_FRICTION,
    FINGER_STIFFNESS_N_PER_M,
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
        from isaacsim.core.utils.stage import get_current_stage

        # Contact attrs must be finite *before* the articulation view parses
        # physics; the importer/runtime defaults ``newton:contactGap`` (and
        # sometimes PhysX offsets) to -inf, which is exactly the "jaws dig
        # into the cube" failure mode.
        prepare_gripper_contacts(get_current_stage(), prim_path=prim_path)

        self._prim_path = prim_path
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
        self._configure_finger_drives()
        self._configure_finger_pads()

    def _configure_finger_drives(self) -> None:
        """Set the jaw drives to the soft, force-capped spring the grip needs.

        The USD may still carry an older import's numbers, so rewriting here
        means a rebuild is not required for the grip force to take effect --
        only for the URDF's own effort/damping attributes to agree.
        """
        configure_finger_joints(self._art.prim.GetStage(), prim_path=self._prim_path)

    def _configure_finger_pads(self) -> None:
        """Bind rubber-pad friction to every finger collision mesh."""
        stage = self._art.prim.GetStage()
        prepare_gripper_contacts(stage, prim_path=self._prim_path)
        bound = bind_finger_friction(stage, prim_path=self._prim_path)
        if bound < 2:
            raise RuntimeError(
                f"expected a physics material on both finger colliders, bound {bound}"
            )

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

    def finger_positions(self) -> tuple[float, float]:
        """Measured prismatic finger openings (m), chain order."""
        positions = self._art.get_joint_positions(joint_indices=self._finger_dofs)
        return (float(positions[0]), float(positions[1]))

    def finger_velocities(self) -> tuple[float, float]:
        velocities = self._art.get_joint_velocities(joint_indices=self._finger_dofs)
        return (float(velocities[0]), float(velocities[1]))

    def set_gripper_s(self, s: float, *, teleport: bool = False) -> None:
        """Command the gripper by firmware S value (120 open .. 285 closed).

        Firmware S is open-loop and goes straight to the host's close command,
        the same as the real board. So does the finger drive: the host closes
        *past* contact (S=255 is a negative span, clamped to zero), the drive's
        position error against an object is therefore always large, and the
        force cap is what the object actually feels. The jaws are a
        constant-force closer at ``FINGER_EFFORT_N``, which is how the real
        servo behaves when it stalls.

        This deliberately does not track contact and hold a position short of
        it. That was tried: it grips an aligned cube the same, and drops a
        misaligned one, because a cube rotating flat under the jaws pushes them
        apart and a latched target never follows it back in.
        """
        if not GRIPPER_S_OPEN <= s <= GRIPPER_S_CLOSED:
            raise ValueError(f"gripper S must be {GRIPPER_S_OPEN}-{GRIPPER_S_CLOSED} (got {s})")
        self._gripper_s = float(s)
        positions = np.asarray(finger_positions_for_s(s), dtype=float)
        if teleport:
            self._art.set_joint_positions(positions, joint_indices=self._finger_dofs)
            self._art.set_joint_velocities(
                np.zeros(len(self._finger_dofs)), joint_indices=self._finger_dofs
            )
        self._drive(positions, self._finger_dofs)

    def grip_force_n(self) -> float:
        """What the harder-pressed jaw is pushing with (N).

        The drive is a spring to the commanded opening, clipped at the force
        cap, so this is ``min(Fmax, k * how far the jaw is from its target)``.
        Against any object it reads the cap, which is the point.
        """
        targets = finger_positions_for_s(self._gripper_s)
        behind = max(
            finger - target for finger, target in zip(self.finger_positions(), targets)
        )
        return min(FINGER_EFFORT_N, FINGER_STIFFNESS_N_PER_M * max(0.0, behind))

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


def configure_finger_joints(stage, *, prim_path: str = ARM_PRIM_PATH) -> int:
    """Write the jaw drive and its armature onto both finger joints.

    Called at scene-build time so the values are in the USD when physics parses
    it -- armature in particular has to be there before the articulation is
    created -- and again from ``SimArm`` so a stale scene still gets the drive.
    """
    from pxr import PhysxSchema, UsdPhysics

    n = 0
    for name in FINGER_JOINT_NAMES:
        prim = stage.GetPrimAtPath(f"{prim_path}/Physics/{name}")
        if not prim.IsValid():
            raise RuntimeError(f"no finger joint prim at {prim_path}/Physics/{name}")
        drive = UsdPhysics.DriveAPI.Get(prim, "linear")
        if not drive:
            raise RuntimeError(f"{name} has no linear drive")
        drive.CreateStiffnessAttr(FINGER_STIFFNESS_N_PER_M)
        drive.CreateDampingAttr(FINGER_DAMPING_N_S_PER_M)
        drive.CreateMaxForceAttr(FINGER_EFFORT_N)
        physx_joint = PhysxSchema.PhysxJointAPI.Apply(prim)
        physx_joint.CreateArmatureAttr(FINGER_ARMATURE_KG)
        physx_joint.CreateMaxJointVelocityAttr(FINGER_MAX_SPEED_M_S)
        n += 1
    return n


def couple_finger_joints(stage, *, prim_path: str = ARM_PRIM_PATH) -> str:
    """Tie the jaws to each other so their midpoint stays on the TCP.

    A fixed tendon over the two prismatic axes with gearings +1 / -1 and a rest
    length of zero, standing in for the real gripper's scissor linkage. See
    ``mt4_sim.chain`` for why it is rooted on the wrist rather than on a jaw, and
    for the mimic-joint route that does not work here.

    Build-time only: PhysX reads tendons when it creates the articulation, so
    applying this to a stage that is already simulating does nothing. Returns the
    wrist joint path it rooted on.
    """
    from pxr import PhysxSchema

    root_path = f"{prim_path}/Physics/{ARM_JOINT_NAMES[-1]}"
    root_prim = stage.GetPrimAtPath(root_path)
    if not root_prim.IsValid():
        raise RuntimeError(f"no wrist joint prim at {root_path} to root the jaw tendon on")

    root = PhysxSchema.PhysxTendonAxisRootAPI.Apply(root_prim, FINGER_COUPLING_NAME)
    root.CreateStiffnessAttr(FINGER_COUPLING_STIFFNESS)
    root.CreateDampingAttr(FINGER_COUPLING_DAMPING)
    root.CreateRestLengthAttr(0.0)
    # The jaws' own prismatic stops already bound the opening; a tendon limit on
    # top of them would fight the drive at the ends of its travel.
    root.CreateLimitStiffnessAttr(0.0)
    # Gearing 0: the wrist is here to be a common ancestor of both jaws, not to
    # take part in the sum the tendon holds at zero.
    PhysxSchema.PhysxTendonAxisAPI(root_prim, FINGER_COUPLING_NAME).CreateGearingAttr([0.0])

    for name, gearing in zip(FINGER_JOINT_NAMES, (1.0, -1.0)):
        prim = stage.GetPrimAtPath(f"{prim_path}/Physics/{name}")
        if not prim.IsValid():
            raise RuntimeError(f"no finger joint prim at {prim_path}/Physics/{name}")
        PhysxSchema.PhysxTendonAxisAPI.Apply(prim, FINGER_COUPLING_NAME).CreateGearingAttr(
            [gearing]
        )
    return root_path


def define_physics_material(
    stage, path: str, *, static: float, dynamic: float, restitution: float = 0.0
):
    """A physics material PhysX will actually read, at ``path``.

    ``UsdPhysics.MaterialAPI`` belongs on a ``UsdShade.Material`` prim that
    colliders bind to with the ``physics`` purpose. Applied straight to a
    collider it writes attributes nothing consumes: a 20 mm cube carrying
    ``physics:staticFriction = 1.1`` that way slides down a 45 deg ramp exactly
    as far as a cube with no material at all.
    """
    from pxr import PhysxSchema, UsdPhysics, UsdShade

    material = UsdShade.Material.Define(stage, path)
    api = UsdPhysics.MaterialAPI.Apply(material.GetPrim())
    api.CreateStaticFrictionAttr(static)
    api.CreateDynamicFrictionAttr(dynamic)
    api.CreateRestitutionAttr(restitution)
    PhysxSchema.PhysxMaterialAPI.Apply(material.GetPrim())
    return material


def bind_physics_material(prim, material) -> None:
    """Bind ``material`` to ``prim`` for the ``physics`` purpose only.

    Kept separate from the visual binding so a collider can be wood-coloured
    and rubber-gripped at once.
    """
    from pxr import UsdShade

    UsdShade.MaterialBindingAPI.Apply(prim)
    UsdShade.MaterialBindingAPI(prim).Bind(
        material, UsdShade.Tokens.weakerThanDescendants, "physics"
    )


PAD_MATERIAL_PATH = "/World/Looks/FingerPadPhysics"


def bind_finger_friction(stage, *, prim_path: str = ARM_PRIM_PATH) -> int:
    """Give both finger colliders the rubber-pad friction. Returns how many.

    A grasp on this arm holds by friction alone, so the pad/cube pair is the
    whole grip. Authored onto the scene stage rather than the referenced arm
    layer, so a re-imported arm does not lose it.
    """
    from pxr import Usd, UsdPhysics

    material = define_physics_material(
        stage,
        PAD_MATERIAL_PATH,
        static=FINGER_STATIC_FRICTION,
        dynamic=FINGER_DYNAMIC_FRICTION,
    )
    n = 0
    for link in ("finger_left", "finger_right"):
        root = stage.GetPrimAtPath(link_prim_path(link, arm_prim_path=prim_path))
        if not root.IsValid():
            raise RuntimeError(f"no finger link at {link_prim_path(link, prim_path)}")
        for prim in Usd.PrimRange(root):
            if not prim.HasAPI(UsdPhysics.CollisionAPI):
                continue
            bind_physics_material(prim, material)
            n += 1
    return n


def prepare_gripper_contacts(stage, *, prim_path: str = ARM_PRIM_PATH) -> int:
    """Rewrite finite Newton/PhysX contact attrs on finger collision meshes.

    Returns how many collision prims were updated. Safe to call before
    ``World.reset`` / articulation init and again afterward.
    """
    from pxr import PhysxSchema, Sdf, Usd, UsdPhysics

    n = 0
    for link in ("finger_left", "finger_right"):
        root = stage.GetPrimAtPath(link_prim_path(link, arm_prim_path=prim_path))
        if not root.IsValid():
            continue
        for prim in Usd.PrimRange(root):
            if not prim.HasAPI(UsdPhysics.CollisionAPI):
                continue
            # Author Newton margins even if the schema API has not been applied
            # yet -- the runtime otherwise defaults contactGap to -inf.
            gap = prim.GetAttribute("newton:contactGap")
            if not gap:
                gap = prim.CreateAttribute("newton:contactGap", Sdf.ValueTypeNames.Float)
            gap.Set(0.0)
            margin = prim.GetAttribute("newton:contactMargin")
            if not margin:
                margin = prim.CreateAttribute(
                    "newton:contactMargin", Sdf.ValueTypeNames.Float
                )
            margin.Set(0.002)
            col = PhysxSchema.PhysxCollisionAPI.Apply(prim)
            # Set contact high first so a -inf rest cannot fail validation.
            col.CreateContactOffsetAttr().Set(0.02)
            col.CreateRestOffsetAttr().Set(0.0)
            col.CreateContactOffsetAttr().Set(0.002)
            n += 1
    return n


def desk_z_mm() -> float:
    """The wood, in the arm's frame -- the plane the base stands on."""
    return DESK_Z_MM


def tcp_above_desk_mm(state: ArmState) -> float:
    """How far the TCP is above the height that grips something on the desk.

    Zero means the arm is at ``TCP_GRIP_Z_MM``, where the tong tips ride
    ``TONG_CLEARANCE_MM`` clear of the wood and straddle a cube lying on it.
    This is the number a caller thinking in "clearance above the table" wants,
    not the TCP's raw Z. The tips reach the wood ``TONG_CLEARANCE_MM`` lower
    still, at ``GROUND_Z_MM``.
    """
    return state.tcp_mm[2] - TCP_GRIP_Z_MM


def deg(radians: float) -> float:
    return math.degrees(radians)
