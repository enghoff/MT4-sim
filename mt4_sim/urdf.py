"""Emit the MT4 URDF from the control repo's kinematic constants.

Link frames and joint origins come from :mod:`mt4_sim.chain`; nothing about the
arm's geometry is restated here. What this module adds is the part every URDF
needs and kinematics does not describe: shapes to draw, shapes to collide with,
and masses.

Shapes are boxes sized from the CAD part bounding boxes in
``vendor/MT4-STL``. They give the right proportions and cheap convex collision.
Masses are nominal -- the arm is position-driven, so they set how hard the
solver has to work, not where the TCP ends up.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from pathlib import Path
from xml.etree import ElementTree as ET

from mt4_sim.chain import (
    CENCER_HEIGHT,
    CENCER_OFFSET,
    FINGER_DAMPING_N_S_PER_M,
    FINGER_EFFORT_N,
    FINGER_MAX_SPEED_M_S,
    GRIPPER_S_OPEN,
    HEAD_HEIGHT,
    HEAD_OFFSET,
    LINKAGE1,
    LINKAGE2,
    MM,
    TONG_REACH_MM,
    span_mm_for_s,
    urdf_limits,
)

# Finger travel per side. The firmware's open limit S=120 is the widest the
# jaws go, so that is the prismatic stop.
FINGER_TRAVEL_M = 0.5 * span_mm_for_s(GRIPPER_S_OPEN) * MM

# The blade's cross-section: 20mm across the pad face, 10mm through. Its third
# dimension is the long one and comes from the gripper's height, below.
FINGER_WIDTH_MM = 20.0
FINGER_THICK_MM = 10.0
# The tongs hang ``TONG_REACH_MM`` below the TCP -- the real gripper's measured
# height from J4 to the tips. That is the real gripper's proportions: long
# blades, so the servo housing and the head plate ride well clear of whatever is
# being picked. The short jaws this replaces put the gripper body's underside
# exactly on a 20 mm cube's top face, pinning the cube against the desk while
# the jaws tried to turn it.
FINGER_ABOVE_TCP_MM = 12.0
FINGER_BELOW_TCP_MM = TONG_REACH_MM
FINGER_HEIGHT_MM = FINGER_ABOVE_TCP_MM + FINGER_BELOW_TCP_MM
# Centre of the blade in the gripper frame: the span -BELOW .. +ABOVE.
FINGER_CENTRE_Z_MM = (FINGER_ABOVE_TCP_MM - FINGER_BELOW_TCP_MM) / 2.0
# The prismatic joint sits on the *inner* face of each blade -- the face the
# span model measures between. ``finger_positions_for_s`` is half the clear
# opening, so the box must be offset outward by half its thickness; centering
# it on the joint ate 10mm of opening and left a 20mm cube barely fitting at
# the calibrated open S.
FINGER_INNER_TO_CENTER_MM = FINGER_THICK_MM / 2.0


@dataclass(frozen=True)
class Box:
    """A box visual/collision shape: size and centre in the link's frame (mm).

    ``material`` names a colour from :data:`MATERIALS` when this box is not the
    same colour as the rest of its link -- the shoulder steppers are bare motors
    bolted to a painted casting, and drawing them in the casting's orange is the
    difference between the arm reading as an MT4 and reading as a lump.
    """

    size: tuple[float, float, float]
    centre: tuple[float, float, float] = (0.0, 0.0, 0.0)
    material: str | None = None

    def size_m(self) -> str:
        return " ".join(f"{v * MM:.6f}" for v in self.size)

    def origin_m(self) -> str:
        return " ".join(f"{v * MM:.6f}" for v in self.centre)


def _box_inertia(mass: float, size_mm: tuple[float, float, float]) -> tuple[float, ...]:
    x, y, z = (v * MM for v in size_mm)
    k = mass / 12.0
    return (k * (y * y + z * z), k * (x * x + z * z), k * (x * x + y * y))


@dataclass(frozen=True)
class Link:
    name: str
    mass: float
    shapes: tuple[Box, ...]


def _lumped_inertia(link: Link) -> tuple[tuple[float, float, float], float, tuple[float, ...]]:
    """A link's boxes as one inertial: centre of mass (m), mass, diagonal inertia.

    URDF allows a single ``<inertial>`` per link. Each box takes a share of the
    link's mass by volume; the shares combine about their common centre of mass
    by the parallel-axis theorem.
    """
    volumes = [b.size[0] * b.size[1] * b.size[2] for b in link.shapes]
    total_volume = sum(volumes)
    masses = [link.mass * v / total_volume for v in volumes]

    com = tuple(
        sum(m * b.centre[i] * MM for m, b in zip(masses, link.shapes)) / link.mass
        for i in range(3)
    )

    inertia = [0.0, 0.0, 0.0]
    for m, box in zip(masses, link.shapes):
        own = _box_inertia(m, box.size)
        d = [box.centre[i] * MM - com[i] for i in range(3)]
        for axis in range(3):
            others = [d[i] for i in range(3) if i != axis]
            inertia[axis] += own[axis] + m * (others[0] ** 2 + others[1] ** 2)

    return com, link.mass, tuple(inertia)


# The two bodies J1 joins, measured off the STEP assembly rather than sketched.
# `tools/step_assembly.py` prints the boxes these come from; the CAD's own frame
# has +Y up and its origin on the J1 axis 4mm above the feet, so every number
# below is its assembly reading converted by
#
#     robot x = CAD x + 20        (the J1 bore sits at CAD x = -20)
#     robot y = CAD z
#     robot z = CAD y + 4         (the underside of the feet is the desk)
#
# The base is *not* centred on the J1 axis: the 110x110 extrusion stands 20mm
# forward of it. That asymmetry is load-bearing for how the arm looks -- centred,
# the pedestal reads as a plinth the column grows out of; offset, it reads as a
# box the column turns on the back of.
_BASE_FORWARD_MM = 20.0

# Heights above the desk, bottom to top. These are spans, not a partition: the
# pedestal extrusion starts 1mm inside the foot plate and the yoke starts 1mm
# clear of the shroud, both as the CAD has them.
_FOOT_MM = (0.0, 5.0)       # 2001-03 底板, the plate the rubber feet are under
_PEDESTAL_MM = (4.0, 54.0)  # 2001-01 底盒型材110x110 + 2001-02 顶板 v1.1
_SHROUD_MM = (54.0, 75.0)   # 2000-04 一轴遮罩, the cover over the J1 bearing
# The rotating body. Its side plates run from the shroud right past the shoulder
# pivot, and the two steppers driving J2 and J3 hang off them sideways -- at
# 184mm tip to tip, the widest thing on the machine.
_YOKE_MM = (76.0, 160.0)    # 0200-01/02/03, the rotating seat and its plates
_MOTOR_MM = (87.0, 129.0)   # 42CM08-60 x2
_MOTOR_HALF_Y_MM = 50.0     # centre of each 84mm-wide motor, out from the midline


def _span(lo_hi: tuple[float, float]) -> tuple[float, float]:
    """A (low, high) span as the (size, centre) a :class:`Box` wants."""
    lo, hi = lo_hi
    return hi - lo, (lo + hi) / 2.0


_FOOT_H, _FOOT_C = _span(_FOOT_MM)
_PEDESTAL_H, _PEDESTAL_C = _span(_PEDESTAL_MM)
_SHROUD_H, _SHROUD_C = _span(_SHROUD_MM)
_YOKE_H, _YOKE_C = _span(_YOKE_MM)
_MOTOR_H, _MOTOR_C = _span(_MOTOR_MM)

LINKS: tuple[Link, ...] = (
    # `base_link`'s frame is the J1 axis on the desk: the arm stands on the wood,
    # so z = 0 is both. `mt4_sim.rig` cuts the tabletop back around the footprint
    # so the wood is not a static collider coincident with the feet.
    Link(
        "base_link",
        1.20,
        (
            Box((110.0, 130.0, _FOOT_H), (_BASE_FORWARD_MM, 0.0, _FOOT_C)),
            Box((110.0, 110.0, _PEDESTAL_H), (_BASE_FORWARD_MM, 0.0, _PEDESTAL_C)),
            # The shroud over the J1 bearing, narrower than the pedestal it sits
            # on. This step is what keeps the base looking squat: without it the
            # pedestal has to run the full 75mm at full width, and a 110-wide box
            # 75 tall reads far taller than the real machine.
            Box((85.0, 72.0, _SHROUD_H), (_BASE_FORWARD_MM - 1.0, 0.0, _SHROUD_C)),
        ),
    ),
    # The rotating column carries on from where the static base stops. Its frame
    # is the shoulder pivot -- CENCER_OFFSET forward of J1 and CENCER_HEIGHT up
    # -- so everything here is measured back and down from there.
    Link(
        "link1_column",
        0.50,
        (
            Box((106.0, 64.0, _YOKE_H), (20.0 - CENCER_OFFSET, 0.0, _YOKE_C - CENCER_HEIGHT)),
            Box(
                (42.0, 84.0, _MOTOR_H),
                (-CENCER_OFFSET, _MOTOR_HALF_Y_MM, _MOTOR_C - CENCER_HEIGHT),
                material="motor",
            ),
            Box(
                (42.0, 84.0, _MOTOR_H),
                (-CENCER_OFFSET, -_MOTOR_HALF_Y_MM, _MOTOR_C - CENCER_HEIGHT),
                material="motor",
            ),
        ),
    ),
    Link("upper_arm", 0.28, (Box((LINKAGE1, 30.0, 40.0), (LINKAGE1 / 2.0, 0.0, 0.0)),)),
    Link("forearm", 0.22, (Box((LINKAGE2, 24.0, 30.0), (LINKAGE2 / 2.0, 0.0, 0.0)),)),
    # The level plate. It reaches from behind the wrist pivot out over the TCP,
    # which is HEAD_OFFSET along it and HEAD_HEIGHT below.
    Link(
        "head",
        0.10,
        (Box((HEAD_OFFSET + 23.0, 26.0, 10.0), ((HEAD_OFFSET - 23.0) / 2.0, 0.0, 2.0)),),
    ),
    # The gripper's frame is the TCP -- the pads themselves. HEAD_HEIGHT puts
    # that only 14.43mm under the wrist pivot, so the jaws are all there is room
    # for below the plate and the servo housing sits on top of it.
    Link("gripper_base", 0.12, (Box((38.0, 32.0, 20.0), (0.0, 0.0, 30.0)),)),
    # Nominal, like every other mass here, but scaled for a blade this long.
    Link(
        "finger_left",
        0.06,
        # +Y is outward; origin is the inner (pad) face, box grows outward.
        (
            Box(
                (FINGER_WIDTH_MM, FINGER_THICK_MM, FINGER_HEIGHT_MM),
                (0.0, FINGER_INNER_TO_CENTER_MM, FINGER_CENTRE_Z_MM),
            ),
        ),
    ),
    Link(
        "finger_right",
        0.06,
        # -Y is outward on this side (joint axis is (0,-1,0)).
        (
            Box(
                (FINGER_WIDTH_MM, FINGER_THICK_MM, FINGER_HEIGHT_MM),
                (0.0, -FINGER_INNER_TO_CENTER_MM, FINGER_CENTRE_Z_MM),
            ),
        ),
    ),
)


@dataclass(frozen=True)
class Joint:
    name: str
    parent: str
    child: str
    axis: tuple[float, float, float]
    origin_mm: tuple[float, float, float]
    kind: str = "revolute"


# Rotations of the arm's vertical plane are about -Y so a positive angle lifts
# the link, matching the model angles' sign. See mt4_sim.chain for the table.
JOINTS: tuple[Joint, ...] = (
    Joint("j1_base_yaw", "base_link", "link1_column", (0, 0, 1), (0.0, 0.0, CENCER_HEIGHT)),
    Joint("j2_shoulder", "link1_column", "upper_arm", (0, -1, 0), (CENCER_OFFSET, 0.0, 0.0)),
    Joint("j3_elbow", "upper_arm", "forearm", (0, -1, 0), (LINKAGE1, 0.0, 0.0)),
    Joint("j_head_level", "forearm", "head", (0, -1, 0), (LINKAGE2, 0.0, 0.0)),
    Joint(
        "j4_wrist_roll",
        "head",
        "gripper_base",
        (0, 0, 1),
        (HEAD_OFFSET, 0.0, -HEAD_HEIGHT),
    ),
    Joint("j_finger_left", "gripper_base", "finger_left", (0, 1, 0), (0.0, 0.0, 0.0), "prismatic"),
    Joint(
        "j_finger_right", "gripper_base", "finger_right", (0, -1, 0), (0.0, 0.0, 0.0), "prismatic"
    ),
)

MATERIALS = {
    "base_link": (0.16, 0.17, 0.19),
    "link1_column": (0.90, 0.36, 0.06),
    "upper_arm": (0.90, 0.36, 0.06),
    "forearm": (0.90, 0.36, 0.06),
    "head": (0.16, 0.17, 0.19),
    "gripper_base": (0.16, 0.17, 0.19),
    "finger_left": (0.75, 0.76, 0.78),
    "finger_right": (0.75, 0.76, 0.78),
    # Not a link: the bare shoulder steppers, which are black where the casting
    # they bolt to is orange. See ``Box.material``.
    "motor": (0.12, 0.12, 0.13),
}


def _mm3(v: tuple[float, float, float]) -> str:
    return " ".join(f"{c * MM:.6f}" for c in v)


def build_urdf() -> ET.ElementTree:
    """The MT4 as a URDF tree. ``base_link`` is the root."""
    robot = ET.Element("robot", {"name": "mt4"})
    limits = urdf_limits()

    for name, rgb in MATERIALS.items():
        mat = ET.SubElement(robot, "material", {"name": f"mt4_{name}"})
        ET.SubElement(mat, "color", {"rgba": f"{rgb[0]} {rgb[1]} {rgb[2]} 1.0"})

    for link in LINKS:
        el = ET.SubElement(robot, "link", {"name": link.name})

        com, mass, (ixx, iyy, izz) = _lumped_inertia(link)
        inertial = ET.SubElement(el, "inertial")
        ET.SubElement(inertial, "origin", {"xyz": " ".join(f"{c:.6f}" for c in com), "rpy": "0 0 0"})
        ET.SubElement(inertial, "mass", {"value": f"{mass:.6f}"})
        ET.SubElement(
            inertial,
            "inertia",
            {
                "ixx": f"{ixx:.9f}",
                "iyy": f"{iyy:.9f}",
                "izz": f"{izz:.9f}",
                "ixy": "0",
                "ixz": "0",
                "iyz": "0",
            },
        )

        for box in link.shapes:
            for tag in ("visual", "collision"):
                node = ET.SubElement(el, tag)
                ET.SubElement(node, "origin", {"xyz": box.origin_m(), "rpy": "0 0 0"})
                geom = ET.SubElement(node, "geometry")
                ET.SubElement(geom, "box", {"size": box.size_m()})
                if tag == "visual":
                    ET.SubElement(
                        node, "material", {"name": f"mt4_{box.material or link.name}"}
                    )

    for joint in JOINTS:
        el = ET.SubElement(robot, "joint", {"name": joint.name, "type": joint.kind})
        ET.SubElement(el, "parent", {"link": joint.parent})
        ET.SubElement(el, "child", {"link": joint.child})
        ET.SubElement(el, "origin", {"xyz": _mm3(joint.origin_mm), "rpy": "0 0 0"})
        ET.SubElement(el, "axis", {"xyz": " ".join(str(a) for a in joint.axis)})
        if joint.kind == "prismatic":
            # Jaw force cap: enough to rotate an 8 g cube into face alignment,
            # not enough to crush through it when the host closes past contact.
            # The velocity ceiling is load-bearing -- see FINGER_MAX_SPEED_M_S
            # for why a slow jaw stops squaring misaligned cubes up.
            lo, hi, eff, vel = 0.0, FINGER_TRAVEL_M, FINGER_EFFORT_N, FINGER_MAX_SPEED_M_S
        else:
            lo, hi = limits[joint.name]
            # Well above the ~1 Nm gravity torque at the shoulder, so the drive
            # is limited by its stiffness rather than clipping on force.
            eff, vel = 200.0, math.radians(120.0)
        ET.SubElement(
            el,
            "limit",
            {"lower": f"{lo:.6f}", "upper": f"{hi:.6f}", "effort": str(eff), "velocity": f"{vel:.4f}"},
        )
        # Drive damping comes from here, not from the importer's
        # override_joint_damping, which the converter ignores in favour of the
        # URDF value. These are the numbers that land in USD verbatim (N.m.s per
        # degree for revolute, N.s/m for prismatic) and both are well past
        # critical, so a stiff position drive settles without ringing.
        damping = str(FINGER_DAMPING_N_S_PER_M) if joint.kind == "prismatic" else "4.0"
        ET.SubElement(el, "dynamics", {"damping": damping, "friction": "0.0"})

    return ET.ElementTree(robot)


def write_urdf(path: Path) -> Path:
    tree = build_urdf()
    ET.indent(tree, space="  ")
    path.parent.mkdir(parents=True, exist_ok=True)
    tree.write(path, encoding="utf-8", xml_declaration=True)
    return path
