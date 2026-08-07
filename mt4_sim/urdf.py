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
    DESK_Z_MM,
    GRIPPER_S_OPEN,
    HEAD_HEIGHT,
    HEAD_OFFSET,
    LINKAGE1,
    LINKAGE2,
    MAX_SPAN_MM,
    MM,
    span_mm_for_s,
    urdf_limits,
)

# Finger travel per side. The firmware's open limit S=120 is the widest the
# jaws go, so that is the prismatic stop.
FINGER_TRAVEL_M = 0.5 * span_mm_for_s(GRIPPER_S_OPEN) * MM

FINGER_LENGTH_MM = 26.0
FINGER_THICK_MM = 10.0
# The pads sit at the TCP and the jaws rise from there. A cube on the desk is
# gripped with the TCP at table height, so a jaw reaching *below* the pads would
# be driven into the desk on every pick.
FINGER_HEIGHT_MM = 12.0


@dataclass(frozen=True)
class Box:
    """A box visual/collision shape: size and centre in the link's frame (mm)."""

    size: tuple[float, float, float]
    centre: tuple[float, float, float] = (0.0, 0.0, 0.0)

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


# The static base is the arm's own column, drawn at the height the CAD gives it:
# 140mm from its foot up to the shoulder pivot, which puts the foot on the
# modelling plane at z = 0. `base_link`'s frame *is* that plane.
#
# Nothing about this is tied to the desk. The desk surface is 120mm up, so the
# column passes through that height -- which is why `mt4_sim.rig` keeps the desk
# clear of the base's footprint instead of running it underneath. A tabletop
# through the column would be a static collider inside the articulation's
# swept volume, and the base yaw would jam solid against it.
_BASE_TOP_MM = 76.0
_FOOT_H = 8.0

LINKS: tuple[Link, ...] = (
    Link(
        "base_link",
        1.20,
        (
            Box((140.0, 120.0, _FOOT_H), (0.0, 0.0, _FOOT_H / 2.0)),
            Box(
                (104.0, 92.0, _BASE_TOP_MM - _FOOT_H),
                (0.0, 0.0, (_BASE_TOP_MM + _FOOT_H) / 2.0),
            ),
        ),
    ),
    # The rotating column carries on from where the static base stops up to the
    # shoulder yoke. Its frame is the shoulder pivot at z = CENCER_HEIGHT, so
    # everything here is measured down from there.
    Link(
        "link1_column",
        0.50,
        (
            Box(
                (96.0, 84.0, CENCER_HEIGHT + 2.0 - _BASE_TOP_MM),
                (0.0, 0.0, (_BASE_TOP_MM - CENCER_HEIGHT + 2.0) / 2.0),
            ),
            Box((62.0, 64.0, 44.0), (CENCER_OFFSET - 17.0, 0.0, 0.0)),
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
    Link(
        "finger_left",
        0.02,
        (Box((FINGER_LENGTH_MM, FINGER_THICK_MM, FINGER_HEIGHT_MM), (0.0, 0.0, 6.0)),),
    ),
    Link(
        "finger_right",
        0.02,
        (Box((FINGER_LENGTH_MM, FINGER_THICK_MM, FINGER_HEIGHT_MM), (0.0, 0.0, 6.0)),),
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
                    ET.SubElement(node, "material", {"name": f"mt4_{link.name}"})

    for joint in JOINTS:
        el = ET.SubElement(robot, "joint", {"name": joint.name, "type": joint.kind})
        ET.SubElement(el, "parent", {"link": joint.parent})
        ET.SubElement(el, "child", {"link": joint.child})
        ET.SubElement(el, "origin", {"xyz": _mm3(joint.origin_mm), "rpy": "0 0 0"})
        ET.SubElement(el, "axis", {"xyz": " ".join(str(a) for a in joint.axis)})
        if joint.kind == "prismatic":
            # Jaw force cap: a grasp squeezes, and 20N across a 20mm cube is
            # firm without launching it.
            lo, hi, eff, vel = 0.0, FINGER_TRAVEL_M, 20.0, 0.1
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
        damping = "200.0" if joint.kind == "prismatic" else "4.0"
        ET.SubElement(el, "dynamics", {"damping": damping, "friction": "0.0"})

    return ET.ElementTree(robot)


def write_urdf(path: Path) -> Path:
    tree = build_urdf()
    ET.indent(tree, space="  ")
    path.parent.mkdir(parents=True, exist_ok=True)
    tree.write(path, encoding="utf-8", xml_declaration=True)
    return path
