"""The MT4's parallel linkage expressed as a serial URDF chain.

The arm is a palletizer: J2 sets the upper arm's *absolute* angle, J3 sets the
forearm's *absolute* angle through a pair of link rods, and the head platform
stays level however the arm folds. A URDF chain has only relative joint angles,
so the model angles the MT4's control stack speaks in (``q1..q4``) are not the
joint positions this articulation takes. :func:`urdf_from_model` converts.

The chain, with every rotation of the arm's vertical plane taken about ``-Y``
so that a positive angle lifts the link:

===================  ==========================  ==================  ================
URDF joint           origin in parent (m)        axis                position
===================  ==========================  ==================  ================
``j1_base_yaw``      (0, 0, 0)                   +Z                  ``q1``
``j2_shoulder``      (0.045, 0, 0.140)           -Y                  ``q2``
``j3_elbow``         (0.130, 0, 0)               -Y                  ``q3 - q2``
``j_head_level``     (0.150, 0, 0)               -Y                  ``-q3``
``j4_wrist_roll``    (0.035, 0, -0.01443)        +Z                  ``q4``
===================  ==========================  ==================  ================

``j_head_level`` is the link rods doing their job: it cancels the forearm's
rotation so the head hangs level, which is why ``HEAD_OFFSET`` is a horizontal
offset in the FK and not a rotating one. ``j4_wrist_roll``'s origin *is* the
TCP -- the wrist roll axis passes through it, which is why
``mt4_jog.kinematics.fk_tcp`` has no ``q4`` term.

Composing the chain reproduces ``fk_tcp`` exactly::

    radial = 45 + 130 cos q2 + 150 cos q3 + 35
    z      = 140 + 130 sin q2 + 150 sin q3 - 14.43

``tests/test_fk_parity.py`` checks that against the control repo's own FK over
a joint sweep.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

from mt4_sim import calibration, mt4_repo  # noqa: F401  (sys.path bootstrap)

from mt4_jog.joints import (
    GRIPPER_S_CLOSED,
    GRIPPER_S_OPEN,
    GROUND_Z_MM,
    JOINT_SOFT_MAX_STEPS,
    JOINT_SOFT_MIN_STEPS,
)
from mt4_jog.kinematics import (
    CENCER_HEIGHT,
    CENCER_OFFSET,
    HEAD_HEIGHT,
    HEAD_OFFSET,
    HOME_J1_DEG,
    HOME_J2_DEG,
    HOME_J3_DEG,
    HOME_J4_DEG,
    J_STEP_SIGN,
    LINKAGE1,
    LINKAGE2,
    STEPS_PER_DEG,
    JointAnglesDeg,
)

MM = 0.001

# `Calibration.table_z` (122) is a **TCP** height: the Z the arm is commanded to
# in order to grip something lying on the table, measured by touching the tags.
# It is not where the wood is. The gripper's tongs hang below the TCP, and they
# are long enough that the surface they reach is the plane the arm's own base
# stands on -- so the MT4 sits on the desk, which is the only arrangement in
# which all three of the firmware's numbers make sense at once:
#
#   CENCER_HEIGHT = 140    the shoulder pivot, 140 mm above the desk it stands on
#   table_z       = 122    the TCP height whose tong tips are on that desk
#   GROUND_Z_MM   = 115    a floor 7 mm of tong-tip *below* the wood, which is
#                          what a floor "a few mm under the table" should be
#
# The alternative -- wood at z = 122 with the arm's base buried beneath it --
# is what this modelled before, and it put the gripper body exactly on a 20 mm
# cube's top face on every pick.
TCP_GRIP_Z_MM = calibration.table_z_mm()
DESK_Z_MM = calibration.desk_surface_z_mm()
# How far the tongs reach below the TCP. Not a free parameter: it is the whole
# distance between the two planes above.
TONG_REACH_MM = TCP_GRIP_Z_MM - DESK_Z_MM

# Jaw span model from `mt4_vision.calib` (grip_span_s_at_zero_mm /
# grip_span_s_per_mm, measured on the real gripper): span_mm = (212.3 - S) /
# 1.881. S = 140 opens 38.4mm; span reaches zero at S = 212.3 and the servo
# keeps pulling past that against the object.
GRIP_SPAN_S_AT_ZERO_MM = 212.3
GRIP_SPAN_S_PER_MM = 1.881

# The park pose the arm holds after homing (firmware kinematics.h): counters at
# +(j2_pull, j3_pull) put the model at these angles, FK (190.0, 0, 225.6).
PARK_Q2_DEG = 107.0
PARK_Q3_DEG = -9.3


@dataclass(frozen=True)
class JointLimit:
    lower_deg: float
    upper_deg: float


def _limit_from_steps(index: int) -> JointLimit:
    """Soft step limits -> model-angle limits, ordered low to high."""
    home = (HOME_J1_DEG, HOME_J2_DEG, HOME_J3_DEG, HOME_J4_DEG)[index]
    scale = J_STEP_SIGN[index] / STEPS_PER_DEG[index]
    a = home + scale * JOINT_SOFT_MIN_STEPS[index]
    b = home + scale * JOINT_SOFT_MAX_STEPS[index]
    return JointLimit(min(a, b), max(a, b))


# Model-angle soft limits for q1..q4, from the firmware's step limits.
Q_LIMITS: tuple[JointLimit, JointLimit, JointLimit, JointLimit] = tuple(
    _limit_from_steps(i) for i in range(4)
)  # type: ignore[assignment]


def park_pose() -> JointAnglesDeg:
    return JointAnglesDeg(HOME_J1_DEG, PARK_Q2_DEG, PARK_Q3_DEG, HOME_J4_DEG)


# --------------------------------------------------------------------------
# Model angles <-> URDF joint positions
# --------------------------------------------------------------------------

ARM_JOINT_NAMES = (
    "j1_base_yaw",
    "j2_shoulder",
    "j3_elbow",
    "j_head_level",
    "j4_wrist_roll",
)
FINGER_JOINT_NAMES = ("j_finger_left", "j_finger_right")


def urdf_from_model(q: JointAnglesDeg) -> tuple[float, float, float, float, float]:
    """Model angles (deg) -> the five arm joint positions (rad), chain order."""
    q1, q2, q3, q4 = map(math.radians, (q.j1, q.j2, q.j3, q.j4))
    return (q1, q2, q3 - q2, -q3, q4)


def model_from_urdf(pos: tuple[float, float, float, float, float]) -> JointAnglesDeg:
    """Inverse of :func:`urdf_from_model`; ``j_head_level`` is redundant."""
    q1, q2, q3_rel, _level, q4 = pos
    return JointAnglesDeg(
        math.degrees(q1),
        math.degrees(q2),
        math.degrees(q2 + q3_rel),
        math.degrees(q4),
    )


def urdf_limits() -> dict[str, tuple[float, float]]:
    """Per-joint (lower, upper) in radians for the URDF's arm joints.

    ``j3_elbow`` and ``j_head_level`` carry no limit of their own on the real
    arm -- they are the derived halves of J3 -- so their range is whatever the
    q2/q3 boxes imply.
    """
    q1, q2, q3, q4 = Q_LIMITS
    return {
        "j1_base_yaw": (math.radians(q1.lower_deg), math.radians(q1.upper_deg)),
        "j2_shoulder": (math.radians(q2.lower_deg), math.radians(q2.upper_deg)),
        "j3_elbow": (
            math.radians(q3.lower_deg - q2.upper_deg),
            math.radians(q3.upper_deg - q2.lower_deg),
        ),
        "j_head_level": (math.radians(-q3.upper_deg), math.radians(-q3.lower_deg)),
        "j4_wrist_roll": (math.radians(q4.lower_deg), math.radians(q4.upper_deg)),
    }


# --------------------------------------------------------------------------
# Gripper
# --------------------------------------------------------------------------

MAX_SPAN_MM = (GRIP_SPAN_S_AT_ZERO_MM - GRIPPER_S_OPEN) / GRIP_SPAN_S_PER_MM


def span_mm_for_s(s: float) -> float:
    """Firmware gripper S -> jaw opening (mm), clamped to the physical range.

    S above ``GRIP_SPAN_S_AT_ZERO_MM`` is the servo pulling past contact, which
    the model reports as a negative span and the jaws cannot do; it clamps to
    closed.
    """
    span = (GRIP_SPAN_S_AT_ZERO_MM - s) / GRIP_SPAN_S_PER_MM
    return max(0.0, min(MAX_SPAN_MM, span))


def s_for_span_mm(span_mm: float) -> float:
    return GRIP_SPAN_S_AT_ZERO_MM - span_mm * GRIP_SPAN_S_PER_MM


def finger_positions_for_s(s: float) -> tuple[float, float]:
    """Gripper S -> prismatic positions (m): half the *clear* opening each side.

    Joint origin is the inner face of each blade (see ``mt4_sim.urdf``), matching
    the live span calibration which measures face-to-face gap, not centre-to-centre.
    """
    half = 0.5 * span_mm_for_s(s) * MM
    return (half, half)


# --------------------------------------------------------------------------
# The jaw drive
# --------------------------------------------------------------------------
#
# The jaws are a **constant-force closer**: a soft spring commanded shut, with
# the force cap doing the gripping. The host closes past contact (S=255 is a
# negative span), so the drive's position error is always large and the force
# is always the cap -- ``FINGER_EFFORT_N`` *is* the grip force, whatever the
# object's width. That is also the better model of the real servo, which
# stalls against the object and holds.
#
# The force is sized from the task, not from "as hard as the solver allows".
# A 20 mm cube is 8 g, so its weight is 0.078 N and:
#
#   hold it against gravity   N >= mg / (2 mu) = 0.078 / 2.2  = 0.036 N
#   rotate a yawed one flat   N >= desk friction moment / arm = 0.02 N
#   lift it on a moving arm   the above with a 10x margin     ~ 0.4 N
#
# 1.5 N is ~19x the cube's weight and ~40x the static hold requirement, which
# is margin enough for the arm to fling it around at the accelerations a
# position drive produces. Going higher buys nothing and costs stability: the
# velocity a capped drive can inject into a 20 g finger in one substep is
# ``F * dt / m``, which at 240 Hz is 0.31 m/s here and was **2.5 m/s** at the
# 12 N cap this replaces. Delivered through a finger into an 8 g cube, that
# impulse is what threw cubes across the desk.
#
# Stiffness and damping only shape the approach, not the grip force. Critical
# damping is 2*sqrt(k*m) = 2*sqrt(1000 * 0.02) = 8.94, so 12.0 is zeta = 1.34:
# the jaw settles without ringing, and its terminal closing speed under the
# cap is F/c = 0.125 m/s, comfortably faster than the firmware's own 360 S/s
# sweep moves the target (~0.096 m/s per jaw).
FINGER_STIFFNESS_N_PER_M = 1000.0
FINGER_EFFORT_N = 1.5
FINGER_DAMPING_N_S_PER_M = 12.0  # zeta = 1.34 at m = 0.02 kg (critical = 8.94)

# Armature: extra inertia in *joint* space, and the thing that makes the above
# usable at 60 Hz.
#
# A drive clipped at ``maxForce`` has no damping left -- the ``c * v`` term is
# clipped away with the rest -- so in sustained contact it stops being a spring
# and becomes a constant-force actuator with nothing opposing velocity. It then
# bounces off the contact at ``F * dt / m`` per step: 1.25 m/s for a 20 g finger
# at 1.5 N and 60 Hz. Measured, the jaws chattered over ~0.8 mm and the cube's
# angular velocity swung +/-280 deg/s about zero, so the couple that should have
# squared the cube up averaged out to nothing.
#
# This is not a fudge factor. The blade is driven through a geared servo, and a
# gearbox reflects the rotor's inertia to the output by the square of its ratio,
# so the mass the drive actually has to accelerate is far more than the blade's
# own 20 g. Modelling that is what makes the contact quiet, and it is also what
# lets the jaws do useful work on a misaligned cube -- measured on a cube 20 deg
# off square, closing at 60 Hz:
#
#     0.02 kg   jaws jitter, cube turns  1.2 of 20 deg, jams on corners at 27.0
#     0.30 kg   quiet contact, cube turns 20.1 of 20 deg, face grip at 19.6
#     1.00 kg   as above, but the cube ends further off the jaws' centre
#
# Too light and the jaws rattle against the cube without ever pushing it; the
# couple that should square it up averages to nothing over the bounce.
FINGER_ARMATURE_KG = 0.30

# How fast a jaw may travel. The firmware's own 360 S/s sweep moves each jaw at
# ~0.096 m/s, so this only has to clear that; 0.15 leaves half again in hand
# without the jaws slamming shut like a trap.
#
# It used to have to be 0.5. With the short jaws the gripper body's underside
# rested on the cube, and the only thing that could turn a misaligned one was
# the *momentum* of a fast-closing blade -- at 0.12 m/s a cube 20 deg off square
# turned 0.9 deg and stayed jammed on its corners. Full-length tongs turn it
# with a static couple instead, so the speed stopped mattering: 20.0 deg of 20
# at 0.5 m/s, 19.9 at 0.1.
FINGER_MAX_SPEED_M_S = 0.15

# Pad / cube contact. Rubber pads on a plastic cube; the grip holds by friction
# alone, and at 1.5 N even mu = 0.03 would carry the cube's weight, so this is
# not a number the grip is sensitive to.
FINGER_STATIC_FRICTION = 1.2
FINGER_DYNAMIC_FRICTION = 1.1


__all__ = [
    "ARM_JOINT_NAMES",
    "CENCER_HEIGHT",
    "CENCER_OFFSET",
    "DESK_Z_MM",
    "TCP_GRIP_Z_MM",
    "TONG_REACH_MM",
    "FINGER_ARMATURE_KG",
    "FINGER_DAMPING_N_S_PER_M",
    "FINGER_DYNAMIC_FRICTION",
    "FINGER_EFFORT_N",
    "FINGER_JOINT_NAMES",
    "FINGER_MAX_SPEED_M_S",
    "FINGER_STATIC_FRICTION",
    "FINGER_STIFFNESS_N_PER_M",
    "GRIPPER_S_CLOSED",
    "GRIPPER_S_OPEN",
    "GROUND_Z_MM",
    "HEAD_HEIGHT",
    "HEAD_OFFSET",
    "JointAnglesDeg",
    "LINKAGE1",
    "LINKAGE2",
    "MAX_SPAN_MM",
    "MM",
    "Q_LIMITS",
    "finger_positions_for_s",
    "model_from_urdf",
    "park_pose",
    "s_for_span_mm",
    "span_mm_for_s",
    "urdf_from_model",
    "urdf_limits",
]
