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

# Desk surface in the arm's home-angle frame, read from the live rig's
# calibration rather than restated: `Calibration.table_z` is both the table
# surface's Z and the TCP Z that grips a cube sitting on it, measured by
# touching the tags with the TCP. `mt4_jog.joints.GROUND_Z_MM` (115) is the
# firmware's soft floor, deliberately a few mm under the surface so a pick at
# table_z presses into the desk instead of stopping short of it.
DESK_Z_MM = calibration.table_z_mm()

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
    """Gripper S -> the two prismatic finger positions (m), each half the span."""
    half = 0.5 * span_mm_for_s(s) * MM
    return (half, half)


__all__ = [
    "ARM_JOINT_NAMES",
    "CENCER_HEIGHT",
    "CENCER_OFFSET",
    "DESK_Z_MM",
    "FINGER_JOINT_NAMES",
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
