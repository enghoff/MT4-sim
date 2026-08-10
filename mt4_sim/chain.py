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

# The physics step every ``World`` in this project is built with.
#
# It lives here rather than in ``mt4_sim.scene`` because the jaw servo's design
# depends on it: the loop's reference may lead the blades by one wind-up length,
# so the blades cannot sweep faster than ``FINGER_WINDUP_M / dt`` and the step
# rate is part of that bound. ``scene`` cannot be imported without the Kit
# runtime, and the checks that assert the bound run without one.
PHYSICS_HZ = 60.0


def physics_dt() -> float:
    """The physics step every ``World`` in this project must be built with."""
    return 1.0 / PHYSICS_HZ


# `Calibration.table_z` (122) is a **TCP** height: the Z the arm is commanded to
# in order to grip something lying on the table, measured by touching the tags.
# It is not where the wood is. The gripper's tongs hang below the TCP, and they
# are long enough that the surface they reach is the plane the arm's own base
# stands on -- so the MT4 sits on the desk, which is the only arrangement in
# which all three of the firmware's numbers make sense at once:
#
#   CENCER_HEIGHT = 140    the shoulder pivot, 140 mm above the desk it stands on
#   table_z       = 122    the TCP height that grips something lying on the desk
#   GROUND_Z_MM   = 115    the TCP height at which the tong tips touch the wood,
#                          i.e. the floor -- which is what a floor should be
#
# The alternative -- wood at z = 122 with the arm's base buried beneath it --
# is what this modelled before, and it put the gripper body exactly on a 20 mm
# cube's top face on every pick.
TCP_GRIP_Z_MM = calibration.table_z_mm()
DESK_Z_MM = calibration.desk_surface_z_mm()

# The gripper's own height: J4 (the wrist roll axis, whose origin *is* the TCP)
# down to the tips of the tongs. Measured on the real gripper -- the CAD is no
# help here, because the STEP assembly carries the soft gripper as named parts
# with no B-rep attached.
#
# This is not free either, and the firmware pins it: `GROUND_Z_MM` = 115 is the
# lowest TCP the host will command, and a floor is the height at which the thing
# hanging lowest touches the wood. Tongs 115 mm long put the tips exactly on the
# desk at TCP = 115, so the firmware's floor and the gripper's height are the
# same number for the same reason.
TONG_REACH_MM = 115.0
# What that leaves at grip height: the tips ride this far above the wood while
# gripping, straddling the lower 13 mm of a 20 mm cube. It used to be 0 -- the
# tongs were sized as `TCP_GRIP_Z_MM - DESK_Z_MM` = 122, on the assumption that
# they reached exactly to the wood -- and tips resting on the desk are what
# `GROUND_Z_MM` then had to be read as driving 7 mm *under* it.
TONG_CLEARANCE_MM = TCP_GRIP_Z_MM - DESK_Z_MM - TONG_REACH_MM

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
# The jaw drive: a torque-limited servo, not a spring
# --------------------------------------------------------------------------
#
# The real gripper is one servo driving a symmetric scissor. It runs a position
# loop; when the blades meet the object the loop cannot null its error, the
# motor current saturates, and from then on the jaws are a **constant-force
# actuator at the servo's torque limit**. That force is what the object feels,
# and it does not depend on how wide the object is.
#
# Modelling that is not the same as writing ``maxForce`` on the drive, and the
# difference is the whole reason this section exists. A PhysX drive clamps
# *per joint*: ``F_i = clip(k*(t - x_i) - c*v_i, +/-Fmax)``. The pair has two
# coordinates,
#
#     gap = left + right       what the servo actually drives
#     mid = (left - right)/2   what the scissor welds to the wrist
#
# and a per-joint clamp destroys the second one. Both blades closing sit on the
# same cap in opposite directions, so the midpoint restoring term -2*k*mid is
# not weak but **identically zero** -- measured at 0.001 N against 1.5 N on each
# blade. The pair becomes a free 0.6 kg mass with no spring and no damper, keeps
# whatever sideways velocity a move onset hands it, and walks 12.8 mm out from
# under the wrist carrying the payload with it. ``check_jaw_midpoint_fixed`` is
# the assertion; docs/gripper-fires-open.md is the investigation.
#
# **So the limit is applied to the gap and not to each blade.** ``SimArm``
# clamps how far the servo's position loop is allowed to wind up, measuring the
# error against the blades' *mean* opening rather than against each blade:
#
#     half_gap = (left + right) / 2
#     error    = commanded - half_gap
#     target   = half_gap + clip(error, +/-FINGER_WINDUP_M)      (both blades)
#
# Handing both blades that one target splits cleanly into the two modes:
#
#     F_left + F_right = 2k*clip(error)   the servo, limited to the torque limit
#     F_left - F_right = -2k*mid          the scissor, at full stiffness, never
#                                         clipped
#
# which is the actual machine: a force-limited actuator on the coordinate it
# drives, and structure on the coordinate it does not. The drive's own
# ``maxForce`` stays slack -- it is a numerical backstop, and a drive that
# reaches it is a drive whose midpoint term has gone to zero.
#
# This is not "track contact and hold a position short of it", which was tried
# and drops a misaligned cube: a latched target never follows a cube that
# rotates flat and pushes the blades apart. Nothing is latched here. The target
# is recomputed from the measured opening every step, so the jaws keep pressing
# at the torque limit wherever the cube goes -- it is a force source with a
# leash, not a position.
#
# Three real constraints were measured against this articulation first, because
# deleting ``mid`` outright would be better than managing it, and none of them
# reaches a reduced-coordinate articulation in this runtime:
#
#   * ``PhysxMimicJointAPI`` -- applied and ignored.
#   * ``PhysxPhysicsRackAndPinionJoint`` -- exact on free rigid bodies (0.000 mm
#     mirror error on an undriven rack) and inert on articulation DOFs. It looks
#     like it works at a high ratio, because a gear reflects its pinion's inertia
#     to the rack as I*ratio^2 and a 4 mm pinion at ratio 1e4 lands 2.7 kg on a
#     0.30 kg armature -- the jaws are not held together, they are too heavy to
#     move apart. Hold the ratio and drop the inertia to 1e-10 and the coupling
#     vanishes completely: a constraint does not care what the pinion weighs.
#   * the fixed tendon -- a force the step integrates, not a constraint, and
#     there is no band where it helps. See FINGER_COUPLING_STIFFNESS.
#
# ``physxJoint:jointFriction`` was measured too, since a geared servo is
# non-backdrivable and friction would hold ``mid`` for free. It is honoured, but
# only just: 3.0 N of it takes the open overshoot from 7.11 mm to 6.13 mm and
# does not slow the close at all, when 3 N of Coulomb friction should stop a
# blade the drive is pushing with 1 N outright. It is not a force this runtime
# delivers at the value asked for, so nothing is built on it.
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
# velocity a limited drive can inject into a finger in one substep is
# ``F * dt / m``, and at the 12 N cap two designs ago that was 2.5 m/s.
# Delivered through a finger into an 8 g cube, that impulse threw cubes across
# the desk.
FINGER_GRIP_FORCE_N = 1.5

# The position loop's gain. With the wind-up clamp above it no longer sets the
# grip force, so it is free to be what a servo's loop is: stiff. What it buys is
# the *other* mode -- the midpoint is held with -2*k*mid, so k is the scissor's
# stiffness as much as the servo's -- and how quickly the torque limit takes
# over once the blades touch something.
#
# It used to be pinned. The grip force was ``k`` times the opening the object
# left (10 mm for a 20 mm cube), so 1.5 N forced k = 150 N/m and the jaws were a
# soft spring pretending to be a servo. Two things were wrong with that and both
# are gone:
#
#   * the grip force depended on the object. 1.5 N on a 20 mm cube, but 0.75 N
#     on a 10 mm one and 2.2 N on a 30 mm one, for a machine whose whole
#     behaviour is "stall at the torque limit".
#   * k = 150 with the damping the tracking budget allowed is zeta = 0.29, and
#     that is the ringing this replaces.
#
# **What bounds it now is a speed ceiling, and it is a sharp one.** The loop's
# reference may never lead the blades by more than ``FINGER_WINDUP_M``, so the
# blades can never advance more than one wind-up length per physics step: the
# fastest they can sweep is ``FINGER_WINDUP_M / dt``, which is
# ``FINGER_GRIP_FORCE_N / (k * dt)``. Too stiff and the jaws simply cannot keep
# up with the firmware's own 0.096 m/s sweep, and the lag is not a transient --
# it accumulates for the length of the ramp. Measured on an open to ``g 140``,
# with everything else held:
#
#     k      wind-up   ceiling     ramp lag   overshoot   settle
#     1000   1.50 mm   0.090 m/s    5.64 mm    1.07 mm    0.067 s   under the sweep
#      600   2.50 mm   0.150 m/s    1.28 mm    1.24 mm    0.067 s
#      400   3.75 mm   0.225 m/s    1.40 mm    1.23 mm    0.100 s
#      300   5.00 mm   0.300 m/s    1.47 mm    1.14 mm    0.117 s
#
# 600 is the stiffest that clears the sweep with margin -- 1.57x -- and every
# softer setting tracks and settles slightly worse while holding the midpoint
# with a slacker spring. The ceiling is why this is not simply set as high as
# the solver tolerates.
FINGER_STIFFNESS_N_PER_M = 600.0

# How far the position loop winds up before the torque limit bites: the servo's
# error at stall. Anything the blades meet that is more than 2*this narrower
# than the commanded opening is gripped at exactly FINGER_GRIP_FORCE_N -- at
# 2.5 mm that is everything in this scene, a 20 mm cube included with 4x to
# spare.
#
# It is also the budget the midpoint has to stay inside. The two blades press at
# ``k*(windup -/+ mid)``, so a midpoint error of one wind-up length unloads one
# blade completely and the grip becomes one-sided.
FINGER_WINDUP_M = FINGER_GRIP_FORCE_N / FINGER_STIFFNESS_N_PER_M

# The drive's own per-joint force ceiling. A **backstop, not the grip**: a drive
# that reaches it is a drive whose midpoint term has gone to zero, which is the
# failure the wind-up clamp exists to avoid, so it has to sit above everything
# the servo legitimately asks for. The spring half is bounded by construction at
# ``k * FINGER_WINDUP_M`` = FINGER_GRIP_FORCE_N; the damper half is not, because
# braking is dissipation rather than motor torque, and a blade meeting a cube at
# sweep speed can ask for ``c * v`` on the way to a stop.
#
# Measured over a replayed place-down, the peak per-joint effort is 5.18 N and
# only 9 steps of 3151 pass 2 N -- all of them the step a blade meets the cube.
# 20.0 leaves ~4x on the worst of those.
FINGER_EFFORT_N = 20.0

# The velocity loop's gain, and the fix for the ringing.
#
# The jaws used to overshoot an open command by 7.11 mm of gap and ring for
# 0.43 s through seven reversals, because zeta was 0.29 -- and chain.py said
# outright that zeta > 1 was unreachable, because a position drive following a
# ramp lags by ``c*rate/k`` and the firmware sweeps each jaw at 0.096 m/s. More
# damping bought less ring and more lag, and there was no setting that was good
# at both.
#
# **That trade is an artifact of driving a servo with position alone.** A servo
# controller feeds the commanded rate forward; only the *error* is left for the
# loop to work on. PhysX drives take a velocity target and damp ``v - v_target``
# rather than ``v``, so feeding the sweep rate forward costs nothing and removes
# the drag term the lag was paying for. ``SimArm.set_gripper_s`` sends it.
#
# Measured on an open to ``g 140`` -- the S the host actually sends, not the
# open stop, where a blade against its travel limit cannot overshoot at all.
# First the old drive, to show the trade it was stuck in, at k = 150:
#
#     drive c   feed-forward   ramp lag   overshoot   settle   reversals
#      4.0        no            9.03 mm    7.11 mm    0.43 s      7    (was)
#     13.9        no           15.73       0.00       0.18 s      0
#      4.0        yes           6.37       8.26       0.53 s      7
#     13.9        yes           3.51       3.25       0.18 s      1
#
# Rows 1 and 2 are the trade: damping enough to stop the ring costs 15.7 mm of
# lag. Row 4 is the same damping with the rate fed forward -- 4.5x less lag than
# row 2 and no ring worth the name. Then the servo as it ships, k = 600:
#
#     drive c    zeta    ramp lag   overshoot   settle   reversals
#      15.0      0.54     2.41 mm    2.33 mm    0.050 s     3
#      25.0      0.90     1.80       1.80       0.067       0
#     *40.0*     1.44     1.28       1.24       0.067       0
#      60.0      2.17     0.92       0.82       0.050       0
#      80.0      2.89     0.72       0.57       0.033       0
#
# Everything improves monotonically with c, which is the tell that the tracking
# cost is gone: it used to be the term that turned this table around. What does
# not appear here is the cost that picked 40.0 over 80.0 -- the same damper
# meets the cube, and ``scripts/check_grip.py`` is where that shows up.
FINGER_DAMPING_N_S_PER_M = 40.0

# Armature: extra inertia in *joint* space, and the thing that makes the above
# usable at 60 Hz.
#
# It was introduced against a drive that clipped at ``maxForce``, which has no
# damping left -- the ``c * v`` term is clipped away with the rest -- so in
# sustained contact it stopped being a spring and became a constant-force
# actuator with nothing opposing velocity, bouncing off the contact at
# ``F * dt / m`` per step: 1.25 m/s for a 20 g finger at 1.5 N and 60 Hz.
# Measured, the jaws chattered over ~0.8 mm and the cube's angular velocity
# swung +/-280 deg/s about zero, so the couple that should have squared the cube
# up averaged out to nothing.
#
# That argument no longer applies on its own -- the wind-up clamp limits the
# force without clipping the drive, so the damper is live at stall too -- but
# the second argument below is the load-bearing one and is unchanged.
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

# --------------------------------------------------------------------------
# The jaws are coupled to each other
# --------------------------------------------------------------------------
#
# The real gripper is a scissor mechanism: one servo drives both blades through
# a symmetric linkage, so the blades cannot move independently and their midpoint
# is pinned to J4. A cube the jaws close on is therefore *centred* on the TCP --
# the mechanism pushes it there -- rather than being walked across the gripper by
# whichever blade reaches it first.
#
# Expressed on the two prismatic axes, the whole constraint is
#
#     q_left - q_right = 0
#
# because both joints have their origin at the TCP and both count positive
# outward. That is what a PhysX **fixed tendon** says: axes with gearings +1 and
# -1 and a rest length of zero, pulled together at ``FINGER_COUPLING_STIFFNESS``.
#
# Two things about this are easy to get wrong, both measured:
#
# * ``PhysxMimicJointAPI`` is the obvious tool and is present in the schema, but
#   this runtime applies it and then ignores it. Told left = 0 and right = 24.5
#   the jaws go to exactly 0.00 / 24.50 with the mimic applied, the same as with
#   no constraint at all -- for the ``transY`` and the ``linear`` axis token
#   alike.
# * A fixed tendon spans the **subtree** of the joint it is rooted on. Rooted on
#   one jaw it cannot reach the other, because the two finger joints are
#   siblings: that arrangement drags the rooted jaw to the rest length and leaves
#   the other exactly where it was told. It has to be rooted on a common
#   ancestor -- ``j4_wrist_roll`` -- carrying gearing 0 so the wrist itself
#   contributes nothing to the tendon's length.
#
# **The tendon is a force the step integrates, not a constraint it solves.** It
# settles wherever it balances the jaw drives rather than holding q_left =
# q_right outright. Told left = 0 and right = 24.5 -- the whole travel, against
# drives capped at FINGER_EFFORT_N -- the residual separation reads how hard it
# actually pulls, and it falls with stiffness the way a spring should:
#
#     k=3e4   3.38 mm      k=3e5   0.47 mm      k=3e6   0.18 mm
#     k=1e5   1.12 mm      k=1e6   0.25 mm
#
# Solver iteration count changes none of it -- 8, 16, 32, 64 and 128 all settle
# at the same figure -- and halving the physics step halves the separation.
#
# **Damping makes the static residual worse at every stiffness**, because it
# drags on the motion that closes the gap and not only on the ringing:
#
#     k=3e5, c=100   0.24 mm      c=1500   1.32 mm
#     k=3e5, c=400   0.47 mm      c=5000   4.02 mm
#
# **3e4/400 is deliberately soft, and the softness is load-bearing.** Rigidity
# here is not free: it is measured in missed picks. Over repeated three-level
# `stack_cubes.py` runs, the tendon setting is the difference between a run that
# works and one that does not --
#
#     tendon damping    missed picks per run      stacks built
#     400 (this)        0, 0, 0, 0, 0, 0          6 of 6
#      80              1, 1, 0                    3 of 3
#      20              1, 2, 2                    3 of 3
#       0              2, 9, 9, 9                 3 of 4
#
# -- and stiffening does the same thing: at k=1e6 a run missed nine and walked a
# green cube from (91, 262) to (115, 151) across the desk.
#
# The difference is what the jaws close on. check_grip.py puts the cube at its
# true position, perfectly centred, where a rigid symmetric pair grips it
# beautifully. A real pick aims at a *detected* position, 5.4 mm off on average
# and 11 mm at worst, and a rigid pair meeting an off-centre cube squeezes it
# out sideways instead of capturing it. The give is the gripper accommodating
# that error -- one jaw arrives first and the pair yields rather than the cube
# being ejected. So the way to earn a stiffer tendon is better cube positions
# rather than firmer jaws, and passing check_grip.py is not sufficient evidence
# on its own; a stacking run is what shows this.
#
# **What the tendon must not be paired with is an underdamped drive.** The
# tendon is enormously stronger than the jaws: measured through a stacking run
# its spring reaches 164 N and its damper 58 N, against drives capped at
# FINGER_EFFORT_N = 1.5 N. That is fine while the drives can absorb what it
# hands them, and it is not fine when they ring -- see
# FINGER_DAMPING_N_S_PER_M, which was mis-derived and is what actually made the
# jaws cycle.
#
# Separately: the lift has to be a lift. Handing the drives a 40 mm step puts
# the TCP through 0.42 m/s and the cube's own inertia levers the jaws open --
# gap 19.7 -> 26.0 mm -- whatever the tendon is set to, which is why
# check_grip.py walks the TCP up at the firmware's own CART_SEGMENT_MM per tick.
#
# **And it is off, because no non-zero value of it can hold a payload.**
# Everything above is about what the tendon buys at the *close*. What it costs
# during the *carry* went unmeasured until cubes were watched being dropped in
# mid-air -- see docs/gripper-fires-open.md for the traces.
#
# The jaw pair has a free sideways-translation mode: both blades sliding the
# same way, which leaves the gap unchanged and so costs the drives nothing. A
# real carry walks it about 22 mm. The tendon is the only thing that opposes
# that mode, and it cannot oppose it usefully, because with a cube between the
# blades the closing direction is blocked -- so the only way it can reduce an
# asymmetry is to push the lagging jaw *outward*. That opens the gap, and the
# gap is the grip. The tendon buys symmetry with the payload.
#
# Both failures are that one sentence, at opposite ends of the travel. Commanded
# shut with the right jaw against its 0 mm stop, the gap opened to 48.6 mm and
# the gripper fired fully open. Commanded shut with the left jaw against its
# 24.535 mm stop, the gap opened 19.7 -> 25.6 mm and dropped a cube from 138 mm.
#
# The walk itself is harmless. Replaying the same carry with the tendon removed,
# the pair still swings 22 mm -- and the gap does not move:
#
#     tendon        asymmetry over the carry   gap            cube
#     k=1e3 c=400   -0.08 -> 23.63 mm          19.66 -> 25.62  dropped
#     k=0   c=0     -9.67 -> 12.30 mm          19.64 -> 19.62  carried
#
# A force balance says why there is no good value. The tendon's force over the
# asymmetry it has to tolerate must stay under what the drive can push back
# with: 1.5 N over a 23 mm walk is k <= 65 N/m, and the damper is worse -- 400
# N.s/m against a 0.15 m/s asymmetry rate is 60 N, forty times the cap. Measured
# against two independent recorded carries, a short hop and a cross-desk
# traverse:
#
#     k     c      clearing carry        transit carry
#     0     0      placed                placed, on the column at z=110
#     0     400    placed                DROPPED
#     65    400    placed                DROPPED
#     1e3   400    placed                DROPPED
#     3e4   400    DROPPED               --
#
# Only zero passes both. So the coupling is authored and left inert rather than
# deleted: the tendon is the right *model* -- the real gripper is a scissor and
# its blades genuinely cannot move independently -- and this runtime has no way
# to express it. PhysxMimicJointAPI is the constraint that would, and is applied
# and ignored here (see above). Until that changes, the jaws are two independent
# force-capped blades, and what centres a cube is aiming at it correctly.
FINGER_COUPLING_STIFFNESS = 0.0
FINGER_COUPLING_DAMPING = 0.0
# The tendon's instance name on each joint prim.
FINGER_COUPLING_NAME = "jaws"


__all__ = [
    "ARM_JOINT_NAMES",
    "CENCER_HEIGHT",
    "CENCER_OFFSET",
    "DESK_Z_MM",
    "TCP_GRIP_Z_MM",
    "TONG_CLEARANCE_MM",
    "TONG_REACH_MM",
    "FINGER_ARMATURE_KG",
    "FINGER_COUPLING_DAMPING",
    "FINGER_COUPLING_NAME",
    "FINGER_COUPLING_STIFFNESS",
    "FINGER_DAMPING_N_S_PER_M",
    "FINGER_DYNAMIC_FRICTION",
    "FINGER_EFFORT_N",
    "FINGER_GRIP_FORCE_N",
    "FINGER_WINDUP_M",
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
    "PHYSICS_HZ",
    "Q_LIMITS",
    "finger_positions_for_s",
    "model_from_urdf",
    "park_pose",
    "physics_dt",
    "s_for_span_mm",
    "span_mm_for_s",
    "urdf_from_model",
    "urdf_limits",
]
