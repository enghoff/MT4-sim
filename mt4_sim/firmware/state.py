"""What the MT4 firmware knows about itself, in the units it knows it in.

The firmware is **open loop**. Its idea of where the arm is is a set of step
counters it has been incrementing since the last home; there is no encoder to
disagree with them. Everything the host reads back -- ``pos``, the derived
``tcp`` line, soft-limit checks, IK seeds -- comes off those counters, so this
model keeps them as the authority too and lets the simulated arm follow them,
exactly as the real steppers follow their pulse train.

Two consequences worth naming, because they are what make this a *substitute*
for the firmware rather than a different controller wearing its protocol:

* Commands that rewrite counters without moving (``setpos``, ``j4zero``) are
  pure bookkeeping on real hardware -- the wrist does not twitch. Here the arm
  is real enough to twitch, so a per-joint ``bias`` records the difference
  between what the counters claim and where the arm was left, and the arm is
  driven to ``counter - bias``.
* Step counters are integers and angles are not. A pose commanded in degrees
  is quantised to the same 1/35 deg the real arm quantises it to.
"""

from __future__ import annotations

from dataclasses import dataclass, field

from mt4_sim import mt4_repo  # noqa: F401  (sys.path bootstrap)

from mt4_jog.joints import (
    GRIPPER_S_CLOSED,
    GRIPPER_S_OPEN,
    GRIPPER_SWEEP_RATE_S_PER_S as FIRMWARE_GRIPPER_SWEEP_RATE_S_PER_S,
    GROUND_Z_MM,
    J1_HOME_CENTER_STEPS,
    J2_HOME_PULLOFF_STEPS,
    J3_HOME_PULLOFF_STEPS,
    J2_J3_SUM_MAX_STEPS,
    J2_J3_SUM_MIN_STEPS,
    JOG_SPEED_MAX_US,
    JOG_SPEED_MIN_US,
    JOINT_SOFT_MAX_STEPS,
    JOINT_SOFT_MIN_STEPS,
    MQ_QUEUE_CAPACITY,
    MQ_STATION_DWELL_MAX_MS,
)
from mt4_jog.kinematics import JointAnglesDeg, fk_tcp, steps_from_angles, ws_j4_deg

# firmware dda.cpp: the step period the DDA boots with, before any `speed`.
DEFAULT_SPEED_US = 1524
# firmware config.h MT4_KEEPOUT_RADIUS_MM, and start_absolute_move's margin.
KEEPOUT_RADIUS_MM = 140.0
KEEPOUT_MARGIN_MM = 0.5
# firmware config.h: how finely `mp` chops a Cartesian line, and the cap.
CART_SEGMENT_MM = 2.0
MAX_SEGMENTS = 250

# Jaw motion vs grip-station hold. The real servo finishes a close faster than
# the firmware's 120 S/s bookkeeping, and the simulated fingers still lag the
# counter. Advance S (finger targets) at 2x the previous sim rate so the jaws
# are on the cube early, but keep ``settled`` paced at the previous rate so
# grip stations hold the arm for the same wall time -- extra settle before lift.
GRIPPER_HOLD_RATE_S_PER_S = FIRMWARE_GRIPPER_SWEEP_RATE_S_PER_S * 1.5  # 180
GRIPPER_SWEEP_RATE_S_PER_S = GRIPPER_HOLD_RATE_S_PER_S * 2.0  # 360 motion

NUM_JOINTS = 4

__all__ = [
    "CART_SEGMENT_MM",
    "DEFAULT_SPEED_US",
    "GRIPPER_HOLD_RATE_S_PER_S",
    "GRIPPER_S_CLOSED",
    "GRIPPER_S_OPEN",
    "GRIPPER_SWEEP_RATE_S_PER_S",
    "GROUND_Z_MM",
    "JOG_SPEED_MAX_US",
    "JOG_SPEED_MIN_US",
    "KEEPOUT_MARGIN_MM",
    "KEEPOUT_RADIUS_MM",
    "MAX_SEGMENTS",
    "MQ_QUEUE_CAPACITY",
    "MQ_STATION_DWELL_MAX_MS",
    "NUM_JOINTS",
    "FirmwareState",
    "Gripper",
]


@dataclass
class Gripper:
    """The servo, as the firmware drives it.

    ``s`` is what ``?`` reports and what the finger drives follow; it advances
    at ``GRIPPER_SWEEP_RATE_S_PER_S``. ``settled`` also waits out a hold paced
    at ``GRIPPER_HOLD_RATE_S_PER_S`` (half the motion rate), so a grip station
    keeps the arm parked for the same wall time while the jaws finish early.
    """

    s: float = float(GRIPPER_S_CLOSED)
    target_s: float = float(GRIPPER_S_CLOSED)
    sweep: str = "stop"  # "stop" | "open" | "close"
    hold_left_s: float = 0.0

    @property
    def settled(self) -> bool:
        return (
            self.sweep == "stop"
            and abs(self.s - self.target_s) < 0.5
            and self.hold_left_s <= 0.0
        )

    def _arm_hold(self, from_s: float, to_s: float) -> None:
        """Schedule the grip-station wait as if the jaws still moved at hold rate."""
        self.hold_left_s = abs(to_s - from_s) / GRIPPER_HOLD_RATE_S_PER_S

    def command(self, s: float) -> None:
        """`g <S>`: stop any sweep and take the new value as commanded."""
        self.sweep = "stop"
        target = float(min(GRIPPER_S_CLOSED, max(GRIPPER_S_OPEN, s)))
        self._arm_hold(self.s, target)
        self.target_s = target

    def start_sweep(self, direction: str) -> None:
        self.sweep = direction
        target = float(GRIPPER_S_OPEN if direction == "open" else GRIPPER_S_CLOSED)
        self._arm_hold(self.s, target)
        self.target_s = target

    def stop_sweep(self) -> None:
        self.sweep = "stop"
        self.target_s = self.s
        self.hold_left_s = 0.0

    def at_end(self, direction: str) -> bool:
        if direction == "open":
            return self.s <= GRIPPER_S_OPEN
        return self.s >= GRIPPER_S_CLOSED

    def tick(self, dt_s: float) -> None:
        """Advance S at the fast motion rate; count down the slower hold clock."""
        span = GRIPPER_SWEEP_RATE_S_PER_S * dt_s
        delta = self.target_s - self.s
        if abs(delta) <= span:
            self.s = self.target_s
            if self.sweep != "stop":
                self.sweep = "stop"
        else:
            self.s += span if delta > 0 else -span
        if self.hold_left_s > 0.0:
            self.hold_left_s = max(0.0, self.hold_left_s - dt_s)

@dataclass
class FirmwareState:
    """Counters, flags and limits -- everything ``?`` prints."""

    steps: list[float] = field(default_factory=lambda: [0.0, 0.0, 0.0, 0.0])
    # What the counters claim minus where the arm actually is, in steps. Only
    # `setpos` and `j4zero` ever move it: they renumber without commanding
    # motion, which on open-loop hardware is free and here is not.
    bias: list[int] = field(default_factory=lambda: [0, 0, 0, 0])
    homed: bool = False
    speed_us: int = DEFAULT_SPEED_US
    orient_hold: bool = True
    drivers_enabled: bool = True
    cart_jog: bool = False
    ground_z_mm: float = GROUND_Z_MM
    soft_min: list[int] = field(default_factory=lambda: list(JOINT_SOFT_MIN_STEPS))
    soft_max: list[int] = field(default_factory=lambda: list(JOINT_SOFT_MAX_STEPS))
    gripper: Gripper = field(default_factory=Gripper)

    # -- counters ---------------------------------------------------------

    def counters(self) -> tuple[int, int, int, int]:
        """The four step counters as ``pos`` prints them."""
        return tuple(int(round(v)) for v in self.steps)  # type: ignore[return-value]

    def angles(self) -> JointAnglesDeg:
        """Model angles the counters correspond to."""
        return JointAnglesDeg.from_steps(self.counters())

    def commanded_angles(self) -> JointAnglesDeg:
        """Model angles to drive the arm to -- counters less the bias."""
        return JointAnglesDeg.from_steps(
            tuple(int(round(v)) - b for v, b in zip(self.steps, self.bias))
        )

    def tcp(self):
        return fk_tcp(self.angles())

    def world_j4_deg(self) -> float:
        return ws_j4_deg(self.angles())

    def set_counters(self, steps) -> None:
        """Renumber without moving: the bias absorbs the whole change."""
        for i, value in enumerate(steps):
            self.bias[i] += int(round(value)) - int(round(self.steps[i]))
            self.steps[i] = float(value)

    # -- limits -----------------------------------------------------------

    def apply_home_limits(self, j1_center: int) -> str:
        """Re-derive the soft limits the way ``motion_apply_home_soft_limits``
        does, and return the line it prints."""
        self.soft_min = list(JOINT_SOFT_MIN_STEPS)
        self.soft_max = list(JOINT_SOFT_MAX_STEPS)
        # J1 homes to its switch and centres from there; J2's counter is
        # limit-referenced, so zero is the switch and nothing goes below it.
        self.soft_min[0] = -int(j1_center)
        self.soft_min[1] = 0
        return (
            f"home limits J1={self.soft_min[0]}..{self.soft_max[0]}"
            f" J2={self.soft_min[1]}..{self.soft_max[1]}"
            f" J3={self.soft_min[2]}..{self.soft_max[2]}"
            f" J4={self.soft_min[3]}..{self.soft_max[3]}"
            f" J2+J3={J2_J3_SUM_MIN_STEPS}..{J2_J3_SUM_MAX_STEPS}"
            f" ground_z={self.ground_z_mm:.1f}"
        )

    def within_soft_limits(self, steps) -> bool:
        values = [int(round(v)) for v in steps]
        for i in range(NUM_JOINTS):
            if values[i] < self.soft_min[i] or values[i] > self.soft_max[i]:
                return False
        total = values[1] + values[2]
        return J2_J3_SUM_MIN_STEPS <= total <= J2_J3_SUM_MAX_STEPS

    def steps_for(self, q: JointAnglesDeg) -> tuple[int, int, int, int]:
        return steps_from_angles(q)

    # -- what homing leaves behind ----------------------------------------

    def finish_home(self, j1_center: int, j2_pull: int) -> str:
        """Counters as ``do_home`` leaves them: J1 centred at zero, J2/J3 at
        their pull-offs, J4 preserved so a previous ``j4zero`` still means
        something. The arm is physically put there, so the bias clears."""
        j4 = int(round(self.steps[3]))
        self.steps = [0.0, float(j2_pull), float(J3_HOME_PULLOFF_STEPS), float(j4)]
        self.bias = [0, 0, 0, self.bias[3]]
        self.homed = True
        return self.apply_home_limits(j1_center)


HOME_J1_CENTER_DEFAULT = J1_HOME_CENTER_STEPS
HOME_J2_PULL_DEFAULT = J2_HOME_PULLOFF_STEPS
