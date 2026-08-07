"""The MT4 jog firmware's command loop, running against a simulated arm.

Feed it the lines a host writes to the serial port and it answers the lines the
firmware would answer, in the firmware's own vocabulary -- ``ok mp``, ``mp done
pos J1=...``, ``err mp keepout`` -- while advancing the arm. ``mt4_jog.client``
cannot tell the difference, which is the point: the scripts in the control repo
drive this without knowing.

What it is faithful about:

* **The counters are the truth.** Like the real firmware it is open loop; every
  reply is derived from step counters it advances itself, and the simulated arm
  is driven to follow them.
* **Timing.** A coordinated move takes one step period per master-axis step, so
  a leg takes as long here as it does on the bench, and ``speed <us>`` changes
  it the same way. The gripper S (finger targets) advances at 360 S/s -- 2x
  the previous sim rate -- while grip-station ``settled`` still waits as if
  the jaws moved at 180 S/s, so the arm stays put long enough for the close.
* **Validation and its wording.** Homing, gripper range, speed range, keep-out,
  ground plane, soft joint limits and the ``mq`` queue's capacity, station-pose
  and dwell checks all reject with the exact strings the host greps for.
* **Queue semantics.** ``mq`` cold-starts when idle and queues when not, a
  drained queue emits one ``mp done``, ``mp`` mid-flight overrides and drops the
  queue, and a grip station holds everything until the jaws have finished.

What it is not:

* **Homing does not seek.** There are no limit switches on the stage, so ``home``
  drives to the pose homing ends at and takes a nominal few seconds rather than
  the real seek's tens of them.
* **No acceleration ramp.** The firmware ramps a move in and out over ~60 ticks
  either side; this runs the whole leg at the step period, which makes a short
  leg finish a few tens of milliseconds early.
* **No pin-level lab commands.** ``d<pin>``/``x<pin>`` are accepted and tracked
  so the jog console's joint mode works, but nothing is wired to them.
* **Floating the drivers does not make the arm fall.** ``e0``/``all f`` set the
  flag ``?`` reports and stop nothing else; the real arm goes limp and loses its
  counters, which is why it has to be re-homed afterwards. Worth knowing because
  ``jog.py`` sends ``all f`` on startup as a matter of course.
"""

from __future__ import annotations

import math
from collections import deque
from dataclasses import dataclass

from mt4_sim.firmware.planner import (
    J4_EXPLICIT,
    J4_HOLD,
    J4_WRIST,
    Leg,
    plan_leg,
    validate_request,
)
from mt4_sim.firmware.state import (
    DEFAULT_SPEED_US,
    GRIPPER_S_CLOSED,
    GRIPPER_S_OPEN,
    HOME_J1_CENTER_DEFAULT,
    HOME_J2_PULL_DEFAULT,
    JOG_SPEED_MAX_US,
    JOG_SPEED_MIN_US,
    KEEPOUT_RADIUS_MM,
    MQ_QUEUE_CAPACITY,
    MQ_STATION_DWELL_MAX_MS,
    NUM_JOINTS,
    FirmwareState,
)
from mt4_jog.joints import JOINTS, J2_J3_SUM_MAX_STEPS, J2_J3_SUM_MIN_STEPS
from mt4_jog.kinematics import (
    DIR_POS_HIGH,
    JointAnglesDeg,
    Vec3,
    cartesian_step_rates,
    fk_tcp,
)

BOOT_BANNER = "MT4 jog firmware ready (joint + cartesian)"
MOVE_MAX_STEPS = 100_000
# The firmware's own tolerance on a grip station's declared pose.
STATION_POSE_TOLERANCE_MM = 1.0
# How closely the simulated joints must have caught up before a move is called
# done, and how long to wait for it. A stepper has no such wait; a position
# drive does, and 0.02 deg is under a tenth of a millimetre at the TCP.
SETTLE_TOLERANCE_DEG = 0.02
SETTLE_TIMEOUT_S = 0.5
# How long `home` takes here. The real seek is tens of seconds of hunting for
# limit switches that the stage does not have; this is honest about being a
# stand-in rather than pretending to a duration it isn't simulating.
HOME_SECONDS = 3.0

_DRIVE_TO_JOINT = {joint.drive: i for i, joint in enumerate(JOINTS)}
_DIR_TO_JOINT = {joint.direction: i for i, joint in enumerate(JOINTS)}


class ArmDriver:
    """What the machine needs from the thing it is driving.

    :class:`mt4_sim.arm.SimArm` satisfies this; the tests use a recorder.
    """

    def set_model_angles(self, q: JointAnglesDeg, *, teleport: bool = False) -> None:
        raise NotImplementedError

    def set_gripper_s(self, s: float, *, teleport: bool = False) -> None:
        raise NotImplementedError


@dataclass
class _Queued:
    """One pending ``mq`` entry: a leg to plan later, or a grip station."""

    x: float
    y: float
    z: float
    j4_field: float
    j4_mode: str
    grip: int
    speed_us: int
    dwell_ms: int

    @property
    def is_station(self) -> bool:
        return self.dwell_ms > 0


class Mt4Machine:
    """The firmware's serial personality, steppable in simulation time."""

    def __init__(
        self,
        arm: ArmDriver | None = None,
        *,
        home_seconds: float = HOME_SECONDS,
        initial_steps: tuple[int, int, int, int] | None = None,
    ) -> None:
        self.state = FirmwareState()
        # A real board powers up with zeroed counters and the arm wherever it
        # was left, which is why nothing absolute is allowed before `home`.
        # The stage opens with the arm already parked, so a host that reads
        # `pos` before homing should be told where it actually is rather than
        # be handed the counters' fiction.
        if initial_steps is not None:
            self.state.steps = [float(v) for v in initial_steps]
        self._arm = arm
        self._home_seconds = home_seconds

        self._segments: deque[tuple[int, int, int, int]] = deque()
        self._queue: deque[_Queued] = deque()
        self._moving = False
        self._move_kind = "mp"  # which "<kind> done" the running move will emit
        self._station: _Queued | None = None
        self._station_stage = ""  # "" | "sweep" | "settle"
        self._station_settle_s = 0.0
        self._settling = 0.0  # seconds left to let the drives catch up
        self._homing_left = 0.0
        self._home_args: tuple[int, int] = (HOME_J1_CENTER_DEFAULT, HOME_J2_PULL_DEFAULT)

        # Legacy joint-jog bookkeeping (the `x<pin>`/`d<pin>`/`j` console).
        self._jog_axes: list[int] = []
        self._jog_dir_high = [DIR_POS_HIGH[i] for i in range(NUM_JOINTS)]
        self._jog_active = False
        self._cart_dir: tuple[float, float, float] | None = None
        self._cart_j4_roll = 0

        self._pending: list[str] = []
        self._sync_arm(teleport=True)

    # -- output -----------------------------------------------------------

    def _emit(self, *lines: str) -> None:
        self._pending.extend(lines)

    def drain(self) -> list[str]:
        """Async lines produced since the last call (move completions, homing)."""
        out, self._pending = self._pending, []
        return out

    def boot_lines(self) -> list[str]:
        """What the firmware prints on reset: the banner, then a full status."""
        return [BOOT_BANNER, *self._status_lines()]

    # -- driving the simulated arm ----------------------------------------

    def _sync_arm(self, *, teleport: bool = False) -> None:
        if self._arm is None:
            return
        self._arm.set_model_angles(self.state.commanded_angles(), teleport=teleport)
        self._arm.set_gripper_s(self.state.gripper.s, teleport=teleport)

    # -- status ------------------------------------------------------------

    def _pos_lines(self) -> list[str]:
        j1, j2, j3, j4 = self.state.counters()
        tcp = self.state.tcp()
        return [
            f"pos J1={j1} J2={j2} J3={j3} J4={j4}",
            f"tcp x={tcp.x:.1f} y={tcp.y:.1f} z={tcp.z:.1f} "
            f"j4={self.state.world_j4_deg():.1f} grip={int(round(self.state.gripper.s))} "
            f"speed={self.state.speed_us}",
        ]

    def _limits_line(self) -> str:
        # No limit switches on the stage. Both read released, which is what they
        # read on the bench at any pose homing leaves the arm in.
        return "I20=1 I21=1"

    def _status_lines(self) -> list[str]:
        s = self.state
        step = "none"
        if self._jog_axes:
            step = "+".join(f"D{JOINTS[i].drive}" for i in self._jog_axes)
        jog_suffix = " T1" if self._jog_active and self._jog_axes else ""
        hold = ""
        if self._station_stage:
            hold = f" HOLD={self._station_stage}"
        return [
            "--- MT4 jog ---",
            f"MODE={'cart' if s.cart_jog else 'joint'}"
            f"  ORIENT={'hold' if s.orient_hold else 'free'}"
            f"  HOMED={'yes' if s.homed else 'no'}"
            f"  SPEED={s.speed_us}",
            f"GROUND_Z={s.ground_z_mm:.1f}"
            f"  SOFT J1={s.soft_min[0]}..{s.soft_max[0]}"
            f" J2={s.soft_min[1]}..{s.soft_max[1]}"
            f" J3={s.soft_min[2]}..{s.soft_max[2]}"
            f" J4={s.soft_min[3]}..{s.soft_max[3]}",
            f"J2+J3={J2_J3_SUM_MIN_STEPS}..{J2_J3_SUM_MAX_STEPS}"
            "  (extension couples J2/J3)",
            *self._pos_lines(),
            f"STEP={step}",
            f"EN={'on' if s.drivers_enabled else 'off'}"
            f"  JOG={'on' if self._jog_active else 'off'}{jog_suffix}"
            f"  MQ={len(self._queue)}{hold}"
            f"  LIM {self._limits_line()}",
            f"  GRIP S={int(round(s.gripper.s))}"
            f" pwm={'on' if s.gripper.sweep != 'stop' else 'off'}"
            f" sweep={s.gripper.sweep}",
            "---------------",
        ]

    # -- motion ------------------------------------------------------------

    def _cancel_move(self) -> None:
        self._segments.clear()
        self._queue.clear()
        self._moving = False
        self._station = None
        self._station_stage = ""
        self._settling = 0.0

    def _arm_leg(self, leg: Leg) -> None:
        if leg.speed_us:
            self.state.speed_us = int(leg.speed_us)
        if leg.grip:
            self.state.gripper.command(leg.grip)
        self._segments = deque(leg.waypoints)
        self._moving = True

    def _start_station(self, entry: _Queued) -> str | None:
        """Arm a grip station, or return the error line it fails with."""
        tcp = self.state.tcp()
        if (
            abs(tcp.x - entry.x) > STATION_POSE_TOLERANCE_MM
            or abs(tcp.y - entry.y) > STATION_POSE_TOLERANCE_MM
            or abs(tcp.z - entry.z) > STATION_POSE_TOLERANCE_MM
        ):
            return (
                f"err mq station pose want {entry.x:.1f},{entry.y:.1f},{entry.z:.1f}"
                f" at {tcp.x:.1f},{tcp.y:.1f},{tcp.z:.1f}"
            )
        if entry.grip:
            self.state.gripper.command(entry.grip)
        self._station = entry
        self._station_stage = "sweep"
        self._station_settle_s = entry.dwell_ms / 1000.0
        return None

    def _pop_queue(self) -> None:
        """Start whatever is next, or begin settling before reporting complete."""
        while self._queue:
            entry = self._queue.popleft()
            if entry.is_station:
                error = self._start_station(entry)
                if error is not None:
                    self._cancel_move()
                    self._emit(error)
                return
            planned = plan_leg(
                self.state,
                entry.x,
                entry.y,
                entry.z,
                entry.j4_field,
                entry.j4_mode,
                entry.grip,
                entry.speed_us,
            )
            if isinstance(planned, str):
                # A queued leg that fails its route check aborts the whole
                # remaining queue, not just itself.
                self._cancel_move()
                self._emit(planned)
                return
            self._arm_leg(planned)
            return

        # The counters have arrived. On the bench that settles it -- a stepper
        # is wherever its last pulse put it -- but a position drive is still
        # a fraction of a degree behind, and a host that captures a camera
        # frame the instant it sees "mp done" would catch the arm moving. Give
        # the drives a moment to land so the word means the same thing here.
        self._settling = SETTLE_TIMEOUT_S
        self._advance_settle(0.0)

    def _advance_settle(self, dt: float) -> None:
        """Hold the finished pose until the arm is on it, then report done."""
        self._settling -= dt
        error = None
        if self._arm is not None:
            error = getattr(self._arm, "tracking_error_deg", lambda: None)()
        if error is not None and error > SETTLE_TOLERANCE_DEG and self._settling > 0:
            return
        self._settling = 0.0
        self._moving = False
        self._emit(
            f"{self._move_kind} done " + self._pos_lines()[0], self._pos_lines()[1]
        )

    def _advance_station(self, dt: float) -> None:
        if self._station_stage == "sweep":
            if not self.state.gripper.settled:
                return
            self._station_stage = "settle"
        self._station_settle_s -= dt
        if self._station_settle_s > 0:
            return
        self._station = None
        self._station_stage = ""
        self._pop_queue()

    def _advance_segments(self, dt: float) -> None:
        """Run the coordinated DDA for ``dt`` of simulation time."""
        budget = dt * 1e6 / self.state.speed_us  # master-axis steps available
        steps = self.state.steps
        while budget > 0 and self._segments:
            target = self._segments[0]
            remaining = max(abs(target[i] - steps[i]) for i in range(NUM_JOINTS))
            if remaining <= budget:
                for i in range(NUM_JOINTS):
                    steps[i] = float(target[i])
                self._segments.popleft()
                budget -= remaining
            else:
                fraction = budget / remaining
                for i in range(NUM_JOINTS):
                    steps[i] += (target[i] - steps[i]) * fraction
                budget = 0.0

    def _advance_jog(self, dt: float) -> None:
        """Legacy joint jog: equal step rate on every selected axis."""
        if not self._jog_active or not self._jog_axes:
            return
        pulses = dt * 1e6 / self.state.speed_us
        for axis in self._jog_axes:
            direction = 1 if self._jog_dir_high[axis] == DIR_POS_HIGH[axis] else -1
            candidate = list(self.state.steps)
            candidate[axis] += direction * pulses
            if self.state.within_soft_limits(candidate):
                self.state.steps[axis] = candidate[axis]

    def _advance_cart_jog(self, dt: float) -> None:
        """World-frame TCP jog, off the control repo's own resolved-rate solve.

        The device solves a damped-least-squares Jacobian for step rates and
        runs them as one Bresenham DDA whose fastest axis ticks at the step
        period; ``cartesian_step_rates`` is that solve, so this is the same
        motion. Where it differs: at a soft limit or the keep-out boundary the
        firmware clamps the offending velocity component and slides along the
        edge, and this simply stops.
        """
        if not self._jog_active or self._cart_dir is None:
            return
        pulses = dt * 1e6 / self.state.speed_us
        candidate = list(self.state.steps)

        if any(self._cart_dir):
            rates = cartesian_step_rates(
                self.state.angles(),
                Vec3(*self._cart_dir),
                hold_orientation=self.state.orient_hold,
            )
            if rates is None:
                return
            *joint_rates, master = rates
            for i, rate in enumerate(joint_rates):
                candidate[i] += pulses * rate / master
        if self._cart_j4_roll:
            candidate[3] += pulses * self._cart_j4_roll

        if not self.state.within_soft_limits(candidate):
            return
        tcp = fk_tcp(
            JointAnglesDeg.from_steps(tuple(int(round(v)) for v in candidate))
        )
        if math.hypot(tcp.x, tcp.y) < KEEPOUT_RADIUS_MM or tcp.z < self.state.ground_z_mm:
            return
        self.state.steps = candidate

    def tick(self, dt: float) -> None:
        """Advance ``dt`` seconds of firmware time and drive the arm."""
        if dt <= 0:
            return

        self.state.gripper.tick(dt)

        if self._homing_left > 0:
            self._homing_left -= dt
            if self._homing_left <= 0:
                j1_center, j2_pull = self._home_args
                self._emit(self.state.finish_home(j1_center, j2_pull), "home ok")
        elif self._settling > 0:
            self._advance_settle(dt)
        elif self._station is not None:
            self._advance_station(dt)
        elif self._segments:
            self._advance_segments(dt)
            if not self._segments:
                self._pop_queue()
        elif self._moving:
            self._pop_queue()
        elif self.state.cart_jog:
            self._advance_cart_jog(dt)
        else:
            self._advance_jog(dt)

        self._sync_arm()

    # -- command parsing ---------------------------------------------------

    _JOG_EXEMPT = ("cj ", "mp ", "mq ", "speed ", "home ")
    _JOG_EXEMPT_EXACT = ("!", "stop", "j", "jog", "home", "$H", "pos", "?", "d")

    def handle_line(self, raw: str) -> list[str]:
        """Handle one host line; return the lines answered synchronously."""
        line = raw.strip()
        if not line:
            return []

        # Anything outside the exemption list stops an active jog first, so a
        # tracker polling `pos`/`?` mid-jog does not silently kill its own jog.
        if (
            line not in self._JOG_EXEMPT_EXACT
            and not line.startswith(self._JOG_EXEMPT)
            and line[0] not in "gG"
        ):
            self._jog_active = False
            self._cart_dir = None

        handler = self._dispatch(line)
        return handler

    def _dispatch(self, line: str) -> list[str]:  # noqa: C901 - a parser is a parser
        lower = line.lower()

        if line in ("?", "d"):
            return self._status_lines()
        if line == "s":
            return [self._limits_line()]
        if line == "pos":
            return self._pos_lines()
        if line[0] in "gG":
            return self._cmd_gripper(line[1:].strip())
        if line in ("home", "$H"):
            return self._cmd_home(HOME_J1_CENTER_DEFAULT, HOME_J2_PULL_DEFAULT)
        if lower.startswith("home "):
            return self._cmd_home_args(line[5:])
        if lower.startswith("setpos "):
            return self._cmd_setpos(line[7:])
        if line == "j4zero":
            return self._cmd_j4zero()
        if lower.startswith("speed "):
            return self._cmd_speed(line[6:])
        if lower.startswith("orient "):
            return self._cmd_orient(line[7:].strip())
        if lower.startswith("cj "):
            return self._cmd_cj(line[3:])
        if lower.startswith("mp "):
            return self._cmd_move_absolute(line[3:], queued=False)
        if lower.startswith("mq "):
            return self._cmd_move_absolute(line[3:], queued=True)
        if lower.startswith("m "):
            return self._cmd_move_relative(line[2:])
        if line in ("j", "jog"):
            self._jog_active = True
            return ["ok jog"]
        if line in ("!", "stop"):
            self._jog_active = False
            self._cancel_move()
            return ["ok stop"]
        if line == "all f":
            self._cancel_move()
            self.state.drivers_enabled = False
            self._jog_axes = []
            self._jog_active = False
            return ["ok all float"]
        if line == "e0":
            self.state.drivers_enabled = False
            return ["ok enable off"]
        if line == "e1":
            self.state.drivers_enabled = True
            return ["ok enable on"]
        if line[0] in "xX":
            return self._cmd_step_pin(line[1:])
        if line[0] in "dD" and len(line) > 1 and line[1].isdigit():
            return self._cmd_dir_pin(line)
        return ["err unknown"]

    # -- individual commands ----------------------------------------------

    def _cmd_gripper(self, arg: str) -> list[str]:
        grip = self.state.gripper
        if not arg:
            return [
                f"grip S={int(round(grip.s))} lim={GRIPPER_S_OPEN}-{GRIPPER_S_CLOSED}"
            ]
        if arg in ("stop", "0"):
            grip.stop_sweep()
            return ["ok grip stop"]
        if arg in ("o", "open"):
            if grip.at_end("open"):
                grip.stop_sweep()
                return ["ok grip at open"]
            grip.start_sweep("open")
            return ["ok grip open"]
        if arg in ("c", "close"):
            if grip.at_end("close"):
                grip.stop_sweep()
                return ["ok grip at closed"]
            grip.start_sweep("close")
            return ["ok grip close"]
        try:
            value = int(float(arg))
        except ValueError:
            return [f"err grip stop|o|c|{GRIPPER_S_OPEN}-{GRIPPER_S_CLOSED}"]
        if not GRIPPER_S_OPEN <= value <= GRIPPER_S_CLOSED:
            return [f"err grip stop|o|c|{GRIPPER_S_OPEN}-{GRIPPER_S_CLOSED}"]
        grip.command(value)
        return ["ok grip"]

    def _cmd_home_args(self, arg: str) -> list[str]:
        parts = arg.split()
        try:
            j1 = int(parts[0])
            j2 = int(parts[1]) if len(parts) > 1 else HOME_J2_PULL_DEFAULT
        except (IndexError, ValueError):
            return ["err home <j1> <j2>"]
        if not 0 < j1 <= 20000 or not 0 < j2 <= 20000:
            return ["err home <j1> <j2>"]
        return self._cmd_home(j1, j2)

    def _cmd_home(self, j1_center: int, j2_pull: int) -> list[str]:
        self._cancel_move()
        self._home_args = (j1_center, j2_pull)
        self._homing_left = self._home_seconds
        # Homing is a physical seek the stage cannot reproduce, so the arm is
        # walked to the pose the seek ends at over the same wall clock the host
        # would spend waiting, and the counters are rewritten when it lands.
        self.state.homed = False
        park = JointAnglesDeg.from_steps((0, j2_pull, 500, int(round(self.state.steps[3]))))
        if self._arm is not None:
            self._arm.set_model_angles(park)
        return ["home start"]

    def _cmd_setpos(self, arg: str) -> list[str]:
        parts = arg.split()
        if len(parts) != NUM_JOINTS:
            return ["err setpos <j1> <j2> <j3> <j4>"]
        try:
            values = [int(p) for p in parts]
        except ValueError:
            return ["err setpos <j1> <j2> <j3> <j4>"]
        self.state.set_counters(values)
        return ["ok " + self._pos_lines()[0], self._pos_lines()[1]]

    def _cmd_j4zero(self) -> list[str]:
        self._cancel_move()
        q = self.state.angles()
        # world_j4 = joint_j4 + j1, so a joint J4 of -j1 reports zero without
        # anything moving -- on the bench because the counter is all there is,
        # here because the bias absorbs it.
        renumbered = JointAnglesDeg(q.j1, q.j2, q.j3, -q.j1)
        steps = list(self.state.counters())
        steps[3] = self.state.steps_for(renumbered)[3]
        self.state.set_counters(steps)
        return ["ok j4zero " + self._pos_lines()[0], self._pos_lines()[1]]

    def _cmd_speed(self, arg: str) -> list[str]:
        try:
            value = int(arg.strip())
        except ValueError:
            return ["err speed <us>"]
        self.state.speed_us = max(JOG_SPEED_MIN_US, min(JOG_SPEED_MAX_US, value))
        return [f"ok speed {self.state.speed_us}"]

    def _cmd_orient(self, arg: str) -> list[str]:
        if arg in ("on", "hold"):
            self.state.orient_hold = True
        elif arg in ("off", "free"):
            self.state.orient_hold = False
        else:
            return ["err orient on|off"]
        return [f"ok orient {'hold' if self.state.orient_hold else 'free'}"]

    def _cmd_cj(self, arg: str) -> list[str]:
        """Cartesian jog. Direction only -- the console drives it, not a script."""
        tokens = arg.split()
        axis_map = {"x": (1.0, 0.0, 0.0), "y": (0.0, 1.0, 0.0), "z": (0.0, 0.0, 1.0)}
        if len(tokens) == 1 and tokens[0].lstrip("+-").lower() in axis_map:
            token = tokens[0]
            sign = -1.0 if token.startswith("-") else 1.0
            base = axis_map[token.lstrip("+-").lower()]
            self._cart_dir = tuple(sign * v for v in base)  # type: ignore[assignment]
            self._cart_j4_roll = 0
        elif len(tokens) >= 3:
            try:
                values = [float(t) for t in tokens[:4]]
            except ValueError:
                return ["err cj +x|-x|+y|-y|+z|-z|dx dy dz [j4]"]
            self._cart_dir = (values[0], values[1], values[2])
            self._cart_j4_roll = int(values[3]) if len(values) > 3 else 0
            if not any(self._cart_dir) and not self._cart_j4_roll:
                return ["err cj +x|-x|+y|-y|+z|-z|dx dy dz [j4]"]
        else:
            return ["err cj +x|-x|+y|-y|+z|-z|dx dy dz [j4]"]
        self.state.cart_jog = True
        self._jog_active = True
        return ["ok cj"]

    @staticmethod
    def _parse_move_args(arg: str) -> tuple | None:
        """``<x> <y> <z> <j4|h|w> <g> [speed_us] [dwell_ms]``."""
        parts = arg.split()
        if len(parts) < 5 or len(parts) > 7:
            return None
        try:
            x, y, z = (float(p) for p in parts[:3])
        except ValueError:
            return None
        token = parts[3].lower()
        if token in ("h", "w"):
            j4_mode = J4_HOLD if token == "h" else J4_WRIST
            j4_field = 0.0
        else:
            j4_mode = J4_EXPLICIT
            try:
                j4_field = float(parts[3])
            except ValueError:
                return None
        try:
            grip = int(float(parts[4]))
            speed_us = int(float(parts[5])) if len(parts) > 5 else 0
            dwell_ms = int(float(parts[6])) if len(parts) > 6 else 0
        except ValueError:
            return None
        return x, y, z, j4_field, j4_mode, grip, speed_us, dwell_ms

    def _cmd_move_absolute(self, arg: str, *, queued: bool) -> list[str]:
        kind = "mq" if queued else "mp"
        parsed = self._parse_move_args(arg)
        if parsed is None:
            usage = f"err {kind} <x> <y> <z> <j4|h|w> <g> [speed_us]"
            if queued:
                usage += " [dwell_ms]"
            return [usage]
        x, y, z, j4_field, j4_mode, grip, speed_us, dwell_ms = parsed

        if dwell_ms and not queued:
            return ["err mp dwell (use mq)"]
        if dwell_ms < 0 or dwell_ms > MQ_STATION_DWELL_MAX_MS:
            return [f"err mq dwell 0-{MQ_STATION_DWELL_MAX_MS}"]

        error = validate_request(self.state, x, y, z, grip, speed_us)
        if error is not None:
            return [error]

        entry = _Queued(x, y, z, j4_field, j4_mode, grip, speed_us, dwell_ms)

        if queued and self._busy():
            if len(self._queue) >= MQ_QUEUE_CAPACITY:
                return [f"err mq full {MQ_QUEUE_CAPACITY}"]
            self._queue.append(entry)
            return [f"ok mq queued {len(self._queue)}"]

        if entry.is_station:
            # A station with nothing running has nothing to hold back, so it
            # runs on the spot -- the pose check still applies.
            station_error = self._start_station(entry)
            if station_error is not None:
                return [station_error]
            self._moving = True
            self._move_kind = "mp"
            return ["ok mq"]

        # `mp` mid-flight is an override: whatever was queued is dropped.
        self._queue.clear()
        planned = plan_leg(
            self.state, x, y, z, j4_field, j4_mode, grip, speed_us
        )
        if isinstance(planned, str):
            return [planned]
        self._arm_leg(planned)
        self._move_kind = "mp"
        return [f"ok {kind}"]

    def _busy(self) -> bool:
        return (
            self._moving
            or bool(self._segments)
            or self._station is not None
            or self._settling > 0
        )

    def _cmd_move_relative(self, arg: str) -> list[str]:
        parts = arg.split()
        if len(parts) < NUM_JOINTS:
            return ["err m <dj1> <dj2> <dj3> <dj4> [dg]"]
        try:
            deltas = [int(float(p)) for p in parts[:NUM_JOINTS]]
            dgrip = int(float(parts[4])) if len(parts) > 4 else 0
        except ValueError:
            return ["err m <dj1> <dj2> <dj3> <dj4> [dg]"]
        if any(abs(d) > MOVE_MAX_STEPS for d in deltas):
            return ["err m step delta too large"]
        span = GRIPPER_S_CLOSED - GRIPPER_S_OPEN
        if abs(dgrip) > span:
            return ["err m gripper delta too large"]

        target = tuple(
            int(round(self.state.steps[i])) + deltas[i] for i in range(NUM_JOINTS)
        )
        self._queue.clear()
        self._segments = deque([target])
        self._moving = True
        self._move_kind = "m"
        if dgrip:
            self.state.gripper.command(self.state.gripper.s + dgrip)
        return ["ok m"]

    def _cmd_step_pin(self, arg: str) -> list[str]:
        add = arg.startswith("+")
        remove = arg.startswith("-")
        token = arg[1:] if (add or remove) else arg
        if token.lower() == "c":
            self._jog_axes = []
            self.state.cart_jog = False
            return ["ok step clear"]
        try:
            pin = int(token.lstrip("dD"))
        except ValueError:
            return ["err x<pin>|x+<pin>|x-<pin>|xc"]
        axis = _DRIVE_TO_JOINT.get(pin)
        if axis is None:
            return ["err x<pin>|x+<pin>|x-<pin>|xc"]
        if remove:
            if axis not in self._jog_axes:
                return ["err step missing"]
            self._jog_axes.remove(axis)
        elif add:
            if axis not in self._jog_axes:
                self._jog_axes.append(axis)
        else:
            self._jog_axes = [axis]
        self.state.cart_jog = False
        return ["ok step"]

    def _cmd_dir_pin(self, line: str) -> list[str]:
        head, _, mode = line.partition(" ")
        mode = mode.strip()
        if mode not in ("f", "l", "h"):
            return ["err mode f|l|h"]
        try:
            pin = int(head.lstrip("dD"))
        except ValueError:
            return ["err d<pin> f|l|h"]
        axis = _DIR_TO_JOINT.get(pin)
        if axis is not None and mode in ("l", "h"):
            self._jog_dir_high[axis] = mode == "h"
        step_axis = _DRIVE_TO_JOINT.get(pin)
        if step_axis is not None and mode == "f" and step_axis in self._jog_axes:
            self._jog_axes.remove(step_axis)
        return ["ok pin"]


def machine_tcp(machine: Mt4Machine) -> tuple[float, float, float]:
    """Convenience for tests and the host loop: where the counters put the TCP."""
    tcp = fk_tcp(machine.state.angles())
    return (tcp.x, tcp.y, tcp.z)


def distance_mm(a: tuple[float, float, float], b: tuple[float, float, float]) -> float:
    return math.dist(a, b)
