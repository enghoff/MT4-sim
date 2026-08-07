"""The firmware substitute must be indistinguishable from the firmware.

Every assertion here is made with the control repo's own code: replies are
parsed by ``mt4_jog.status``, and the last test drives the machine with a real
``mt4_jog.client.Mt4Client`` over a real socket, so what is being checked is
not "does it emit the string I expected" but "does the client that talks to the
arm accept this as an arm".

No Isaac Sim, no GPU:

    python -m unittest discover -s tests
"""

from __future__ import annotations

import sys
import threading
import time
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from mt4_sim.firmware import Mt4Machine, TcpLineLink
from mt4_sim.firmware.state import GRIPPER_S_CLOSED, GRIPPER_S_OPEN

from mt4_jog.kinematics import JointAnglesDeg, fk_tcp
from mt4_jog.status import parse_status_lines, parse_tcp_line

TICK = 1.0 / 240.0


class RecordingArm:
    """Stands in for ``SimArm``: remembers what it was told to do."""

    def __init__(self) -> None:
        self.q = JointAnglesDeg(0.0, 0.0, 0.0, 0.0)
        self.gripper_s = float(GRIPPER_S_CLOSED)
        self.commands = 0

    def set_model_angles(self, q: JointAnglesDeg, *, teleport: bool = False) -> None:
        self.q = q
        self.commands += 1

    def set_gripper_s(self, s: float, *, teleport: bool = False) -> None:
        self.gripper_s = float(s)


def run_for(machine: Mt4Machine, seconds: float) -> list[str]:
    """Advance simulated time, collecting whatever the machine emits."""
    out: list[str] = []
    for _ in range(int(seconds / TICK)):
        machine.tick(TICK)
        out.extend(machine.drain())
    return out


def run_until(machine: Mt4Machine, prefix: str, limit_s: float = 40.0) -> list[str]:
    """Advance until a line starts with ``prefix``; returns everything emitted."""
    out: list[str] = []
    for _ in range(int(limit_s / TICK)):
        machine.tick(TICK)
        out.extend(machine.drain())
        if any(line.startswith(prefix) for line in out):
            return out
    raise AssertionError(f"never saw {prefix!r}; got {out}")


def homed_machine(arm: RecordingArm | None = None) -> Mt4Machine:
    machine = Mt4Machine(arm or RecordingArm(), home_seconds=0.05)
    assert machine.handle_line("home") == ["home start"]
    run_until(machine, "home ok", limit_s=2.0)
    return machine


class TestStatus(unittest.TestCase):
    def test_status_parses_with_the_repos_own_parser(self) -> None:
        machine = homed_machine()
        status = parse_status_lines(machine.handle_line("?"))
        self.assertTrue(status.homed)
        self.assertFalse(status.parse_failed)
        self.assertEqual(status.mode, "joint")
        self.assertEqual(status.orient, "hold")
        self.assertEqual(status.joints, {"j1": 0, "j2": 1000, "j3": 500, "j4": 0})
        self.assertIsNotNone(status.tcp)
        self.assertTrue(status.drivers_enabled)

    def test_home_leaves_the_arm_in_the_park_pose(self) -> None:
        machine = homed_machine()
        status = parse_status_lines(machine.handle_line("pos"))
        assert status.tcp is not None
        # firmware kinematics.h: homed park FK is (190.0, 0, 225.6).
        self.assertAlmostEqual(status.tcp.x, 190.0, delta=0.1)
        self.assertAlmostEqual(status.tcp.y, 0.0, delta=0.1)
        self.assertAlmostEqual(status.tcp.z, 225.6, delta=0.1)

    def test_unknown_command(self) -> None:
        self.assertEqual(Mt4Machine().handle_line("wat"), ["err unknown"])


class TestValidation(unittest.TestCase):
    def test_mp_before_home_is_refused(self) -> None:
        machine = Mt4Machine()
        self.assertEqual(machine.handle_line("mp 230 -60 122 h 0"), ["err not homed"])

    def test_keepout_ground_grip_and_speed(self) -> None:
        machine = homed_machine()
        self.assertEqual(machine.handle_line("mp 50 0 200 h 0"), ["err mp keepout"])
        self.assertEqual(
            machine.handle_line("mp 230 -60 10 h 0"), ["err mp ground z<115.0"]
        )
        self.assertEqual(
            machine.handle_line("mp 230 -60 200 h 999"),
            [f"err mp grip {GRIPPER_S_OPEN}-{GRIPPER_S_CLOSED}"],
        )
        self.assertEqual(
            machine.handle_line("mp 230 -60 200 h 0 100"), ["err mp speed 700-4000"]
        )

    def test_unreachable_target(self) -> None:
        machine = homed_machine()
        self.assertEqual(machine.handle_line("mp 900 0 200 h 0"), ["err mp unreachable"])

    def test_mp_rejects_a_dwell(self) -> None:
        machine = homed_machine()
        self.assertEqual(
            machine.handle_line("mp 230 -60 200 h 0 0 500"), ["err mp dwell (use mq)"]
        )


class TestMoves(unittest.TestCase):
    def test_mp_arrives_where_it_was_sent(self) -> None:
        arm = RecordingArm()
        machine = homed_machine(arm)
        self.assertEqual(machine.handle_line("mp 230 -60 150 h 0"), ["ok mp"])
        lines = run_until(machine, "mp done")

        tcp = next(t for t in (parse_tcp_line(line) for line in lines) if t)
        self.assertAlmostEqual(tcp.x, 230.0, delta=0.5)
        self.assertAlmostEqual(tcp.y, -60.0, delta=0.5)
        self.assertAlmostEqual(tcp.z, 150.0, delta=0.5)
        # The arm was driven there, not teleported: many intermediate commands.
        self.assertGreater(arm.commands, 100)
        reached = fk_tcp(arm.q)
        self.assertAlmostEqual(reached.x, 230.0, delta=0.5)

    def test_move_takes_one_step_period_per_step(self) -> None:
        machine = homed_machine()
        before = machine.state.counters()
        machine.handle_line("speed 1000")
        machine.handle_line("mp 260 0 160 h 0")
        elapsed = 0.0
        while True:
            machine.tick(TICK)
            elapsed += TICK
            if any(line.startswith("mp done") for line in machine.drain()):
                break
            self.assertLess(elapsed, 40.0, "move never finished")
        after = machine.state.counters()
        master = max(abs(a - b) for a, b in zip(after, before))
        # Every segment runs at the step period; the whole leg is at least the
        # master axis's own step count long, and the segmentation adds a little.
        self.assertGreaterEqual(elapsed, master * 1000e-6 * 0.95)
        self.assertLess(elapsed, master * 1000e-6 * 2.0)

    def test_speed_is_clamped_and_acked(self) -> None:
        machine = homed_machine()
        self.assertEqual(machine.handle_line("speed 99"), ["ok speed 700"])
        self.assertEqual(machine.handle_line("speed 99999"), ["ok speed 4000"])

    def test_relative_move(self) -> None:
        machine = homed_machine()
        before = machine.state.counters()
        self.assertEqual(machine.handle_line("m 100 0 0 0"), ["ok m"])
        lines = run_until(machine, "m done")
        self.assertTrue(any(line.startswith("m done pos J1=") for line in lines))
        self.assertEqual(machine.state.counters()[0], before[0] + 100)

    def test_stop_cancels_without_a_done(self) -> None:
        machine = homed_machine()
        machine.handle_line("mp 300 0 200 h 0")
        run_for(machine, 0.2)
        self.assertEqual(machine.handle_line("stop"), ["ok stop"])
        self.assertEqual(run_for(machine, 2.0), [])


class TestQueue(unittest.TestCase):
    def test_queue_cold_starts_then_queues_and_drains_once(self) -> None:
        machine = homed_machine()
        self.assertEqual(machine.handle_line("mq 240 0 200 h 0"), ["ok mq"])
        self.assertEqual(
            machine.handle_line("mq 240 60 200 h 0"), ["ok mq queued 1"]
        )
        self.assertEqual(
            machine.handle_line("mq 240 -60 200 h 0"), ["ok mq queued 2"]
        )
        lines = run_until(machine, "mp done")
        self.assertEqual(sum(1 for line in lines if line.startswith("mp done")), 1)
        tcp = [t for t in (parse_tcp_line(line) for line in lines) if t][-1]
        self.assertAlmostEqual(tcp.y, -60.0, delta=0.5)

    def test_queue_capacity(self) -> None:
        machine = homed_machine()
        machine.handle_line("mq 240 0 200 h 0")
        for _ in range(8):
            machine.handle_line("mq 240 20 200 h 0")
        self.assertEqual(machine.handle_line("mq 240 20 200 h 0"), ["err mq full 8"])

    def test_grip_station_holds_the_queue_then_releases_it(self) -> None:
        machine = homed_machine()
        machine.handle_line("mq 240 0 200 h 0")
        # A station's pose must be the one the leg ahead of it ends at.
        self.assertEqual(
            machine.handle_line("mq 240 0 200 h 285 0 400"), ["ok mq queued 1"]
        )
        machine.handle_line("mq 240 0 260 h 0")

        machine.handle_line("g 120")
        run_for(machine, 2.0)  # jaws open, then the leg runs
        lines = run_until(machine, "mp done")
        self.assertEqual(sum(1 for line in lines if line.startswith("mp done")), 1)
        self.assertAlmostEqual(machine.state.gripper.s, 285.0, delta=0.5)

    def test_station_pose_mismatch_is_loud(self) -> None:
        machine = homed_machine()
        machine.handle_line("mq 240 0 200 h 0")
        machine.handle_line("mq 300 0 200 h 285 0 400")
        lines = run_until(machine, "err mq station pose")
        self.assertTrue(any("want 300.0,0.0,200.0" in line for line in lines))

    def test_dwell_cap(self) -> None:
        machine = homed_machine()
        self.assertEqual(
            machine.handle_line("mq 240 0 200 h 285 0 99999"), ["err mq dwell 0-5000"]
        )


class TestGripper(unittest.TestCase):
    def test_sweep_takes_the_firmware_s_own_time(self) -> None:
        machine = homed_machine()
        machine.handle_line("g 120")
        run_for(machine, 3.0)
        self.assertEqual(machine.handle_line("g c"), ["ok grip close"])
        # Motion at 360 S/s: 165-unit span is 0.458s. Hold paced at 180 S/s.
        run_for(machine, 0.35)
        self.assertLess(machine.state.gripper.s, GRIPPER_S_CLOSED)
        self.assertFalse(machine.state.gripper.settled)
        run_for(machine, 0.20)
        self.assertAlmostEqual(machine.state.gripper.s, GRIPPER_S_CLOSED, delta=0.5)
        # S arrived early; station hold still running.
        self.assertFalse(machine.state.gripper.settled)
        run_for(machine, 0.45)
        self.assertTrue(machine.state.gripper.settled)
        self.assertEqual(machine.handle_line("g c"), ["ok grip at closed"])

    def test_absolute_grip_drives_the_arm(self) -> None:
        arm = RecordingArm()
        machine = homed_machine(arm)
        self.assertEqual(machine.handle_line("g 200"), ["ok grip"])
        run_for(machine, 0.8)
        self.assertAlmostEqual(arm.gripper_s, 200.0, delta=0.5)

    def test_gripper_does_not_stop_a_move(self) -> None:
        machine = homed_machine()
        machine.handle_line("mp 300 0 200 h 0")
        run_for(machine, 0.2)
        machine.handle_line("g o")
        run_until(machine, "mp done")


class TestCartesianJog(unittest.TestCase):
    """`cj` is the only motion mode jog.py has left, so it has to work."""

    def test_each_axis_moves_only_itself(self) -> None:
        machine = homed_machine()
        for command, axis in (("cj +x", "x"), ("cj -y", "y"), ("cj 0 0 -1", "z")):
            before = machine.state.tcp()
            self.assertEqual(machine.handle_line(command), ["ok cj"])
            run_for(machine, 0.5)
            after = machine.state.tcp()
            moved = {
                name: getattr(after, name) - getattr(before, name)
                for name in ("x", "y", "z")
            }
            self.assertGreater(abs(moved[axis]), 5.0, f"{command} did not move {axis}")
            for name, delta in moved.items():
                if name != axis:
                    self.assertLess(abs(delta), 1.0, f"{command} moved {name} too")
            machine.handle_line("stop")

    def test_orientation_is_held_through_a_jog(self) -> None:
        machine = homed_machine()
        machine.handle_line("cj -y")
        run_for(machine, 0.8)
        self.assertAlmostEqual(machine.state.world_j4_deg(), 0.0, delta=0.5)

    def test_roll_turns_the_wrist_without_moving_the_tcp(self) -> None:
        machine = homed_machine()
        before = machine.state.tcp()
        machine.handle_line("cj 0 0 0 1")
        run_for(machine, 0.5)
        self.assertGreater(machine.state.world_j4_deg(), 3.0)
        after = machine.state.tcp()
        self.assertAlmostEqual(after.x, before.x, delta=0.2)
        self.assertAlmostEqual(after.y, before.y, delta=0.2)

    def test_stop_ends_it(self) -> None:
        machine = homed_machine()
        machine.handle_line("cj +x")
        run_for(machine, 0.2)
        machine.handle_line("stop")
        before = machine.state.tcp()
        run_for(machine, 0.5)
        self.assertAlmostEqual(machine.state.tcp().x, before.x, delta=1e-6)

    def test_a_status_poll_does_not_kill_the_jog(self) -> None:
        machine = homed_machine()
        machine.handle_line("cj +x")
        run_for(machine, 0.2)
        machine.handle_line("?")
        machine.handle_line("pos")
        before = machine.state.tcp()
        run_for(machine, 0.3)
        self.assertGreater(machine.state.tcp().x - before.x, 5.0)


class TestBookkeepingCommands(unittest.TestCase):
    def test_j4zero_renumbers_without_moving(self) -> None:
        arm = RecordingArm()
        machine = homed_machine(arm)
        machine.handle_line("mp 200 120 200 h 0")
        run_until(machine, "mp done")
        before = arm.q

        lines = machine.handle_line("j4zero")
        self.assertTrue(lines[0].startswith("ok j4zero pos J1="))
        run_for(machine, 0.1)
        self.assertAlmostEqual(machine.state.world_j4_deg(), 0.0, delta=0.05)
        # The counters changed; the arm did not.
        self.assertAlmostEqual(arm.q.j4, before.j4, delta=1e-6)

    def test_setpos_renumbers_without_moving(self) -> None:
        arm = RecordingArm()
        machine = homed_machine(arm)
        before = arm.q
        machine.handle_line("setpos 10 1000 500 0")
        run_for(machine, 0.1)
        self.assertEqual(machine.state.counters(), (10, 1000, 500, 0))
        self.assertAlmostEqual(arm.q.j1, before.j1, delta=1e-6)

    def test_orient_toggle(self) -> None:
        machine = homed_machine()
        self.assertEqual(machine.handle_line("orient off"), ["ok orient free"])
        self.assertEqual(machine.handle_line("orient on"), ["ok orient hold"])
        self.assertEqual(machine.handle_line("orient sideways"), ["err orient on|off"])


class _Bench:
    """A machine on a socket, ticked by a background thread."""

    def __init__(self) -> None:
        self.arm = RecordingArm()
        self.machine = Mt4Machine(self.arm, home_seconds=0.2)
        self.link = TcpLineLink("127.0.0.1", 0)
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def _run(self) -> None:
        while not self._stop.is_set():
            if self.link.take_connect():
                self.link.send_all(self.machine.boot_lines())
            for line in self.link.poll():
                self.link.send_all(self.machine.handle_line(line))
            self.machine.tick(TICK * 4)
            self.link.send_all(self.machine.drain())
            time.sleep(TICK)

    def close(self) -> None:
        self._stop.set()
        self._thread.join(timeout=2.0)
        self.link.close()


class TestAgainstTheRealClient(unittest.TestCase):
    """The strongest statement available: drive it with the control repo's client."""

    def test_client_homes_moves_and_grips(self) -> None:
        try:
            import serial  # noqa: F401
        except ImportError:  # pragma: no cover
            self.skipTest("pyserial not installed")

        from mt4_jog.client import Mt4Client
        import mt4_jog.serial as jog_serial

        bench = _Bench()
        self.addCleanup(bench.close)

        url = f"socket://127.0.0.1:{bench.link.port}"
        original = jog_serial.open_serial

        def open_socket(port=None, baud=115200):
            import serial

            return serial.serial_for_url(url, baudrate=baud, timeout=0.5)

        jog_serial.open_serial = open_socket
        import mt4_jog.client as client_module

        client_module.open_serial = open_socket
        self.addCleanup(setattr, jog_serial, "open_serial", original)
        self.addCleanup(setattr, client_module, "open_serial", original)

        client = Mt4Client(port="socket")
        self.addCleanup(client.close)

        result = client.home()
        self.assertTrue(result["ok"], result)
        self.assertTrue(result["homed"], result)

        status = client.get_status()
        self.assertTrue(status.homed)
        assert status.tcp is not None
        self.assertAlmostEqual(status.tcp.x, 190.0, delta=0.5)

        result = client.move_to(230.0, -60.0, 150.0, j4="hold", speed_us=900)
        self.assertTrue(result["ok"], result)
        pose = client.get_tcp()
        self.assertAlmostEqual(pose.x, 230.0, delta=0.5)
        self.assertAlmostEqual(pose.y, -60.0, delta=0.5)
        self.assertAlmostEqual(pose.z, 150.0, delta=0.5)

        result = client.gripper(200)
        self.assertTrue(result["ok"], result)
        self.assertAlmostEqual(client.get_tcp().grip, 200.0, delta=1.0)

    def test_client_runs_a_queued_pick_and_place_path(self) -> None:
        try:
            import serial  # noqa: F401
        except ImportError:  # pragma: no cover
            self.skipTest("pyserial not installed")

        from mt4_jog.client import Mt4Client
        import mt4_jog.client as client_module
        import mt4_jog.serial as jog_serial

        bench = _Bench()
        self.addCleanup(bench.close)

        url = f"socket://127.0.0.1:{bench.link.port}"

        def open_socket(port=None, baud=115200):
            import serial

            return serial.serial_for_url(url, baudrate=baud, timeout=0.5)

        original = jog_serial.open_serial
        jog_serial.open_serial = open_socket
        client_module.open_serial = open_socket
        self.addCleanup(setattr, jog_serial, "open_serial", original)
        self.addCleanup(setattr, client_module, "open_serial", original)

        client = Mt4Client(port="socket")
        self.addCleanup(client.close)
        self.assertTrue(client.home()["ok"])

        # Hover, descend, grip, lift -- the shape mt4_vision.pickplace queues.
        result = client.move_path(
            [(230.0, -60.0, 190.0), (230.0, -60.0, 122.0), (230.0, -60.0, 122.0),
             (230.0, -60.0, 190.0)],
            j4="hold",
            grip=[0, 0, 240, 0],
            dwell_ms=[0, 0, 200, 0],
            speed_us=900,
        )
        self.assertTrue(result["ok"], result)
        pose = client.get_tcp()
        self.assertAlmostEqual(pose.z, 190.0, delta=0.5)
        self.assertAlmostEqual(pose.grip, 240.0, delta=1.0)


if __name__ == "__main__":
    unittest.main()
