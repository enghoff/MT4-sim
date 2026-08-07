"""Run the scene and answer the MT4's serial protocol on a socket.

This is the arm's firmware, replaced. Point the control repo's `Mt4Client` at
the address printed on startup and everything above it -- `mt4_vision`'s
pick/place primitives, the task scripts, the MCP tools -- drives the simulated
arm instead of the real one, with no change to any of them.

    python scripts/serve_firmware.py                     # headless, tcp 5570
    python scripts/serve_firmware.py --gui               # watch it move
    python scripts/serve_firmware.py --camera            # also publish the scene camera
    python scripts/serve_firmware.py --listen COM21      # a com0com pair

Then, from anywhere:

    python scripts/run_against_sim.py -- Z:\\MT4\\jog.py

Simulation time is paced to the wall clock, because the host is timing us: a
move that takes 4 seconds on the bench has to take 4 seconds here or every
timeout in the client means something different.
"""

from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))

from mt4_sim.paths import SCENE_USD  # noqa: E402


def parse_args() -> argparse.Namespace:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument(
        "--listen",
        default="tcp://127.0.0.1:5570",
        help="tcp://host:port, or a serial port name for a virtual null-modem pair",
    )
    ap.add_argument("--gui", action="store_true", help="open the Isaac Sim window")
    ap.add_argument(
        "--render",
        action="store_true",
        help="render every step even when headless",
    )
    ap.add_argument(
        "--camera",
        action="store_true",
        help="publish /World/SceneCamera frames on a shared-memory feed for "
        "run_against_sim.py (implies paced rendering at --camera-hz)",
    )
    ap.add_argument(
        "--camera-hz",
        type=float,
        default=15.0,
        help="how often to render and publish the scene camera (default 15)",
    )
    ap.add_argument(
        "--camera-name",
        default="mt4_scene_cam",
        help="shared-memory name for the camera feed",
    )
    ap.add_argument(
        "--home-seconds",
        type=float,
        default=3.0,
        help="how long `home` takes; the real seek is tens of seconds of hunting "
        "for limit switches the stage does not have",
    )
    ap.add_argument(
        "--free-run",
        action="store_true",
        help="do not pace to the wall clock (faster, but the host's timeouts "
        "and the firmware's step periods stop meaning the same thing)",
    )
    ap.add_argument("--quiet", action="store_true", help="do not echo the wire traffic")
    return ap.parse_args()


args = parse_args()
# SimulationApp forwards leftover sys.argv into Kit. Our flags (--camera,
# --listen, …) are not Kit's; leaving them there makes the app exit as soon as
# it finishes starting.
sys.argv = [sys.argv[0]]

from isaacsim import SimulationApp  # noqa: E402

app = SimulationApp({"headless": not args.gui, "renderer": "RaytracedLighting"})

from isaacsim.core.api import World  # noqa: E402
from isaacsim.core.utils.stage import open_stage  # noqa: E402

from mt4_sim import rig  # noqa: E402
from mt4_sim.arm import SimArm  # noqa: E402
from mt4_sim.scene import physics_dt  # noqa: E402
from mt4_sim.camera_feed import SceneCameraPublisher  # noqa: E402
from mt4_sim.firmware import Mt4Machine, open_link  # noqa: E402
from mt4_sim.chain import park_pose  # noqa: E402
from mt4_jog.kinematics import steps_from_angles  # noqa: E402


def main() -> int:
    if not SCENE_USD.is_file():
        print(f"missing {SCENE_USD} -- run scripts/build_scene.py first")
        return 1

    open_stage(str(SCENE_USD))
    world = World(stage_units_in_meters=1.0, physics_dt=physics_dt())

    from isaacsim.core.utils.stage import get_current_stage

    from mt4_sim.arm import prepare_gripper_contacts
    from mt4_sim.scene import lock_scene_camera

    prepare_gripper_contacts(get_current_stage())
    world.reset()
    prepare_gripper_contacts(get_current_stage())
    lock_scene_camera(get_current_stage())

    arm = SimArm()
    arm.park()
    say = lambda text: print(text, flush=True)  # noqa: E731 - a server must not buffer
    # Let the articulation settle before attaching the camera sensor -- the
    # same order scripts/check.py uses -- so the first rendered frames are of
    # a parked arm rather than a teleporting one.
    for _ in range(30):
        world.step(render=False)

    machine = Mt4Machine(
        arm,
        home_seconds=args.home_seconds,
        initial_steps=steps_from_angles(park_pose()),
    )
    link = open_link(args.listen)

    feed: SceneCameraPublisher | None = None
    if args.camera:
        say("warming scene camera…")
        feed = SceneCameraPublisher(
            resolution=rig.CAM_RESOLUTION, shm_name=args.camera_name
        )
        feed.warm(world)
        say(f"  first frame ok ({feed.resolution[0]}x{feed.resolution[1]})")

    dt = float(world.get_physics_dt())
    camera_period = 1.0 / max(args.camera_hz, 0.1)
    camera_due = 0.0
    say(f"MT4 firmware substitute listening on {link.description}")
    say(
        f"  physics {1.0 / dt:.0f} Hz, "
        f"{'paced to' if not args.free_run else 'free of'} the wall clock"
    )
    say(
        f"  point a client at it:  MT4_SIM_URL={link.description} "
        f"python scripts/run_against_sim.py -- <script>"
    )
    if feed is not None:
        say(
            f"  scene camera on {feed.url} ({feed.resolution[0]}x{feed.resolution[1]} "
            f"@ {args.camera_hz:g} Hz)"
        )
        say(
            f"  point vision at it:  MT4_CAMERA_URL={feed.url} "
            f"python scripts/run_against_sim.py -- <script>"
        )

    started = time.monotonic()
    ticks = 0
    try:
        while app.is_running():
            if link.take_connect():
                # Opening the port on the real board with DTR held off does not
                # reset it, so state survives a client reconnecting. What it
                # does get is a status block, which is also what `probe_port`
                # sniffs for when a host is hunting for the arm.
                link.send_all(machine.handle_line("?"))

            for line in link.poll():
                if not args.quiet:
                    say(f"  <- {line}")
                replies = machine.handle_line(line)
                if not args.quiet:
                    for reply in replies:
                        say(f"  -> {reply}")
                link.send_all(replies)

            machine.tick(dt)
            async_lines = machine.drain()
            if async_lines:
                for line in async_lines:
                    if not args.quiet:
                        say(f"  ~> {line}")
                    # A completion is the moment to ask the other question the
                    # firmware cannot: the counters say the move is done, but
                    # did the physical arm get there? The gap is the drives'
                    # tracking error, and it is the sim's own contribution.
                    if line.startswith(("mp done", "m done", "home ok")):
                        want = machine.state.tcp()
                        got = arm.measured_tcp_mm()
                        say(
                            f"     counters ({want.x:7.1f},{want.y:7.1f},{want.z:7.1f})"
                            f"  arm ({got[0]:7.1f},{got[1]:7.1f},{got[2]:7.1f})"
                            f"  off {max(abs(got[0] - want.x), abs(got[1] - want.y), abs(got[2] - want.z)):.2f}mm"
                        )
                link.send_all(async_lines)

            camera_due += dt
            publish_camera = feed is not None and camera_due >= camera_period
            render = args.gui or args.render or publish_camera
            world.step(render=render)
            if publish_camera:
                # Annotator can still miss a frame under load; skip rather than die.
                if feed.try_publish() is not None:
                    camera_due = 0.0
                # else: try again next tick without resetting the period clock
                # past a small grace -- keep due so we retry soon.
                elif camera_due > camera_period * 3:
                    camera_due = 0.0

            ticks += 1
            if not args.free_run:
                behind = (started + ticks * dt) - time.monotonic()
                if behind > 0:
                    time.sleep(behind)
                elif behind < -1.0:
                    # Losing a second of ground means every duration the host
                    # measures is wrong; say so rather than drift silently.
                    print(f"  (running {-behind:.1f}s behind real time)")
                    started = time.monotonic() - ticks * dt
    except KeyboardInterrupt:
        print("\nstopping")
    finally:
        if feed is not None:
            feed.close()
        link.close()
    return 0


if __name__ == "__main__":
    code = main()
    app.close()
    sys.exit(code)
