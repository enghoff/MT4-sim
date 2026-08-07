"""Open the scene in the Isaac Sim GUI with the arm live.

By default the arm parks and holds. ``--demo`` runs a slow pick-and-place-shaped
tour of the desk so the articulation can be watched moving.

    python scripts/run_sim.py
    python scripts/run_sim.py --demo
    python scripts/run_sim.py --headless --seconds 5
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from mt4_sim.paths import SCENE_USD  # noqa: E402

parser = argparse.ArgumentParser()
parser.add_argument("--headless", action="store_true", help="no window")
parser.add_argument("--demo", action="store_true", help="tour the desk instead of parking")
parser.add_argument(
    "--seconds", type=float, default=0.0, help="stop after this long (0 = run until closed)"
)
args = parser.parse_args()

from isaacsim import SimulationApp  # noqa: E402

app = SimulationApp({"headless": args.headless})

from isaacsim.core.api import World  # noqa: E402
from isaacsim.core.utils.stage import open_stage  # noqa: E402

from mt4_sim import rig  # noqa: E402
from mt4_sim.arm import SimArm  # noqa: E402
from mt4_sim.scene import physics_dt  # noqa: E402
from mt4_sim.chain import TCP_GRIP_Z_MM, GRIPPER_S_CLOSED, GRIPPER_S_OPEN  # noqa: E402

STEP_HZ = 60.0
# Above the desk by the same clearance the real stack transits at.
TRANSIT_Z_MM = TCP_GRIP_Z_MM + 70.0


def demo_waypoints():
    """Hover each marker, then each cube, opening and closing the jaws."""
    for marker in rig.MARKERS:
        yield (marker.x_mm, marker.y_mm, TRANSIT_Z_MM, GRIPPER_S_OPEN)
        yield (marker.x_mm, marker.y_mm, TCP_GRIP_Z_MM + 25.0, GRIPPER_S_CLOSED)
        yield (marker.x_mm, marker.y_mm, TRANSIT_Z_MM, GRIPPER_S_CLOSED)
    for cube in rig.CUBES:
        yield (cube.x_mm, cube.y_mm, TRANSIT_Z_MM, GRIPPER_S_OPEN)


def main() -> int:
    if not SCENE_USD.is_file():
        print(f"missing {SCENE_USD} -- run scripts/build_scene.py first")
        return 1

    open_stage(str(SCENE_USD))
    world = World(stage_units_in_meters=1.0, physics_dt=physics_dt())
    world.reset()

    from isaacsim.core.utils.stage import get_current_stage

    from mt4_sim.scene import lock_scene_camera

    lock_scene_camera(get_current_stage())

    arm = SimArm()
    arm.park()
    print(f"parked at {arm.state().q}, TCP {arm.state().tcp_mm}")

    waypoints = list(demo_waypoints()) if args.demo else []
    hold_steps = int(1.2 * STEP_HZ)
    index, held = 0, 0
    elapsed = 0.0

    while app.is_running():
        if waypoints:
            if held == 0:
                x, y, z, grip = waypoints[index % len(waypoints)]
                q = arm.move_to_tcp(x, y, z)
                arm.set_gripper_s(grip)
                if q is None:
                    print(f"  ({x:.0f}, {y:.0f}, {z:.0f}) is out of reach -- skipped")
                else:
                    print(f"  -> ({x:6.1f},{y:7.1f},{z:6.1f})  q={q}")
                index += 1
            held = (held + 1) % hold_steps

        world.step(render=True)
        elapsed += 1.0 / STEP_HZ
        if args.seconds and elapsed >= args.seconds:
            break

    return 0


if __name__ == "__main__":
    code = main()
    app.close()
    sys.exit(code)
