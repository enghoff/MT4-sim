"""Does the control repo's pick-and-place actually move a simulated cube?

Everything else that verifies this layer checks it against a description of the
firmware -- the protocol tests parse replies with ``mt4_jog.status``, the client
tests drive it with ``Mt4Client``. This one checks it against the world: it runs
``mt4_vision.pickplace.pick`` and ``place``, unmodified, over a real socket, and
then looks at where the cube ended up on the stage.

If a cube that was at (232, -95) is found at the place target afterwards, then
the serial protocol, the motion planning, the grip timing, the jaw friction and
the arm's kinematics all did their jobs in the right order.

    python scripts/check_firmware.py
"""

from __future__ import annotations

import math
import sys
import threading
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))

from mt4_sim.paths import SCENE_USD  # noqa: E402

from isaacsim import SimulationApp  # noqa: E402

app = SimulationApp({"headless": True})

from isaacsim.core.api import World  # noqa: E402
from isaacsim.core.utils.prims import get_prim_at_path  # noqa: E402
from isaacsim.core.utils.stage import open_stage  # noqa: E402
from pxr import UsdGeom  # noqa: E402

from mt4_sim import rig  # noqa: E402
from mt4_sim.arm import SimArm  # noqa: E402
from mt4_sim.chain import park_pose  # noqa: E402
from mt4_sim.firmware import Mt4Machine, TcpLineLink  # noqa: E402
from mt4_jog.kinematics import steps_from_angles  # noqa: E402

# Where the picked cube should end up. Far enough from where it started that
# nothing but a real pick could put it there.
PLACE_OFFSET_MM = 70.0
# This asks "did a pick actually happen", not "how accurate is the placement".
# The cube lands about 12mm out, radially, and that is the gripper's doing, not
# the protocol's: the calibration closes to S=255, which the jaw-span model puts
# past zero opening, so the simulated fingers squeeze a 20mm cube that the real
# servo would simply stall against. Tightening this number is a gripper-fidelity
# job (a stall model on the finger drives), not a firmware one.
PLACE_TOLERANCE_MM = 25.0


def cube_xy_mm(index: int, color: str) -> tuple[float, float, float]:
    prim = get_prim_at_path(f"/World/Cubes/cube_{index}_{color}")
    if not prim.IsValid():
        raise RuntimeError(f"no cube prim for {color}")
    translation = UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(0.0).ExtractTranslation()
    return (translation[0] * 1000.0, translation[1] * 1000.0, translation[2] * 1000.0)


def main() -> int:
    if not SCENE_USD.is_file():
        print(f"missing {SCENE_USD} -- run scripts/build_scene.py first")
        return 1

    open_stage(str(SCENE_USD))
    world = World(stage_units_in_meters=1.0)
    world.reset()

    arm = SimArm()
    arm.park()
    machine = Mt4Machine(
        arm, home_seconds=1.0, initial_steps=steps_from_angles(park_pose())
    )
    link = TcpLineLink("127.0.0.1", 0)
    url = f"socket://127.0.0.1:{link.port}"

    index, cube = next(
        (i, c) for i, c in enumerate(rig.CUBES, start=1) if c.color == "red"
    )
    target = (cube.x_mm + PLACE_OFFSET_MM, cube.y_mm)

    result: dict[str, object] = {}

    def drive() -> None:
        """The client half, in its own thread -- as separate as a real host."""
        sys.path.insert(0, str(ROOT / "scripts"))
        from run_against_sim import patch_serial

        patch_serial(url)
        from mt4_jog.client import Mt4Client
        from mt4_vision.calib import load_calibration
        from mt4_vision import pickplace

        calib = load_calibration()
        client = Mt4Client(port=url)
        try:
            result["home"] = client.home()
            if not result["home"].get("ok"):
                return
            result["pick"] = pickplace.pick(
                client, calib, cube.x_mm, cube.y_mm, yaw_deg=cube.yaw_deg
            )
            if not result["pick"].get("ok"):
                return
            result["place"] = pickplace.place(client, calib, target[0], target[1])
        finally:
            result["finished"] = True
            client.close()

    thread = threading.Thread(target=drive, daemon=True)

    dt = float(world.get_physics_dt())
    print(f"driving the sim through mt4_vision.pickplace over {url}")
    before = cube_xy_mm(index, cube.color)

    started = time.monotonic()
    ticks = 0
    thread.start()
    while not result.get("finished"):
        if time.monotonic() - started > 300.0:
            print("timed out waiting for the client")
            return 1
        if link.take_connect():
            link.send_all(machine.handle_line("?"))
        for line in link.poll():
            link.send_all(machine.handle_line(line))
        machine.tick(dt)
        link.send_all(machine.drain())
        world.step(render=False)
        ticks += 1
        behind = (started + ticks * dt) - time.monotonic()
        if behind > 0:
            time.sleep(behind)
    thread.join(timeout=5.0)

    # Let the cube come to rest after the release.
    for _ in range(120):
        machine.tick(dt)
        world.step(render=False)
    after = cube_xy_mm(index, cube.color)
    link.close()

    print("\n-- what the control repo's pick/place did ---------------------------")
    for stage in ("home", "pick", "place"):
        outcome = result.get(stage)
        state = "missing" if outcome is None else ("ok" if outcome.get("ok") else outcome.get("error"))
        print(f"  {stage:6s} {state}")

    print("\n-- where the cube went ---------------------------------------------")
    print(f"  before  ({before[0]:7.1f}, {before[1]:7.1f}, {before[2]:7.1f})")
    print(f"  after   ({after[0]:7.1f}, {after[1]:7.1f}, {after[2]:7.1f})")
    print(f"  asked   ({target[0]:7.1f}, {target[1]:7.1f})")
    landed = math.dist(after[:2], target)
    print(f"  landed {landed:.1f}mm from the place target")

    failures = []
    for stage in ("home", "pick", "place"):
        outcome = result.get(stage)
        if outcome is None or not outcome.get("ok"):
            failures.append(f"{stage} failed: {outcome}")
    if landed > PLACE_TOLERANCE_MM:
        failures.append(
            f"the cube is {landed:.1f}mm from where place() put it "
            f"(tolerance {PLACE_TOLERANCE_MM:.0f}mm) -- the grip slipped or "
            f"never closed"
        )

    print("\n====================================================================")
    if failures:
        print(f"{len(failures)} check(s) FAILED:")
        for failure in failures:
            print(f"  - {failure}")
        return 1
    print("the control repo picked and placed a simulated cube")
    return 0


if __name__ == "__main__":
    code = main()
    app.close()
    sys.exit(code)
