"""Pick and place a simulated cube with the control repo's own motion stack.

Nothing in here plans a move. ``mt4_vision.pickplace.pick`` and ``place`` are
the functions the live rig's task scripts call, and they are given the live
rig's own ``vision_calibration.json``; the only thing this file supplies is
which cube to move and where to put it, which is normally the camera's job.
If this works, the firmware substitute is good enough for the real stack.

    python scripts/serve_firmware.py            # in one terminal
    python scripts/demo_pick_place.py           # in another

Add ``--color`` to move a different cube, ``--to X Y`` to choose where it lands.
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "scripts"))

from run_against_sim import DEFAULT_URL, patch_serial  # noqa: E402

from mt4_sim import rig  # noqa: E402


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--url", default=DEFAULT_URL)
    ap.add_argument("--color", default="red", help="which scene cube to move")
    ap.add_argument(
        "--to",
        nargs=2,
        type=float,
        metavar=("X", "Y"),
        help="robot-frame mm to place it at (default: 60mm further out in +X)",
    )
    ap.add_argument(
        "--skip-home",
        action="store_true",
        help="the arm has already homed this session",
    )
    args = ap.parse_args()

    patch_serial(args.url)

    from mt4_jog.client import Mt4Client
    from mt4_vision.calib import load_calibration
    from mt4_vision import pickplace

    cube = next((c for c in rig.CUBES if c.color == args.color), None)
    if cube is None:
        ap.error(f"no {args.color} cube in the scene; have "
                 f"{[c.color for c in rig.CUBES]}")
    target = tuple(args.to) if args.to else (cube.x_mm + 60.0, cube.y_mm)

    calib = load_calibration()
    client = Mt4Client(port=args.url)
    try:
        if not args.skip_home:
            print("homing...")
            result = client.home()
            if not result.get("ok"):
                print(f"home failed: {result.get('error')}")
                return 1

        print(f"pick {args.color} at ({cube.x_mm:.1f}, {cube.y_mm:.1f})")
        result = pickplace.pick(
            client, calib, cube.x_mm, cube.y_mm, yaw_deg=cube.yaw_deg
        )
        if not result.get("ok"):
            print(f"pick failed: {result.get('error')}")
            return 1

        print(f"place at ({target[0]:.1f}, {target[1]:.1f})")
        result = pickplace.place(client, calib, target[0], target[1])
        if not result.get("ok"):
            print(f"place failed: {result.get('error')}")
            return 1

        pose = client.get_tcp()
        print(f"done; TCP at ({pose.x:.1f}, {pose.y:.1f}, {pose.z:.1f}) grip {pose.grip:.0f}")
        return 0
    finally:
        client.close()


if __name__ == "__main__":
    sys.exit(main())
