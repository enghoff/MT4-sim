#!/usr/bin/env python3
"""Pick cube positions that are as far apart as the rig actually allows.

``mt4_sim.rig.CUBES`` used to be placed by hand, which meant the constraints
lived in a comment and the spacing was whatever survived the last edit. This
maps every place a cube is *permitted* to be, then takes the arrangement with
the largest smallest gap, and prints it ready to paste back into ``rig.py``.

Five things bound the region, and every one of them is binding somewhere:

* the firmware's **keep-out cylinder** -- ``KEEPOUT_RADIUS_MM`` about the J1
  axis, at any height, enforced in ``mt4_sim.firmware.machine``;
* **reachable through a whole pick** -- not just at table height but at the
  ``TRANSIT_MM`` the live stack lifts to, where the annulus is smaller. A cube
  you can grip but not carry is no use;
* a **radius ceiling** well inside what the IK will solve, because the last
  stretch of reach is the arm with a nearly straight elbow, where joint error
  swings the TCP a long way;
* inside the region the calibration was **fit** over, not merely inside the
  frame -- outside the probe hull the pixel<->table homography is extrapolating,
  and the live stack reads cube positions through that same map;
* clear of the **tag cards**, which are wider than the printed black square by
  the quiet zone. A cube parked on a tag costs a decode.

Usage::

    python tools/spread_cubes.py [--count N] [--max-radius MM]
"""

from __future__ import annotations

import argparse
import math
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from mt4_sim import calibration, rig  # noqa: E402
from mt4_sim.chain import TCP_GRIP_Z_MM, park_pose  # noqa: E402
from mt4_sim.firmware.state import KEEPOUT_RADIUS_MM  # noqa: E402
from mt4_sim.markers import quiet_zone_fraction  # noqa: E402
from mt4_jog.kinematics import ik_position  # noqa: E402

# Half-extents that hold whatever the yaw is, so nothing has to reason about
# rotated squares: a square's circumradius covers its worst orientation.
CUBE_CIRCUM_MM = rig.CUBE_SIZE_MM * math.sqrt(2) / 2.0
CARD_MM = rig.MARKER_SIZE_MM * quiet_zone_fraction()
CARD_CIRCUM_MM = CARD_MM * math.sqrt(2) / 2.0

TRANSIT_MM = 70.0        # what mt4_vision.pickplace lifts to between pick and place
KEEPOUT_MARGIN_MM = 10.0
TAG_GAP_MM = 8.0         # visible wood between a cube and the nearest card
FRAME_MARGIN_PX = 55.0   # the cube stands 20mm tall; keep its top in shot too
DESK_EDGE_MARGIN_MM = 30.0
PROBE_INSET_MM = 15.0

PROBES = np.array([o["robot"] for o in calibration.CALIBRATION.probe_observations], float)
PROBE_LO, PROBE_HI = PROBES.min(axis=0), PROBES.max(axis=0)


def reachable(x: float, y: float, z: float) -> bool:
    """Real IK at the real (x, y), so the J1 limits count and not just radius."""
    return ik_position(float(x), float(y), float(z), near=park_pose()) is not None


def in_frame(x: float, y: float) -> bool:
    width, height = calibration.frame_size_px()
    u, v = calibration.CALIBRATION.robot_to_pixel(float(x), float(y))
    return (
        FRAME_MARGIN_PX <= u <= width - FRAME_MARGIN_PX
        and FRAME_MARGIN_PX <= v <= height - FRAME_MARGIN_PX
    )


def feasible(x: float, y: float, max_radius_mm: float) -> bool:
    radius = math.hypot(x, y)
    if radius < KEEPOUT_RADIUS_MM + KEEPOUT_MARGIN_MM + CUBE_CIRCUM_MM:
        return False
    if radius > max_radius_mm:
        return False
    if not reachable(x, y, TCP_GRIP_Z_MM):
        return False
    if not reachable(x, y, TCP_GRIP_Z_MM + TRANSIT_MM):
        return False
    if not in_frame(x, y):
        return False
    if not (
        PROBE_LO[0] + PROBE_INSET_MM <= x <= PROBE_HI[0] - PROBE_INSET_MM
        and PROBE_LO[1] + PROBE_INSET_MM <= y <= PROBE_HI[1] - PROBE_INSET_MM
    ):
        return False
    if any(
        math.hypot(x - m.x_mm, y - m.y_mm) < CARD_CIRCUM_MM + CUBE_CIRCUM_MM + TAG_GAP_MM
        for m in rig.MARKERS
    ):
        return False
    if x < rig.desk_back_x_mm(y) + DESK_EDGE_MARGIN_MM:
        return False
    return x <= rig.DESK_FRONT_X_MM - DESK_EDGE_MARGIN_MM


def spread(points: np.ndarray, count: int, rounds: int = 60) -> np.ndarray:
    """The ``count`` cells whose closest pair is as far apart as possible.

    Greedy farthest-point for a starting set, then repeatedly move each point to
    whichever feasible cell buys it the most clearance from the others. Exact
    maximin is NP-hard; this settles within a cell or two of it on a grid this
    size and is deterministic, which matters more here than optimality.
    """
    chosen = [points[np.argmin(np.linalg.norm(points - points.mean(axis=0), axis=1))]]
    for _ in range(count - 1):
        gaps = np.min([np.linalg.norm(points - c, axis=1) for c in chosen], axis=0)
        chosen.append(points[int(np.argmax(gaps))])
    chosen = np.array(chosen)

    for _ in range(rounds):
        for i in range(count):
            others = np.delete(chosen, i, axis=0)
            gaps = np.min([np.linalg.norm(points - o, axis=1) for o in others], axis=0)
            chosen[i] = points[int(np.argmax(gaps))]
    return chosen


def closest_pair_mm(points: np.ndarray) -> float:
    return min(
        float(np.linalg.norm(points[i] - points[j]))
        for i in range(len(points))
        for j in range(i + 1, len(points))
    )


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--count", type=int, default=len(rig.CUBES))
    ap.add_argument("--max-radius", type=float, default=300.0,
                    help="ceiling on reach; the IK solves to 359 but the last "
                         "stretch is the arm at full extension (default 300)")
    ap.add_argument("--step", type=float, default=4.0, help="grid pitch in mm")
    args = ap.parse_args(argv)

    xs = np.arange(60.0, 380.0, args.step)
    ys = np.arange(-320.0, 320.0, args.step)
    points = np.array(
        [(x, y) for x in xs for y in ys if feasible(x, y, args.max_radius)]
    )
    if len(points) < args.count:
        print(f"only {len(points)} feasible cells; cannot place {args.count} cubes")
        return 1
    print(f"feasible cells: {len(points)} of {len(xs) * len(ys)}")

    chosen = spread(points, args.count)
    current = np.array([(c.x_mm, c.y_mm) for c in rig.CUBES])
    print(f"closest pair: {closest_pair_mm(current):.1f} mm now, "
          f"{closest_pair_mm(chosen):.1f} mm proposed\n")

    # Front to back, then across, so the pasted table reads in rig order.
    chosen = chosen[np.lexsort((chosen[:, 1], -chosen[:, 0]))]
    colours = [c.color for c in rig.CUBES[: args.count]]
    colours = sorted(colours, key=lambda c: ("red", "green", "blue").index(c))
    colours = [colours[i % 3 * (len(colours) // 3) + i // 3] for i in range(len(colours))]
    yaws = [0.0, 25.0, -15.0, 30.0, -10.0, 20.0, -30.0, 10.0, -20.0]

    print("CUBES: tuple[Cube, ...] = (")
    for (x, y), colour, yaw in zip(chosen, colours, yaws * 9):
        print(f'    Cube("{colour}", {x:.1f}, {y:.1f}, {yaw:.1f}),')
    print(")\n")

    print(f"{'position':>18}  {'radius':>7}  {'nearest cube':>13}  {'nearest card':>13}")
    for i, (x, y) in enumerate(chosen):
        others = np.delete(chosen, i, axis=0)
        to_cube = float(np.min(np.linalg.norm(others - (x, y), axis=1)))
        to_card = min(math.hypot(x - m.x_mm, y - m.y_mm) for m in rig.MARKERS)
        clear = to_card - CARD_CIRCUM_MM - CUBE_CIRCUM_MM
        print(f"  ({x:6.1f},{y:7.1f})  {math.hypot(x, y):7.0f}  "
              f"{to_cube:11.0f}mm  {to_card:8.0f}mm ({clear:+.0f} clear)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
