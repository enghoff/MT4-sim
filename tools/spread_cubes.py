#!/usr/bin/env python3
"""Pick cube positions that are as far apart as the rig actually allows.

``mt4_sim.rig.CUBES`` used to be placed by hand, which meant the constraints
lived in a comment and the spacing was whatever survived the last edit. This
maps every place a cube is *permitted* to be, then takes the arrangement with
the largest smallest gap, and prints it ready to paste back into ``rig.py``.

Eight things bound the region, and every one of them is binding somewhere:

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
  the quiet zone. A cube parked on a tag costs a decode;
* **imaged at an area the live detector accepts**. Its gates are fixed pixel
  counts and this camera is steeply oblique, so a cube's blob spans a factor of
  nine across the desk and the ends of that range fall out of the gates;
* clear of every stack site by more than `SITE_CLEAR_MM`, so the run never has
  to shove a cube aside to build, and out of every site's **forearm shadow** at
  the tallest column a pick
  faces. A cube directly beyond a site is reachable on its own and unroutable
  once the column is up, so it is held back and the run ends with it in view.

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
from mt4_vision.detect import MAX_BLOB_AREA, MIN_BLOB_AREA  # noqa: E402
from mt4_vision.pickplace import near_camera_park  # noqa: E402
from mt4_vision.scene import PICK_MAX_AREA, PICK_MIN_AREA  # noqa: E402
from mt4_vision.stackpath import StackPlanner  # noqa: E402

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

# `mt4_vision` gates cube blobs on fixed pixel areas, and a blob outside them is
# not mislocated but *gone*: under the floor `detect_cubes` never reports the
# cube, over the ceiling `scene.is_phantom_detection` refuses it as a pick
# target and the task script reports no reachable cube. Both gates are pairs and
# the tighter of each pair is the one that bites.
BLOB_FLOOR_PX2 = max(MIN_BLOB_AREA, PICK_MIN_AREA)
BLOB_CEILING_PX2 = min(MAX_BLOB_AREA, PICK_MAX_AREA)
# Those numbers were measured on cubes sitting on the markers, where a 20mm top
# face covers 417-1361 px^2. The camera is 242mm up and steeply oblique, so the
# same cube spans a factor of nine between the ends of the work region and both
# gates are reachable inside it. Leaving the ceiling to chance cost two of the
# nine cubes: at (244,120) and (240,-180) they read 5599 and 5596 px^2 against a
# PICK_MAX_AREA of 5000, were dropped as phantoms, and stopped a nine-level
# `stack_cubes` run at seven with both sitting plainly on the desk.
#
# The margin is what the silhouette's own error needs. It over-predicts the
# thresholded blob, landing at 0.92-0.99 of it over the nine cubes, so the floor
# has to be cleared by 1/0.92 = 8.7% before the blob itself clears it; the
# ceiling is safe at zero and takes the same figure as slack against lighting.
# It is not free: 0.10 costs 3 mm of the closest pair against an unmargined fit,
# 0.15 costs 19 mm and 0.25 costs 36 mm.
BLOB_MARGIN = 0.10
# Turning a cube swings its silhouette 16-26%, and its yaw is not this tool's to
# choose -- the gripper leaves it wherever the last grasp did. Both bounds have
# to hold at every yaw, so the ceiling is tested against the worst one and the
# floor against the best. Quarter-turn symmetry makes 0-90 the whole range.
YAW_SAMPLES_DEG = tuple(range(0, 90, 15))

PROBES = np.array([o["robot"] for o in calibration.CALIBRATION.probe_observations], float)
PROBE_LO, PROBE_HI = PROBES.min(axis=0), PROBES.max(axis=0)

# Every marker `stack_cubes` will accept as a stack site. It refuses the one
# under the camera-park pose, because the arm has to stand there to take the
# picture.
STACK_SITES = tuple(
    m for m in rig.MARKERS if not near_camera_park(m.x_mm, m.y_mm)
)


# Every gate downstream is applied to where vision *says* a cube is, not where
# it is, so a position that only just clears one is not clear at all. The sim's
# nine read back 5.4mm from truth on average and 11mm at worst, so boundaries
# are held off by more than that: a cube at (180,-240) sits outside marker 2's
# forearm shadow at every stack height and read 13mm out, at (192,-235), which
# is inside it -- and the run ended a level early with the cube in plain view.
READ_ERROR_MM = 20.0
# Cubes nearer a site than `stack_cubes.SITE_CLEAR_MM` are shoved aside before
# building, and the landing it picks is only held to the work region and the
# shadow -- not to the area gates above. Measured: clearing a cube off marker 2
# put it at (266,-110), where it images 5973 px^2, over `PICK_MAX_AREA`, and it
# was never picked. Starting outside the radius means the shove never happens.
SITE_CLEAR_MM = 70.0


def _clear_of(predicate, x: float, y: float, margin: float) -> bool:
    """Is the whole disc of radius ``margin`` about (x, y) outside ``predicate``?"""
    if predicate(x, y):
        return False
    return not any(
        predicate(x + margin * math.cos(a), y + margin * math.sin(a))
        for a in np.linspace(0.0, 2.0 * math.pi, 12, endpoint=False)
    )


def column_shadows(sites, count: int):
    """Each site's forearm shadow, at the tallest column a pick ever faces.

    Placing level N picks with N-1 cubes standing, so the last pick of a
    ``count``-level run is the worst case and nothing is ever reached over a
    full column. A cube inside one of these is not mislocated and not
    unreachable in itself -- ``StackPlanner`` refuses the *route*, because the
    forearm would cross over the standing column to get to it, so
    ``stack_candidates`` holds it back and the run ends early with the cube
    sitting in plain view.

    The two site-dependent bounds are set against **one** site rather than all
    four. Together they cost the whole region: four keep-clear discs of 104mm
    and four shadow wedges leave 18 of 12800 cells and cubes 5.7mm apart, where
    `mt4_vision.workspace.PICK_CLEARANCE_MM` alone wants 45mm between any two.
    So the layout is generated for the site it will be built on, and `--site`
    regenerates it for another.
    """
    planners = (StackPlanner(calibration.CALIBRATION, m.x_mm, m.y_mm) for m in sites)
    return [s for s in (p.column_shadow(count - 1) for p in planners) if s is not None]


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


def blob_span_px2(x: float, y: float) -> tuple[float, float]:
    """Smallest and largest area a cube here covers in the frame, over its yaw."""
    areas = [
        calibration.cube_silhouette_px2(rig.SCENE_CAMERA, x, y, rig.CUBE_SIZE_MM, yaw)
        for yaw in YAW_SAMPLES_DEG
    ]
    return min(areas), max(areas)


def detectable(x: float, y: float, margin: float) -> bool:
    """Does a cube here image inside the live detector's area gates, at any yaw?"""
    smallest, largest = blob_span_px2(x, y)
    return (
        smallest >= BLOB_FLOOR_PX2 * (1.0 + margin)
        and largest <= BLOB_CEILING_PX2 * (1.0 - margin)
    )


def feasible_anywhere(x: float, y: float, max_radius_mm: float, blob_margin: float) -> bool:
    """The bounds that hold wherever the stack is built."""
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
    if x > rig.DESK_FRONT_X_MM - DESK_EDGE_MARGIN_MM:
        return False
    # Last, because it is the costliest test here.
    return detectable(x, y, blob_margin)


def feasible(
    x: float, y: float, max_radius_mm: float, blob_margin: float, sites, shadows
) -> bool:
    """Everything above, plus the two bounds that depend on where the stack goes.

    Both are held off by ``READ_ERROR_MM``, because they are applied downstream
    to where vision says the cube is rather than where it is.
    """
    if not feasible_anywhere(x, y, max_radius_mm, blob_margin):
        return False
    if any(
        math.hypot(x - m.x_mm, y - m.y_mm) < SITE_CLEAR_MM + CUBE_CIRCUM_MM + READ_ERROR_MM
        for m in sites
    ):
        return False
    return all(_clear_of(shadow, x, y, READ_ERROR_MM) for shadow in shadows)


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
    ap.add_argument("--site", type=int, default=None,
                    help="marker the stack will be built on. Adds its "
                         "keep-clear radius and forearm shadow to the bounds, "
                         "which costs spacing; off by default")
    ap.add_argument("--blob-margin", type=float, default=BLOB_MARGIN,
                    help="headroom kept inside the detector's area gates "
                         "(default {:.2f})".format(BLOB_MARGIN))
    args = ap.parse_args(argv)

    sites: list = []
    if args.site is not None:
        sites = [m for m in STACK_SITES if m.tag_id == args.site]
        if not sites:
            print(f"marker {args.site} is not a stack site; "
                  f"{sorted(m.tag_id for m in STACK_SITES)} are")
            return 1
    xs = np.arange(60.0, 380.0, args.step)
    ys = np.arange(-320.0, 320.0, args.step)
    shadows = column_shadows(sites, args.count)
    print(
        f"laid out for a {args.count}-level stack on marker {args.site} "
        f"({sites[0].x_mm:.1f},{sites[0].y_mm:.1f})"
        if sites
        else f"laid out for {args.count} cubes, no stack site assumed"
    )
    points = np.array(
        [(x, y) for x in xs for y in ys
         if feasible(x, y, args.max_radius, args.blob_margin, sites, shadows)]
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

    print(f"{'position':>18}  {'radius':>7}  {'nearest cube':>13}  {'nearest card':>13}"
          f"  {'blob over yaw':>17}")
    for i, (x, y) in enumerate(chosen):
        others = np.delete(chosen, i, axis=0)
        to_cube = float(np.min(np.linalg.norm(others - (x, y), axis=1)))
        to_card = min(math.hypot(x - m.x_mm, y - m.y_mm) for m in rig.MARKERS)
        clear = to_card - CARD_CIRCUM_MM - CUBE_CIRCUM_MM
        smallest, largest = blob_span_px2(x, y)
        print(f"  ({x:6.1f},{y:7.1f})  {math.hypot(x, y):7.0f}  "
              f"{to_cube:11.0f}mm  {to_card:8.0f}mm ({clear:+.0f} clear)"
              f"  {smallest:6.0f}-{largest:6.0f}px2")
    print(f"\n  the live detector wants {BLOB_FLOOR_PX2:.0f}-{BLOB_CEILING_PX2:.0f} px2 "
          f"(sought {BLOB_FLOOR_PX2 * (1 + args.blob_margin):.0f}-"
          f"{BLOB_CEILING_PX2 * (1 - args.blob_margin):.0f} at margin "
          f"{args.blob_margin:.2f})")
    return 0


if __name__ == "__main__":
    sys.exit(main())
