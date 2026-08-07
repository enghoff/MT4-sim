"""The simulated rig's layout, in the arm's home-angle frame (millimetres).

Where the real rig has been measured, these mirror the measurement so a
coordinate means the same thing in both places. Where it has not, the value is
a plausible stand-in and says so.
"""

from __future__ import annotations

from dataclasses import dataclass

from mt4_sim.chain import DESK_Z_MM

# --------------------------------------------------------------------------
# Where the arm stands, and where the work surface is
# --------------------------------------------------------------------------

# z = 0 is a modelling origin: the plane 140mm straight below the J2 shoulder
# pivot on the J1 turning axis, inherited from the factory link geometry. The
# CAD agrees the arm's own column is 140mm from that plane up to the shoulder,
# so drawing the column at full height puts its foot at z = 0.
#
# The work surface is 120mm up from there (`DESK_Z_MM`, where the gripper pads
# reach the desk). So the arm's column passes straight through that height, and
# the desk cannot extend underneath it -- it is a raised surface standing in
# front of the arm, on the same bench.
BENCH_TOP_Z_MM = 0.0
BENCH_X_MM = (-320.0, 700.0)
BENCH_Y_MM = (-500.0, 500.0)
BENCH_THICKNESS_MM = 20.0
BENCH_RGB = (0.30, 0.31, 0.33)

# The near edge clears the base's 140mm-wide foot with a few mm to spare. Only
# the far edge of the real desk is measurable (`calibrate_table_edge.py`) -- the
# others run past the arm's reach -- so these are the sim's own bounds, chosen
# to cover the reachable annulus.
DESK_X_MM = (75.0, 620.0)
DESK_Y_MM = (-420.0, 420.0)
DESK_THICKNESS_MM = 22.0
# The riser holding the surface up off the bench, inset so it reads as a table.
DESK_RISER_INSET_MM = 70.0

# The real desk is wood. Red cubes' shaded faces and the table share the orange
# hue band (`mt4_vision.detect` says so, which is why there is no "orange" cube
# colour), so a wood-toned surface keeps HSV detection behaving as it does live.
DESK_RGB = (0.62, 0.44, 0.26)

# `calibrate_table_edge.py` needs the wall above the desk visible, so the rig
# has one behind the arm, rising from the bench.
WALL_X_MM = BENCH_X_MM[0]
WALL_TOP_Z_MM = 560.0
WALL_RGB = (0.86, 0.85, 0.82)


# --------------------------------------------------------------------------
# Scene camera
# --------------------------------------------------------------------------

# The real rig's mount was measured 2026-07-25 (`calibrate_camera_nadir.py`):
# nadir (the point the lens looks straight down at) at robot (518, -35), lens
# 244mm above the table plane. That is steeply oblique -- the nadir lands well
# off the desk, so height parallax runs radially outward from it.
#
# The sim keeps that character (lens off the far +X side, aimed back across the
# desk) but sits higher, because a lens 244mm above the plane needs about a
# 120-degree field to cover the work area and that is a fisheye. The real rig's
# intrinsics are not recorded anywhere: its calibration is a homography fit
# straight from tag pixels to robot millimetres and deliberately needs none.
CAM_POSITION_MM = (620.0, -60.0, DESK_Z_MM + 420.0)
CAM_TARGET_MM = (240.0, 0.0, DESK_Z_MM)
CAM_RESOLUTION = (1280, 720)
# The optical axis meets the desk ~570mm out, so 50 degrees covers the whole
# reachable annulus with margin.
CAM_HORIZONTAL_FOV_DEG = 50.0

# Where the real rig's camera was measured, kept for comparison: `check.py`
# reports the simulated camera's own nadir and height against these.
REAL_CAM_NADIR_XY_MM = (518.0, -35.0)
REAL_CAM_HEIGHT_ABOVE_DESK_MM = 244.0


@dataclass(frozen=True)
class Marker:
    """A printed ArUco tag taped to the desk. ``tag_id`` is its printed number."""

    tag_id: int
    x_mm: float
    y_mm: float
    yaw_deg: float = 0.0


# DICT_4X4_50, 50mm squares -- the printable sheet the real rig uses
# (`docs/ArUco Markers A4 5x5cm.pdf`). Laid out across the reachable annulus:
# every centre is beyond the 157mm innermost holdable radius at table height
# and inside reach.
MARKER_SIZE_MM = 50.0
MARKER_DICT = "4x4_50"
MARKERS: tuple[Marker, ...] = (
    Marker(1, 190.0, 95.0),
    Marker(2, 190.0, -95.0),
    Marker(3, 285.0, 95.0),
    Marker(4, 285.0, -95.0),
    Marker(5, 285.0, 0.0),
    Marker(6, 190.0, 0.0),
)


@dataclass(frozen=True)
class Cube:
    color: str
    x_mm: float
    y_mm: float
    yaw_deg: float = 0.0


# 20mm, matching `Calibration.cube_height_mm`. Colours are the four the HSV
# detector knows (`mt4_vision.detect.COLOR_RANGES`).
CUBE_SIZE_MM = 20.0
CUBE_RGB = {
    "red": (0.72, 0.09, 0.09),
    "green": (0.10, 0.48, 0.16),
    "blue": (0.09, 0.24, 0.70),
    "yellow": (0.86, 0.72, 0.09),
}
CUBES: tuple[Cube, ...] = (
    Cube("red", 225.0, 175.0),
    Cube("green", 225.0, -175.0),
    Cube("blue", 330.0, 60.0, 25.0),
    Cube("yellow", 330.0, -60.0, -15.0),
)

DESK_TOP_Z_MM = DESK_Z_MM
