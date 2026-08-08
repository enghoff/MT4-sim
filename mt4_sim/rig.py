"""The simulated rig's layout, in the arm's home-angle frame (millimetres).

Most of it is not written down here at all: the work surface, the tags and the
camera are read out of the live rig's ``vision_calibration.json`` by
:mod:`mt4_sim.calibration`, so the sim's scene and the real stack's beliefs
about the scene cannot drift apart. What is written down here is the part the
calibration has no opinion on -- how far the desk runs once it leaves the
camera's view, what colour things are, and where the cubes start.
"""

from __future__ import annotations

from dataclasses import dataclass

from mt4_sim import calibration
from mt4_sim.chain import DESK_Z_MM

# --------------------------------------------------------------------------
# The work surface
# --------------------------------------------------------------------------

# One flat surface, top at `DESK_Z_MM` = 0 -- the plane the arm's own base
# stands on. The tags are taped to it and the gripper's tongs reach down to it;
# `Calibration.table_z` = 122 is the *TCP* height that puts the tong tips here,
# not the height of the wood. See `mt4_sim.chain` for why that reading is the
# one that makes CENCER_HEIGHT, table_z and GROUND_Z_MM agree.
#
# The surface runs past the arm rather than stopping in front of it, which is
# why `calibrate_table_edge.py` measured its back edge behind the J1 axis.
DESK_THICKNESS_MM = 22.0

# The one measured side of `table_polygon_robot`, margin removed: the line the
# wood actually stops on. It runs behind the arm and is slightly skew to Y.
DESK_BACK_SLOPE, DESK_BACK_X_MM = calibration.desk_back_edge()

# The other three sides are nominal in the calibration too ("well outside a
# 350mm reach"), so these are the sim's own: far enough out to fill the camera's
# frame, since a desk edge inside the frame would give the tag detector and
# `calibrate_table_edge.py` a second edge to find.
DESK_FRONT_X_MM = 560.0
DESK_HALF_Y_MM = 600.0

# The bay the arm sits in, cut back from the desk's rear edge. Only the *static*
# base needs clearing: per the CAD its 110x110 pedestal stands 20mm forward of
# the J1 axis on a 110x130 foot plate, so it reaches x = 75 and |y| = 65. The
# rotating body sweeps wider than that -- 73mm at the yoke, 94mm at the shoulder
# steppers -- but its underside is 54mm up, so it never comes near the wood.
DESK_BAY_FRONT_X_MM = 82.0
DESK_BAY_HALF_Y_MM = 78.0

# The real desk is wood. Red cubes' shaded faces and the table share the orange
# hue band (`mt4_vision.detect` says so, which is why there is no "orange" cube
# colour), so a wood-toned surface keeps HSV detection behaving as it does live.
DESK_RGB = (0.62, 0.44, 0.26)

# `calibrate_table_edge.py` finds the desk's back edge as the line where the
# wood gives way to whatever is behind it, so the rig needs something there.
# It has to stand off, though: J1's soft limits reach -137 deg, which swings the
# gripper back to x = -288, and a backdrop inside that is something the arm
# drives into. Deep enough below the desk to close the sightline that grazes
# the desk's back edge, and wide enough to fill the frame at that distance.
WALL_X_MM = -450.0
WALL_TOP_Z_MM = 900.0
WALL_BOTTOM_Z_MM = -120.0
WALL_HALF_Y_MM = 1500.0
WALL_RGB = (0.86, 0.85, 0.82)


# --------------------------------------------------------------------------
# Scene camera
# --------------------------------------------------------------------------

# The lens exactly where `calibrate_camera_nadir.py` measured it -- nadir
# (505, 1), 242mm above the table -- with the orientation, the two focal
# lengths and the principal point fitted to reproduce the calibration's own
# pixel<->table map. Every one of those seven is load-bearing: the live stack
# reads cube positions through that map, so whatever the fit leaves on the
# table is added to every pick. `check.py` prints how much that is.
SCENE_CAMERA = calibration.scene_camera()
CAM_POSITION_MM = SCENE_CAMERA.position_mm
CAM_TARGET_MM = SCENE_CAMERA.target_mm
CAM_RESOLUTION = SCENE_CAMERA.resolution
CAM_HORIZONTAL_FOV_DEG = SCENE_CAMERA.horizontal_fov_deg
CAM_ROLL_DEG = SCENE_CAMERA.roll_deg
CAM_PRINCIPAL_POINT_PX = SCENE_CAMERA.principal_point_px


# --------------------------------------------------------------------------
# Markers
# --------------------------------------------------------------------------

# The five tags the live calibration was fit against, where the arm touched
# them, turned the way the camera saw them. DICT_4X4_50 is the dictionary the
# live detector opens with; the calibration file does not record it.
MARKER_DICT = "4x4_50"
MARKERS = calibration.marker_placements()
# The printed black square, measured off the recorded corners: 44.3mm, which is
# a 6-cell code at 7.5mm a cell inside the 50mm tile the sheet is cut from.
MARKER_SIZE_MM = calibration.marker_side_mm()


# --------------------------------------------------------------------------
# Cubes
# --------------------------------------------------------------------------


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
# Nine cubes -- three each of red, green and blue, the colours a nine-level
# `stack_cubes` run needs -- spread as far apart as the rig allows rather than
# clustered in front of the arm. `tools/spread_cubes.py` picks them: it maps
# every place a cube is *permitted* to be and then takes the arrangement with the
# largest smallest gap, which comes out at 111mm between the closest pair where
# the hand-placed set managed 51mm.
#
# Five things bound that region at once, and all of them bite somewhere:
#
#   * the firmware's keep-out cylinder -- `KEEPOUT_RADIUS_MM` = 140 about the J1
#     axis, at any height -- which is why nothing sits closer in than r = 164
#   * reachable through a whole pick: not only at table height but at the +70mm
#     the live stack transits at, where the reachable annulus is smaller
#   * r <= 300, well inside the 359mm the IK will actually solve. The last few
#     tens of millimetres are the arm at full stretch, where the elbow is nearly
#     straight and joint error swings the TCP a long way
#   * inside the region the calibration was *fit* over, not merely inside the
#     frame. Outside the probe hull the pixel<->table homography is
#     extrapolating, and the live stack reads cube positions through that map
#   * clear of the five tag cards, which are 59mm across including the quiet
#     zone, not the 44.3mm of printed black -- a cube parked on one costs a
#     decode. Every cube keeps at least 8mm of visible wood to the nearest card.
CUBES: tuple[Cube, ...] = (
    Cube("red", 248.0, -44.0, 0.0),
    Cube("green", 244.0, 120.0, 25.0),
    Cube("blue", 240.0, -180.0, -15.0),
    Cube("red", 196.0, 224.0, 30.0),
    Cube("green", 156.0, 52.0, -10.0),
    Cube("blue", 132.0, -264.0, 20.0),
    Cube("red", 92.0, -136.0, -30.0),
    Cube("green", 92.0, 264.0, 10.0),
    Cube("blue", 80.0, 144.0, -20.0),
)

DESK_TOP_Z_MM = DESK_Z_MM


def desk_back_x_mm(y_mm: float) -> float:
    """Where the desk's measured back edge sits at this Y."""
    return DESK_BACK_SLOPE * y_mm + DESK_BACK_X_MM
