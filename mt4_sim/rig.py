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

# One flat surface, top at `DESK_Z_MM` -- `Calibration.table_z`, the plane the
# arm touches and the tags are taped to. There is no second level: the arm is
# mounted at the desk's back edge with its base below the surface, which is why
# `calibrate_table_edge.py` measured that edge running past the arm's own
# footprint rather than in front of it.
#
# The height is not a choice. The arm's shoulder pivot is 140mm above the frame
# origin and the surface is 122mm above it, so the pivot clears the surface by
# 18mm and everything below the shoulder is under the desk. That is what the
# firmware's `GROUND_Z_MM` guard is for: the soft joint limits let the TCP be
# driven to z = 37, well inside the tabletop.
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

# The bay the arm sits in, cut back from the desk's rear edge. The rotating
# column sweeps a 67mm radius about the J1 axis and the static base is 140mm
# across, so the surface has to stop clear of both or the base yaw jams solid
# against a static collider.
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
# (505, 1), 242mm above the table -- aimed and zoomed to reproduce the
# calibration's own pixel<->table map as closely as a distortion-free pinhole
# can. `check.py` prints how closely that is.
SCENE_CAMERA = calibration.scene_camera()
CAM_POSITION_MM = SCENE_CAMERA.position_mm
CAM_TARGET_MM = SCENE_CAMERA.target_mm
CAM_RESOLUTION = SCENE_CAMERA.resolution
CAM_HORIZONTAL_FOV_DEG = SCENE_CAMERA.horizontal_fov_deg


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
# Placed where every constraint holds at once: inside what the real camera's
# frame covers (which stops at x ~ 270, well short of the arm's 338mm reach at
# table height), clear of the tags since a cube parked on one costs a decode,
# and reachable through a whole pick -- not just at table height but at the
# +70mm the live stack transits at, where the annulus's inner edge jumps from
# radius 104 out to about 134 and would otherwise strand a cube it can grip.
# Nine cubes (three each of red, green, blue — the colours a nine-level
# stack_cubes run needs); centres stay ≥45mm apart and clear of the five tags.
CUBES: tuple[Cube, ...] = (
    Cube("red", 232.0, -95.0),
    Cube("green", 232.0, 95.0),
    Cube("blue", 135.0, -58.0, 25.0),
    Cube("green", 133.0, 60.0, -15.0),
    Cube("red", 265.0, -55.0, -10.0),
    Cube("green", 265.0, 55.0, 10.0),
    Cube("blue", 175.0, -90.0, -30.0),
    Cube("red", 175.0, 90.0, 30.0),
    Cube("blue", 145.0, 0.0, 20.0),
)

DESK_TOP_Z_MM = DESK_Z_MM


def desk_back_x_mm(y_mm: float) -> float:
    """Where the desk's measured back edge sits at this Y."""
    return DESK_BACK_SLOPE * y_mm + DESK_BACK_X_MM
