"""The live vision calibration, read as scene geometry.

``vision_calibration.json`` is what the real stack believes about its own rig:
where the table plane is, where the ArUco tags are taped, where the lens sits.
The sim reads it rather than copying numbers out of it, the same way
:mod:`mt4_sim.chain` reads the firmware's kinematics -- so a recalibration on
the real rig is one ``build_scene.py`` away from being true in here too.

Three rig facts the file does not state outright are recovered from what it
does record:

* each tag's **orientation and printed size**, from the pixel corners in
  ``raw_marker_observations`` mapped onto the table plane through the same
  homography the live stack uses;
* the desk's **physical back edge**, which is the polygon's one measured side
  with ``calibrate_table_edge.EDGE_MARGIN_MM`` taken back off -- the stored
  polygon is the *usable* region, already inset toward the arm;
* the scene camera's **aim and field of view**. The file records where the lens
  is (nadir plus height) but no intrinsics, by design: the live calibration is a
  homography fit straight from tag pixels to millimetres and deliberately needs
  none. It does record that homography, and for a pinhole a plane homography
  *is* the aim and the focal length -- so those are fitted back out of it with
  the lens pinned where the rig measured it.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np
from scipy.optimize import least_squares

from mt4_sim import mt4_repo  # noqa: F401  (sys.path bootstrap)

from mt4_vision.calib import DEFAULT_CALIB_PATH, load_calibration

CALIBRATION = load_calibration()
CALIB_PATH = DEFAULT_CALIB_PATH

# `calibrate_table_edge.EDGE_MARGIN_MM`. The stored polygon is pulled this far
# back toward the arm from the edge that was measured, so a place command near
# the rim still lands on wood; the surface itself carries on that bit further.
EDGE_MARGIN_MM = 25.0


def _require(value, field: str):
    if value is None or (hasattr(value, "__len__") and len(value) == 0):
        raise RuntimeError(
            f"{CALIB_PATH} has no {field}; the sim's layout is read from it. "
            f"Run the calibration that writes it on the real rig first."
        )
    return value


def table_z_mm() -> float:
    """The table plane, in the arm's frame. Also the TCP Z that grips a cube."""
    return float(CALIBRATION.table_z)


def frame_size_px() -> tuple[int, int]:
    width, height = _require(CALIBRATION.frame_size_px, "frame_size_px")
    return int(width), int(height)


# --------------------------------------------------------------------------
# Markers
# --------------------------------------------------------------------------


@dataclass(frozen=True)
class MarkerPlacement:
    """Where one printed tag lies on the table, in the arm's frame."""

    tag_id: int
    x_mm: float
    y_mm: float
    yaw_deg: float
    side_mm: float


def marker_placements() -> tuple[MarkerPlacement, ...]:
    """The taped-down tags, from the observations the calibration was fit to.

    Centres are the arm's own touches -- the calibration's definition of where
    each tag is. Orientation and printed size come from the detected corners
    mapped onto the table plane: ArUco reports corners clockwise from the tag's
    printed top-left, so corner 0 -> corner 1 runs along the tag's own +x, and
    the angle that edge makes in the robot frame is the yaw to lay it at.
    """
    observations = _require(CALIBRATION.raw_marker_observations, "raw_marker_observations")

    placements = []
    for key, obs in sorted(observations.items(), key=lambda kv: int(kv[0])):
        corners = np.array([CALIBRATION.pixel_to_robot(*c) for c in obs["corners"]], float)
        edge = corners[1] - corners[0]
        sides = [float(np.linalg.norm(corners[(i + 1) % 4] - corners[i])) for i in range(4)]
        x_mm, y_mm = obs["robot"]
        placements.append(
            MarkerPlacement(
                tag_id=int(key),
                x_mm=float(x_mm),
                y_mm=float(y_mm),
                yaw_deg=math.degrees(math.atan2(edge[1], edge[0])),
                side_mm=float(np.mean(sides)),
            )
        )
    return tuple(placements)


def marker_side_mm() -> float:
    """The printed tags' black square, averaged over every observed corner."""
    return float(np.mean([m.side_mm for m in marker_placements()]))


# --------------------------------------------------------------------------
# Desk
# --------------------------------------------------------------------------


def desk_back_edge() -> tuple[float, float]:
    """The desk's measured back edge as ``x = slope * y + x_at_zero`` (mm).

    ``table_polygon_robot`` is one measured side plus three nominal ones parked
    outside the arm's reach; the measured one is the segment joining the first
    and last corners. The margin comes back off, so this is where the wood
    physically stops rather than where a place command is allowed to.
    """
    polygon = np.array(_require(CALIBRATION.table_polygon_robot, "table_polygon_robot"), float)
    (x0, y0), (x1, y1) = polygon[0], polygon[-1]
    slope = (x1 - x0) / (y1 - y0)
    return slope, x0 - slope * y0 - EDGE_MARGIN_MM


# --------------------------------------------------------------------------
# Scene camera
# --------------------------------------------------------------------------


@dataclass(frozen=True)
class SceneCamera:
    """A pinhole standing in for the rig's webcam, fitted to its table map."""

    position_mm: tuple[float, float, float]
    target_mm: tuple[float, float, float]
    horizontal_fov_deg: float
    resolution: tuple[int, int]
    residual_px: float
    residual_mm: float


def desk_surface_z_mm() -> float:
    """The work surface itself, in the arm's frame: the plane its base stands on.

    Not the same number as :func:`table_z_mm`. That one is a *TCP* height -- the
    Z the arm is commanded to in order to grip something lying on the table --
    and the gripper's tongs hang below the TCP, so the wood is that much lower
    than the TCP that reaches it. The tongs are as long as they need to be for
    the surface to land here, on the plane the arm's own base sits on, which is
    what puts the MT4 on top of the desk rather than sunk into it.
    """
    return 0.0


def lens_position_mm() -> tuple[float, float, float]:
    """The lens, from the nadir and height ``calibrate_camera_nadir.py`` fit.

    The height is measured above the *wood*, so it hangs off the surface plane.
    """
    nadir = _require(CALIBRATION.cam_xy_robot, "cam_xy_robot")
    height = _require(CALIBRATION.cam_height_mm, "cam_height_mm")
    return float(nadir[0]), float(nadir[1]), desk_surface_z_mm() + float(height)


def _fit_samples(step_mm: float = 10.0) -> tuple[np.ndarray, np.ndarray]:
    """Table points the calibration actually covers, with their real pixels.

    Bounded by the probe touches the homography was fit against, plus a little,
    so the fit is not steered by extrapolating the map past its own evidence.
    """
    probes = np.array(
        [p["robot"] for p in _require(CALIBRATION.probe_observations, "probe_observations")],
        float,
    )
    low, high = probes.min(axis=0) - 40.0, probes.max(axis=0) + 40.0
    grid = np.array(
        [
            (x, y)
            for x in np.arange(low[0], high[0] + step_mm, step_mm)
            for y in np.arange(low[1], high[1] + step_mm, step_mm)
        ]
    )
    pixels = np.array([CALIBRATION.robot_to_pixel(float(x), float(y)) for x, y in grid])

    width, height = frame_size_px()
    seen = (
        (pixels[:, 0] >= 0)
        & (pixels[:, 0] <= width)
        & (pixels[:, 1] >= 0)
        & (pixels[:, 1] <= height)
    )
    return grid[seen], pixels[seen]


def _project(lens, target, focal_px, points_mm, resolution):
    """Robot XY on the table -> pixel, for a level (unrolled) look-at pinhole."""
    width, height = resolution
    forward = np.array(target, float) - np.array(lens, float)
    forward /= np.linalg.norm(forward)
    right = np.cross(forward, [0.0, 0.0, 1.0])
    right /= np.linalg.norm(right)
    down = np.cross(forward, right)

    rel = np.column_stack([points_mm, np.full(len(points_mm), target[2])]) - np.array(lens, float)
    depth = rel @ forward
    return (
        np.column_stack(
            [
                focal_px * (rel @ right) / depth + width / 2.0,
                focal_px * (rel @ down) / depth + height / 2.0,
            ]
        ),
        depth,
    )


def scene_camera() -> SceneCamera:
    """The sim's camera: the measured lens, aimed and zoomed to match the rig.

    The lens is pinned exactly where the rig measured it, leaving three numbers
    -- where it points and how wide it sees -- to be recovered by least squares
    against the calibration's own pixel<->table map. The residual is reported
    rather than hidden: a real webcam has barrel distortion that no pinhole can
    express, and it is the reason this lands tens of pixels out at the frame
    edges rather than exactly.
    """
    lens = lens_position_mm()
    # The plane the camera is looking at is the wood, not the TCP height that
    # grips something lying on it.
    table_z = desk_surface_z_mm()
    resolution = frame_size_px()
    points, truth = _fit_samples()

    def residual(params):
        target = (params[0], params[1], table_z)
        pixels, depth = _project(lens, target, params[2], points, resolution)
        if params[2] <= 0.0 or depth.min() <= 1.0:
            return np.full(2 * len(points), 1e6)  # aimed behind its own lens
        return (pixels - truth).ravel()

    solution = least_squares(residual, [0.0, 0.0, 700.0], xtol=1e-14, ftol=1e-14)
    target_x, target_y, focal_px = solution.x

    pixels, _ = _project(lens, (target_x, target_y, table_z), focal_px, points, resolution)
    error_px = pixels - truth
    # What the mismatch costs downstream: a feature the sim renders at `pixels`,
    # read back through the live calibration, lands this far from the truth.
    read_back = np.array([CALIBRATION.pixel_to_robot(float(u), float(v)) for u, v in pixels])

    return SceneCamera(
        position_mm=lens,
        target_mm=(float(target_x), float(target_y), table_z),
        horizontal_fov_deg=math.degrees(2.0 * math.atan(resolution[0] / 2.0 / focal_px)),
        resolution=resolution,
        residual_px=float(np.sqrt((error_px**2).sum(axis=1).mean())),
        residual_mm=float(np.sqrt(((read_back - points) ** 2).sum(axis=1).mean())),
    )
