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
    """A pinhole standing in for the rig's webcam, fitted to its table map.

    ``target_mm`` is a point on the optical axis rather than a place of
    interest: with the axis this close to horizontal, where it meets the wood
    runs off to a metre and a half out and is numerically useless to aim by.
    ``roll_deg`` turns the image about that axis, and ``principal_point_px`` is
    where the axis crosses the sensor -- both are ordinary camera parameters a
    USD camera carries, and both are needed to reproduce the rig's map.

    Pixels are square, and that is a constraint rather than a simplification:
    ``isaacsim.sensors.camera.Camera`` rewrites ``verticalAperture`` to match
    the resolution's aspect ratio whenever the two disagree, so a second focal
    length fitted here would be dropped on the way to the renderer and this
    object would stop describing the camera that took the picture. It costs
    0.7 px of the fit.
    """

    position_mm: tuple[float, float, float]
    target_mm: tuple[float, float, float]
    roll_deg: float
    horizontal_fov_deg: float
    principal_point_px: tuple[float, float]
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


# How far along the optical axis :attr:`SceneCamera.target_mm` is placed. Only
# the direction matters -- ``scene.look_at`` renormalises it -- so this is a
# readable distance and nothing depends on the value.
AXIS_POINT_MM = 1000.0


def _axis_from(yaw_rad: float, pitch_rad: float) -> np.ndarray:
    """Unit optical axis from a compass bearing and an elevation."""
    return np.array(
        [
            math.cos(pitch_rad) * math.cos(yaw_rad),
            math.cos(pitch_rad) * math.sin(yaw_rad),
            math.sin(pitch_rad),
        ]
    )


def _camera_basis(forward, roll_rad: float):
    """Optical axis, image +x and image +y (down), for a camera rolled by roll.

    At zero roll the image's x axis is world-horizontal, which is what
    ``scene.look_at`` builds from a bare eye/target pair. The roll term turns
    the pair about the axis, matching the ``roll_deg`` that function takes.
    """
    forward = np.asarray(forward, float)
    forward = forward / np.linalg.norm(forward)
    level_right = np.cross(forward, [0.0, 0.0, 1.0])
    level_right /= np.linalg.norm(level_right)
    level_down = np.cross(forward, level_right)
    cos_r, sin_r = math.cos(roll_rad), math.sin(roll_rad)
    return (
        forward,
        cos_r * level_right + sin_r * level_down,
        -sin_r * level_right + cos_r * level_down,
    )


def _project(lens, forward, roll_rad, focal_px, principal_px, points_mm, z_mm):
    """Robot XY at height ``z_mm`` -> pixel, for a general pinhole.

    ``focal_px`` and ``principal_px`` are (x, y) pairs: a camera whose two axes
    scale differently and whose axis does not cross the middle of the sensor is
    an ordinary camera, and the rig's map needs both to be reproduced.
    """
    lens = np.array(lens, float)
    forward, right, down = _camera_basis(forward, roll_rad)
    points_mm = np.atleast_2d(points_mm)

    rel = np.column_stack([points_mm, np.full(len(points_mm), z_mm)]) - lens
    depth = rel @ forward
    return (
        np.column_stack(
            [
                focal_px[0] * (rel @ right) / depth + principal_px[0],
                focal_px[1] * (rel @ down) / depth + principal_px[1],
            ]
        ),
        depth,
    )


def scene_camera() -> SceneCamera:
    """The sim's camera: the measured lens, oriented and zoomed to match the rig.

    The lens is pinned exactly where the rig measured it. Everything a pinhole
    with square pixels has left -- the three angles of its orientation, the
    focal length and the principal point -- is recovered by least squares
    against the calibration's own pixel<->table map.

    All six matter. Pinning the roll to level and the principal point to the
    middle of the sensor leaves a 3-parameter camera, and that camera
    reproduces the rig's map to 21 px (14 mm on the table) where the full one
    manages 5.4 px (4.2 mm). Those 14 mm are not a rendering nicety: they are
    added to every cube position the live stack reads out of a simulated frame,
    and they are what makes a simulated pick close its jaws beside the cube
    instead of on it. The principal point is the term that carries most of it,
    which is the same fact as the rig's own work area sitting low in its frame.

    What is left over is a genuine disagreement rather than a slack fit. With
    the lens free as well the map is reproduced exactly, at a lens 60 mm from
    the measured one -- so the rig's homography, which is a least-squares fit
    over lens distortion the projective model cannot carry, is not quite the
    map of *any* pinhole standing where the rig says the lens stands. The
    measured position is kept, because it is a measurement and because the
    parallax of anything with height hangs off it, and the residual is
    reported.
    """
    lens = np.array(lens_position_mm(), float)
    # The plane the camera is looking at is the wood, not the TCP height that
    # grips something lying on it.
    table_z = desk_surface_z_mm()
    width, height = resolution = frame_size_px()
    points, truth = _fit_samples()

    def unpack(params):
        yaw, pitch, roll, focal, cx, cy = params
        return _axis_from(yaw, pitch), roll, (focal, focal), (cx, cy)

    def residual(params):
        forward, roll, focal, principal = unpack(params)
        pixels, depth = _project(lens, forward, roll, focal, principal, points, table_z)
        if focal[0] <= 0.0 or depth.min() <= 1.0:
            return np.full(2 * len(points), 1e6)  # aimed behind its own lens
        return (pixels - truth).ravel()

    # Seed the orientation by aiming at the middle of the region being fit, so
    # the solver starts with the desk in frame rather than hunting for it.
    middle = points.mean(axis=0)
    to_middle = np.array([middle[0], middle[1], table_z]) - lens
    seed = [
        math.atan2(to_middle[1], to_middle[0]),
        math.asin(to_middle[2] / np.linalg.norm(to_middle)),
        0.0,
        700.0,
        width / 2.0,
        height / 2.0,
    ]
    solution = least_squares(residual, seed, xtol=1e-14, ftol=1e-14, max_nfev=20000)
    forward, roll, focal, principal = unpack(solution.x)

    pixels, _ = _project(lens, forward, roll, focal, principal, points, table_z)
    error_px = pixels - truth
    # What the mismatch costs downstream: a feature the sim renders at `pixels`,
    # read back through the live calibration, lands this far from the truth.
    read_back = np.array([CALIBRATION.pixel_to_robot(float(u), float(v)) for u, v in pixels])

    return SceneCamera(
        position_mm=tuple(float(v) for v in lens),
        target_mm=tuple(float(v) for v in lens + forward * AXIS_POINT_MM),
        roll_deg=math.degrees(roll),
        horizontal_fov_deg=math.degrees(2.0 * math.atan(width / 2.0 / focal[0])),
        principal_point_px=(float(principal[0]), float(principal[1])),
        resolution=resolution,
        residual_px=float(np.sqrt((error_px**2).sum(axis=1).mean())),
        residual_mm=float(np.sqrt(((read_back - points) ** 2).sum(axis=1).mean())),
    )
