"""Plan one ``mp``/``mq`` leg the way the firmware plans it.

A leg is a straight world-frame line from wherever the arm is to the requested
TCP, chopped into ~2mm segments, each solved with the control repo's own
closed-form IK and turned into joint step counters. That is the firmware's
algorithm, run against the firmware's own kinematics module rather than a
reimplementation of it.

The one piece with real geometry of its own is the keep-out route. The TCP
cannot get closer than 140mm to the J1 axis, so a chord that would cut inside
the cylinder is replaced by tangent-arc-tangent around it -- out along a
tangent, round the boundary, back in along the other tangent -- which is what
``plan_mp_xy_route`` does on the device. A leg whose segments cannot all be
solved comes back as an error line in the firmware's own vocabulary, because
the host parses those.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

from mt4_sim.firmware.state import (
    CART_SEGMENT_MM,
    GRIPPER_S_CLOSED,
    GRIPPER_S_OPEN,
    JOG_SPEED_MAX_US,
    JOG_SPEED_MIN_US,
    KEEPOUT_MARGIN_MM,
    KEEPOUT_RADIUS_MM,
    MAX_SEGMENTS,
    FirmwareState,
)
from mt4_jog.kinematics import JointAnglesDeg, fk_tcp, ik_position, ws_j4_deg

# J4 field sentinels, matching Mt4J4Mode in the firmware's motion.h.
J4_EXPLICIT = "explicit"
J4_HOLD = "hold"
J4_WRIST = "wrist"

# How finely the keep-out arc is polylined. The arc is only ever a fraction of
# a turn at r=140, so 2mm of chord error would be visible in the executed path;
# 2 degrees keeps the polyline inside 0.02mm of the true arc.
_ARC_STEP_DEG = 2.0


@dataclass(frozen=True)
class Leg:
    """A planned leg: the step-counter waypoints the DDA will chase."""

    waypoints: list[tuple[int, int, int, int]]
    grip: int
    speed_us: int
    target_xyz: tuple[float, float, float]


def validate_request(
    state: FirmwareState, x: float, y: float, z: float, grip: int, speed_us: int
) -> str | None:
    """The cheap checks, in the firmware's order. Returns an error line or None.

    Order matters: the host distinguishes these, and ``mq`` runs exactly this
    set at enqueue time while the route feasibility waits until the leg pops.
    """
    if not state.homed:
        return "err not homed"
    if grip != 0 and not GRIPPER_S_OPEN <= grip <= GRIPPER_S_CLOSED:
        return f"err mp grip {GRIPPER_S_OPEN}-{GRIPPER_S_CLOSED}"
    if speed_us != 0 and not JOG_SPEED_MIN_US <= speed_us <= JOG_SPEED_MAX_US:
        return f"err mp speed {JOG_SPEED_MIN_US}-{JOG_SPEED_MAX_US}"
    if math.hypot(x, y) < KEEPOUT_RADIUS_MM - KEEPOUT_MARGIN_MM:
        return "err mp keepout"
    if z < state.ground_z_mm - 0.05:
        return f"err mp ground z<{state.ground_z_mm:.1f}"
    return None


# --------------------------------------------------------------------------
# XY route
# --------------------------------------------------------------------------


def _closest_approach(x0: float, y0: float, x1: float, y1: float) -> float:
    """How near the origin the straight XY chord gets."""
    dx, dy = x1 - x0, y1 - y0
    length_sq = dx * dx + dy * dy
    if length_sq < 1e-12:
        return math.hypot(x0, y0)
    t = -(x0 * dx + y0 * dy) / length_sq
    t = max(0.0, min(1.0, t))
    return math.hypot(x0 + t * dx, y0 + t * dy)


def _tangent_route(
    x0: float, y0: float, x1: float, y1: float, radius: float
) -> list[tuple[float, float]] | None:
    """Tangent, arc, tangent around a cylinder of ``radius`` about the J1 axis.

    Both ends must be outside the cylinder. Returns the polyline including both
    endpoints, taking whichever way round the boundary is shorter.
    """
    r0, r1 = math.hypot(x0, y0), math.hypot(x1, y1)
    if r0 < radius or r1 < radius:
        return None

    theta0, theta1 = math.atan2(y0, x0), math.atan2(y1, x1)
    alpha0 = math.acos(max(-1.0, min(1.0, radius / r0)))
    alpha1 = math.acos(max(-1.0, min(1.0, radius / r1)))

    best: tuple[float, list[tuple[float, float]]] | None = None
    for turn in (1.0, -1.0):
        # Leave the start on one side of the cylinder and rejoin on the same
        # side, so the arc between the two tangent points runs that way round.
        a = theta0 + turn * alpha0
        b = theta1 - turn * alpha1
        sweep = (b - a) % (2 * math.pi)
        if turn < 0:
            sweep = -((a - b) % (2 * math.pi))

        points = [(x0, y0)]
        count = max(1, int(abs(math.degrees(sweep)) / _ARC_STEP_DEG))
        for i in range(count + 1):
            angle = a + sweep * i / count
            points.append((radius * math.cos(angle), radius * math.sin(angle)))
        points.append((x1, y1))

        length = sum(
            math.dist(points[i], points[i + 1]) for i in range(len(points) - 1)
        )
        if best is None or length < best[0]:
            best = (length, points)
    return best[1] if best else None


def _polyline_length(points: list[tuple[float, float]]) -> float:
    return sum(math.dist(points[i], points[i + 1]) for i in range(len(points) - 1))


def _point_at(points: list[tuple[float, float]], distance: float) -> tuple[float, float]:
    """Walk ``distance`` along a polyline."""
    remaining = distance
    for i in range(len(points) - 1):
        span = math.dist(points[i], points[i + 1])
        if span <= 1e-12:
            continue
        if remaining <= span:
            f = remaining / span
            return (
                points[i][0] + f * (points[i + 1][0] - points[i][0]),
                points[i][1] + f * (points[i + 1][1] - points[i][1]),
            )
        remaining -= span
    return points[-1]


# --------------------------------------------------------------------------
# Leg planning
# --------------------------------------------------------------------------


def _segment_waypoints(
    state: FirmwareState,
    route: list[tuple[float, float]],
    z0: float,
    z1: float,
    ws_j4_start: float,
    ws_j4_end: float,
    near: JointAnglesDeg,
) -> list[tuple[int, int, int, int]] | None:
    """Chop the route into segments and solve each. None if any segment fails."""
    xy_len = _polyline_length(route)
    distance = math.hypot(xy_len, z1 - z0)
    count = max(1, min(MAX_SEGMENTS, math.ceil(distance / CART_SEGMENT_MM)))

    waypoints: list[tuple[int, int, int, int]] = []
    seed = near
    for i in range(1, count + 1):
        f = i / count
        x, y = _point_at(route, xy_len * f)
        z = z0 + (z1 - z0) * f
        solved = ik_position(x, y, z, near=seed, hold_orientation=False)
        if solved is None:
            return None
        # World yaw is what is held or interpolated; the J4 *joint* angle
        # follows from it and whatever J1 the segment solved to, which is the
        # 1:1 wrist unwind `orient on` describes.
        ws_j4 = ws_j4_start + (ws_j4_end - ws_j4_start) * f
        solved = JointAnglesDeg(solved.j1, solved.j2, solved.j3, ws_j4 - solved.j1)
        steps = state.steps_for(solved)
        if not state.within_soft_limits(steps):
            return None
        waypoints.append(steps)
        seed = solved
    return waypoints


def plan_leg(
    state: FirmwareState,
    x: float,
    y: float,
    z: float,
    j4_field: float,
    j4_mode: str,
    grip: int,
    speed_us: int,
) -> Leg | str:
    """Plan one leg from the arm's current counters. Returns a Leg or an error.

    The J4 sentinels resolve here rather than host-side, against the pose this
    leg actually starts from -- for a queued leg that is wherever the leg ahead
    of it ended, which is the whole reason the firmware resolves them late.
    """
    near = state.angles()
    ws_now = ws_j4_deg(near)

    target = ik_position(x, y, z, near=near, hold_orientation=False)
    if target is None:
        return "err mp unreachable"

    if j4_mode == J4_HOLD:
        ws_target = ws_now
    elif j4_mode == J4_WRIST:
        # Hold the J4 joint angle across the leg's J1 swing: world yaw follows
        # the base. A world hold across a big swing drives joint J4 = world -
        # j1 straight into its soft limit.
        ws_target = ws_now + (target.j1 - near.j1)
    else:
        ws_target = float(j4_field)

    target = JointAnglesDeg(target.j1, target.j2, target.j3, ws_target - target.j1)
    if not state.within_soft_limits(state.steps_for(target)):
        return "err mp joints"

    start = fk_tcp(near)

    routes: list[list[tuple[float, float]]] = []
    if _closest_approach(start.x, start.y, x, y) >= KEEPOUT_RADIUS_MM:
        routes.append([(start.x, start.y), (x, y)])
    # The firmware sweeps candidate cylinder radii for the smallest feasible
    # one; the keep-out radius itself is the one it settles on unless a soft
    # limit intrudes, so try it and then progressively wider.
    for extra in (0.0, 10.0, 25.0, 50.0):
        arc = _tangent_route(start.x, start.y, x, y, KEEPOUT_RADIUS_MM + extra)
        if arc is not None:
            routes.append(arc)
    if not routes:
        return "err mp route"

    for route in routes:
        waypoints = _segment_waypoints(state, route, start.z, z, ws_now, ws_target, near)
        if waypoints is not None:
            return Leg(
                waypoints=waypoints,
                grip=int(grip),
                speed_us=int(speed_us),
                target_xyz=(x, y, z),
            )
    return "err mp route"
