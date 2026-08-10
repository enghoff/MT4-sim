"""Assertions about the jaws that must hold in *any* run, hypothesis or not.

This exists because two serious defects sat undetected in logs that were already
on disk. A gripper travelled to its fully-open stop while commanded fully shut,
and cubes were dropped from a closed gripper in mid-transit -- both visible to
anyone who watched the recording, neither found by the analysis, because the
analysis only ever looked where it already suspected something. Every query
written that week was downstream of a story: "is the cube off-centre at the
grab", "how far did the pair walk". None of them was "did the gripper ever do
something it was not told to do".

So the point here is that nothing below takes an argument about what went wrong.
Each check is a property of a correct run, evaluated over the whole file, and it
reports violations whether or not anyone expected them. Run it on every log,
including the ones from runs that looked fine -- especially those.

    python scripts/check_invariants.py jaws.csv [jaws_cubes.csv]

Columns come from the per-step logger: t, s, target_mm, left_mm, right_mm,
gap_mm, vl, vr, tcp_x/y/z. The cube CSV is optional; without it the payload
checks are skipped and say so rather than silently passing.
"""
from __future__ import annotations

import bisect
import csv
import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from mt4_sim.chain import (  # noqa: E402
    FINGER_MAX_SPEED_M_S,
    GRIPPER_S_CLOSED,
)

# Room above the widest thing the jaws can legitimately be held open by.
GAP_OVERSHOOT_MM = 5.0

# A 20 mm cube gripped across its diagonal is the widest the blades should ever
# be pushed apart by something in this scene: 20*sqrt(2) = 28.3 mm.
WIDEST_OBJECT_MM = 28.3

# Commanded-shut. The firmware sweeps S, so anything above this is "closing or
# closed" rather than a specific value.
SHUT_S = 200.0

# A cube this close to the TCP in XY is between the blades rather than beside
# them, at the scale of a 20 mm cube and a 49 mm jaw span.
IN_JAWS_MM = 30.0

# Falling further than this is a drop, not settling onto a stack.
DROP_MM = 8.0

# How far the midpoint of the two blades may sit from the wrist. On the real
# gripper the answer is zero and not approximately zero: one servo drives both
# blades through a symmetric scissor, so the midpoint is a weld, not a
# tolerance. The number here is only the solver's own noise floor -- with the
# servo's torque limit applied to the pair rather than to each blade, a replayed
# place-down measures -0.117 .. +0.056 mm, so 1 mm is well clear of numerical
# slop and well under the excursions this catches.
MIDPOINT_TOL_MM = 1.0
# What separates a capture from a walk. The widest capture measured over ten
# trials is 3.32 mm and every one is back inside MIDPOINT_TOL_MM within 0.20 s;
# the failures this catches are 8-12.8 mm and hold for seconds. Both thresholds
# therefore sit with ~1.5x either side of a gap that is an order of magnitude
# wide, which is the only reason picking them is not a judgement call.
MIDPOINT_PEAK_MM = 5.0
MIDPOINT_YIELD_S = 1.5


def load(path: str) -> list[dict]:
    with open(path, newline="", encoding="utf-8") as fh:
        return list(csv.DictReader(fh))


def col(rows: list[dict], name: str) -> list[float]:
    return [float(r[name]) for r in rows]


def report(name: str, bad: list[tuple], detail) -> bool:
    """Print a check's verdict. Returns True when it passed."""
    if not bad:
        print(f"  PASS  {name}")
        return True
    print(f"  FAIL  {name}: {len(bad)} violation(s)")
    for item in bad[:5]:
        print(f"          {detail(item)}")
    if len(bad) > 5:
        print(f"          ... and {len(bad) - 5} more")
    return False


def check_gap_obeys_command(rows: list[dict]) -> bool:
    """Commanded shut, the gap never exceeds what an object could be holding open.

    The naive form of this -- ``gap <= target + margin`` -- fires thousands of
    times on a perfectly good run, and writing it that way first is instructive.
    The jaws are a servo stalled at its torque limit: the host commands *past*
    contact, so a gap of 19.7 mm against a commanded 0.00 is the correct reading
    for a 20 mm cube held properly. Resting above the target is the normal case.

    What is not normal is a gap wider than anything in the scene. Nothing here
    is broader than a cube, so while the gripper is commanded shut the blades
    have no business being further apart than one, plus room for a cube gripped
    across its diagonal. The fire-open reached 48.6 mm.
    """
    t, gap, s = col(rows, "t"), col(rows, "gap_mm"), col(rows, "s")
    bad = [
        (t[i], gap[i], s[i])
        for i in range(len(t))
        if s[i] > SHUT_S and gap[i] > WIDEST_OBJECT_MM + GAP_OVERSHOOT_MM
    ]
    return report(
        f"commanded shut, gap stays under {WIDEST_OBJECT_MM + GAP_OVERSHOOT_MM:.0f} mm",
        bad,
        lambda b: f"t={b[0]:8.2f}s  gap {b[1]:6.2f} mm with S={b[2]:.0f} (shut)",
    )


def check_jaw_speed(rows: list[dict]) -> bool:
    """Neither jaw exceeds the joint velocity cap it was configured with."""
    t, vl, vr = col(rows, "t"), col(rows, "vl"), col(rows, "vr")
    lim = FINGER_MAX_SPEED_M_S * 1.05
    bad = [
        (t[i], vl[i], vr[i])
        for i in range(len(t))
        if abs(vl[i]) > lim or abs(vr[i]) > lim
    ]
    return report(
        f"jaw speed stays under {FINGER_MAX_SPEED_M_S} m/s",
        bad,
        lambda b: f"t={b[0]:8.2f}s  vl={b[1]:+.3f} vr={b[2]:+.3f} m/s",
    )


def check_no_jaw_pinned_open(rows: list[dict]) -> bool:
    """No jaw sits on its open stop while the gripper is commanded shut.

    Distinct from the gap check: a single jaw can reach its stop while the pair
    has walked sideways, which the sum hides.
    """
    t, s = col(rows, "t"), col(rows, "s")
    left, right = col(rows, "left_mm"), col(rows, "right_mm")
    stop = 24.535 - 0.2
    bad = [
        (t[i], left[i], right[i])
        for i in range(len(t))
        if s[i] > SHUT_S and (left[i] > stop or right[i] > stop)
    ]
    return report(
        "no jaw on its open stop while commanded shut",
        bad,
        lambda b: f"t={b[0]:8.2f}s  left {b[1]:6.2f} right {b[2]:6.2f} mm",
    )


def check_jaw_midpoint_fixed(rows: list[dict]) -> bool:
    """The blades' midpoint stays on the wrist.

    ``left`` and ``right`` are half-gaps on two prismatic joints that share an
    origin on ``gripper_base`` and both count positive outward, so the pair has
    two coordinates and only one of them is held:

        gap = left + right      the drives hold this
        mid = (left - right)/2  nothing holds this

    ``mid`` is where the pair sits along the grip axis, measured from J4. The
    real gripper cannot move it at all. This one can, and does: both blades
    slide the same way, the gap does not change, and the whole assembly walks
    sideways under the wrist while the payload goes with it.

    The reason it is free is worth stating, because it is not the tendon. Two
    position drives to a common target *do* restore the midpoint, with force
    -2*k*mid -- but only while they can still modulate their force. Gripping,
    the host commands S=255, which is a negative span, so both drives are hard
    against ``FINGER_EFFORT_N`` and cannot push any harder on one side than the
    other. Two equal forces in opposite directions sum to nothing, and the
    restoring term is not weak but identically zero. The pair is then a free
    mass carrying ``FINGER_ARMATURE_KG`` on each side with no spring and no
    damper: whatever sideways velocity a transient hands it, it keeps, until a
    blade reaches a travel stop.

    Distinct from ``check_no_jaw_pinned_open``, which catches only the end of
    that walk, and from ``check_gap_obeys_command``, which by construction
    cannot see it -- the gap is exactly what a sideways walk leaves alone.

    **What it must not fire on is a capture.** Written as "every step stays
    inside MIDPOINT_TOL_MM" this failed a run that built its column perfectly:
    a pick aims at a *detected* cube position, 5.4 mm off on average, so one
    blade reaches the cube before the other and the pair yields while closing
    on it. That give is the thing that captures an off-centre cube instead of
    squeezing it out sideways -- README argues for it at length -- and it is
    over as soon as the cube is off the desk. Measured on marker 3, the same
    pick under both drives:

        drive                peak mid   back inside 1 mm after
        k=150 soft spring     3.32 mm          0.15 s
        servo, k=600          1.26 mm          0.20 s

    So the property of a correct run is not "never yields". It is **yields and
    comes back**: an excursion is a violation when it is bigger than a capture
    can produce, or when it lasts long enough to be a walk rather than a yield.
    The failures this exists for are both -- the transit walk reaches 12.8 mm,
    and the place-down slide holds 8 mm for seconds while the arm descends.
    """
    t = col(rows, "t")
    left, right = col(rows, "left_mm"), col(rows, "right_mm")
    mid = [(left[i] - right[i]) / 2.0 for i in range(len(t))]

    bad = []
    i = 0
    while i < len(mid):
        if abs(mid[i]) <= MIDPOINT_TOL_MM:
            i += 1
            continue
        j = i
        while j + 1 < len(mid) and abs(mid[j + 1]) > MIDPOINT_TOL_MM:
            j += 1
        peak = max(mid[i : j + 1], key=abs)
        held_s = t[j] - t[i]
        if abs(peak) > MIDPOINT_PEAK_MM or held_s > MIDPOINT_YIELD_S:
            bad.append((t[i], peak, held_s))
        i = j + 1

    return report(
        f"jaw midpoint yields no more than {MIDPOINT_PEAK_MM:.0f} mm and returns "
        f"inside {MIDPOINT_YIELD_S:.1f} s",
        bad,
        lambda b: f"t={b[0]:8.2f}s  mid peaked {b[1]:+6.2f} mm, "
                  f"outside {MIDPOINT_TOL_MM:.0f} mm for {b[2]:5.2f}s",
    )


def check_payload_not_dropped(rows: list[dict], cubes: list[dict]) -> bool:
    """A cube held in a shut gripper does not lose altitude *of its own*.

    The check the transit drops needed. It is deliberately about the *cube*
    rather than the jaws: whatever the gap does, a cube that was in the gripper
    and is now on the desk was dropped, and that is true without knowing why.

    The qualifier is load-bearing and its absence made this check useless. "Held
    in a shut gripper and losing altitude" describes a drop and it also describes
    every place-down ever commanded -- the gripper lowers 24 mm onto the column
    with the cube gripped the whole way. Written without it, this counted one
    violation per level: nine a run for an eight-high stack plus a clearing hop,
    on runs that placed all nine cubes perfectly. That is what made the drop rate
    look flat at ~50 per 100 grip-closes across every tendon setting while the
    real figure went 4 -> 13 -> 0.

    So the fall has to be one the TCP did not make with it.
    """
    if not cubes:
        print("  SKIP  payload never dropped (no cube log given)")
        return True
    jt = col(rows, "t")
    js, jx, jy = col(rows, "s"), col(rows, "tcp_x"), col(rows, "tcp_y")
    jz = col(rows, "tcp_z")
    ct = col(cubes, "t")
    names = [k[:-2] for k in cubes[0] if k.endswith("_x")]
    bad = []
    for n in names:
        x, y, z = (col(cubes, f"{n}_{a}") for a in ("x", "y", "z"))
        i = 1
        while i < len(z) - 1:
            if z[i] < z[i - 1] - 1.0:
                j = i
                while j < len(z) - 1 and z[j + 1] < z[j] - 0.5:
                    j += 1
                if z[i - 1] - z[j] > DROP_MM:
                    k = min(bisect.bisect_left(jt, ct[i - 1]), len(jt) - 1)
                    k2 = min(bisect.bisect_left(jt, ct[j]), len(jt) - 1)
                    held = math.hypot(x[i - 1] - jx[k], y[i - 1] - jy[k]) < IN_JAWS_MM
                    # What the cube lost that the gripper did not lower it by.
                    own = (z[i - 1] - z[j]) - (jz[k] - jz[k2])
                    if js[k] > SHUT_S and held and own > DROP_MM:
                        bad.append((ct[i - 1], n, z[i - 1], z[j], own))
                i = j + 1
            else:
                i += 1
    return report(
        "no cube falls out of a shut gripper",
        bad,
        lambda b: f"t={b[0]:8.2f}s  {b[1]} fell {b[2]:.0f} -> {b[3]:.0f} mm"
                  f"  ({b[4]:.0f} mm of it its own)",
    )


def main(argv: list[str]) -> int:
    if not argv:
        print(__doc__)
        return 2
    rows = load(argv[0])
    cubes = load(argv[1]) if len(argv) > 1 else []
    print(f"{Path(argv[0]).name}: {len(rows)} steps, {float(rows[-1]['t']):.0f}s")
    ok = all(
        [
            check_gap_obeys_command(rows),
            check_jaw_speed(rows),
            check_no_jaw_pinned_open(rows),
            check_jaw_midpoint_fixed(rows),
            check_payload_not_dropped(rows, cubes),
        ]
    )
    print("  ALL PASS" if ok else "  VIOLATIONS ABOVE")
    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
