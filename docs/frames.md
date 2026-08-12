# Frames: what z = 0 is, and where the desk sits

The stage is authored in the arm's **home-angle frame**, the frame every
coordinate in the MT4 stack lives in.

**z = 0 is both the modelling origin and the work surface.** It is the plane
exactly 140 mm straight below the J2 shoulder pivot, on the J1 turning axis,
inherited from the factory link geometry — and it is the plane the arm's own
base stands on, so it is where the desk is. The MT4 sits *on* the desk.

**`Calibration.table_z` = 122 mm is not the desk.** It is a **TCP** height: the
Z the arm is commanded to in order to grip something lying on the table,
measured by touching the tags. The gripper's tongs hang 115 mm below the TCP, so
at a TCP of 122 their tips clear the wood by 7 mm and straddle the lower two
thirds of a 20 mm cube.

That one distinction is what makes three separate firmware numbers agree at
once, where before only two could:

| | | |
|---|---|---|
| `CENCER_HEIGHT` | 140 | the shoulder pivot, 140 mm above the desk it stands on |
| `Calibration.table_z` | 122 | the TCP height that grips something lying on that desk |
| `GROUND_Z_MM` | 115 | the TCP height at which the tong tips touch the wood — a floor, which is exactly what a floor should be |

The last row is the one that pins the gripper's length. A floor is the height at
which the lowest thing on the arm reaches the ground; tongs 115 mm long put the
tips on the wood at TCP = 115, so the firmware's floor and the gripper's measured
height are the same number for the same reason.

and it is consistent with `calibrate_table_edge.py` measuring the desk's back
edge at x ≈ −76, behind the J1 axis: the desk runs *past* the arm, because the
arm is standing on it.

The sim draws **one flat surface**, top at z = 0, spanning the measured back
edge out to the frame edges, with a **bay** cut around the base's footprint. The
bay is no longer load-bearing now that the arm stands on the wood rather than
through it, but it keeps the tabletop from being a static collider coincident
with the base's own foot.

Knobs: `calibration.desk_surface_z_mm()` (the wood), `chain.TCP_GRIP_Z_MM` (read
from the calibration), `chain.TONG_REACH_MM` (the gripper's measured height),
`rig.DESK_FRONT_X_MM` / `rig.DESK_HALF_Y_MM` for how far the surface runs.

## What the previous arrangement broke

Reading `table_z` as the wood put the surface at z = 122 with the arm's base
buried beneath it — and, because the gripper was then drawn with short jaws
rising from the pads, it put the **gripper body's underside at exactly a 20 mm
cube's top face** on every pick. The cube was pinned between the desk and the
gripper while the jaws tried to turn it, which is why a misaligned cube could
not be squared up: closing on one 20° off square turned it 1° and jammed on its
corners at a 27 mm gap.

With full-length tongs the same close turns it **20.0° of 20** and ends
face-gripped at 19.6 mm. `check_grip.py --yaw-error` is the test.

## The tongs are 115 mm because the gripper is

`HEAD_HEIGHT` = 14.43 mm is the whole drop from the wrist pivot to the pads, so
the head plate and the servo housing have to live above the TCP — but the tongs
themselves hang `TONG_REACH_MM` = 115 below it. That is what lets the gripper
reach a cube on the table without any part of the head touching it, and it is
what real jaws that clear a tall object look like.

115 is measured off the real gripper, not derived. The CAD cannot supply it: the
STEP assembly carries the soft gripper (`两只柔爪2022`) as named parts with no
B-rep attached, so `tools/step_assembly.py` finds the mount and the J4 stepper
but nothing below them.

The number this replaces was 122 — `TCP_GRIP_Z_MM - DESK_Z_MM`, on the
assumption that the tongs reached exactly to the wood. Seven millimetres, and it
was the wrong seven:

- **The tips clear the wood by 7 mm at grip height**, straddling the lower two
  thirds of a 20 mm cube, rather than resting on the desk.
- **`GROUND_Z_MM` = 115 is now a floor and nothing stranger.** At 122-long tongs
  it drove the tips 7 mm *into* the desk, which jammed the jaws solid — measured,
  they would not close at all. A host that floored the Z and then gripped got
  nothing. At 115 the tips land exactly on the wood.

## Related

- [kinematics.md](kinematics.md) — the chain these heights hang off.
- [scene.md](scene.md) — the desk, tags and cubes authored in this frame.
