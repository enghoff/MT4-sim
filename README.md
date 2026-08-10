# MT4-sim — the WLKATA MT4 in Isaac Sim

An Isaac Sim model of the WLKATA MT4 desktop arm that lives in **the same
coordinate frame as the real arm's control stack**. A cube the live rig reports
at robot (230, −60) sits at (0.230, −0.060) on the stage, the arm's joint angles
are the same `q1..q4` the firmware speaks, and the gripper takes the same
firmware `S` value.

Geometry, joint limits and the gripper's jaw-span model are **read from the
control repo** ([enghoff/MT4](https://github.com/enghoff/MT4)) rather than
copied, so the simulated arm cannot drift from the firmware.

![the rig](out/preview.png)

## What is verified

`scripts/check.py` runs five checks against the live stack's own code and data,
and all five pass:

| Check | Result |
|---|---|
| The URDF chain reproduces `mt4_jog.kinematics.fk_tcp` | **0.0002 mm** unexplained across the soft-limit box |
| Position drives settle at the commanded pose | **≤ 0.04°** per joint |
| The simulated camera's ArUco tags decode | **5 of 5**, via `cv2.aruco` DICT_4X4_50 |
| Those tags, read back with the rig's own `vision_calibration.json` | **1.6–7.1 mm** and **≤ 1.3°** from where that file says they are taped |
| Cubes clear `mt4_vision.detect`'s own HSV thresholds | **3 of 3** colours, 1513–5724 px² blobs |

The last three matter more than they look: the simulated scene camera's frames
go straight into the real `mt4_vision.detect` and the real `Calibration` with
nothing adapted. A tag detected in a simulated frame, pushed through the live
homography, comes out where the live rig says that tag is — which is the
strongest available statement that the two scenes are the same scene — and the
nine cubes read back to **5.6 mm mean, 10.8 mm worst** of where the stage
actually put them.

The blob areas are the one number that does *not* sit inside what the rig
measured, and the spread is the point: pushed through the rig's own cube-top
map, a 20 mm cube's top face covers **281 px² at the far side of the work
region and 2573 px² at the near side** — a factor of 9 — because the camera is
242 mm up and steeply oblique. `mt4_vision` gates cube blobs on **fixed** areas
(`MIN_BLOB_AREA` 800, `MAX_BLOB_AREA` 6000, `PICK_MIN_AREA` 400, `PICK_MAX_AREA`
5000) measured on cubes sitting on the markers, where the top face is 417–1361
px². Cubes further out image past the cap and are dropped as phantoms — not
mislocated, *gone*.

Carrying the rig's *own* on-pad measurements across the work region says the
same thing without the sim in the argument: a cube that reads 2790–3627 px² on
a marker reads 523–6856 px² over the region the arm is allowed to work, which
puts 5.8–19.3% of it under `MIN_BLOB_AREA` and 0.9–8.7% over `PICK_MAX_AREA`.
That is the live stack's gate, not the sim's, and it is faithfully reproduced
here.

`tools/spread_cubes.py` therefore bounds the layout by it, predicting a cube's
blob as its projected **silhouette** — the whole cube, not its top face, which
is what a colour threshold actually outlines. That matches the detected blob to
within 3% (0.92–0.99 of it over the nine), and it is checked at every yaw,
because turning a cube swings its silhouette 16–26%. The nine now cover
1669–4496 px².

## Quick start

```powershell
$env:PY = "Z:\IsaacSim\venv\Scripts\python.exe"

& $env:PY scripts/build_urdf.py     # kinematics -> assets/mt4.urdf
& $env:PY scripts/import_urdf.py    # -> assets/mt4/mt4.usda   (~3 min first run)
& $env:PY scripts/build_scene.py    # -> assets/mt4_scene.usda
& $env:PY scripts/check.py          # verify + render to out/

& $env:PY scripts/run_sim.py            # open the GUI, arm parks and holds
& $env:PY scripts/run_sim.py --demo     # tour the desk: hover every tag and cube
& $env:PY -m unittest discover -s tests # chain maths + firmware protocol, no GPU

& $env:PY scripts/serve_firmware.py --camera  # be the arm's firmware + scene camera
& $env:PY scripts/check_firmware.py           # a control-repo pick, on a simulated cube
& $env:PY scripts/run_against_sim.py --check-camera
```

The control repo must be reachable. It defaults to the sibling `Z:\MT4`;
override with `MT4_REPO`. Driving the sim with control-repo code also needs its
`pyserial` in whatever interpreter runs the script.

## The parallel linkage, as a serial chain

The MT4 is a palletizer. J2 sets the upper arm's **absolute** angle, J3 sets the
forearm's **absolute** angle through a pair of link rods, and the head platform
stays level however the arm folds. A URDF chain has only *relative* joint
angles, so the model angles the control stack speaks are not the joint positions
the articulation takes. `mt4_sim.chain.urdf_from_model` converts:

| URDF joint | origin in parent (m) | axis | position |
|---|---|---|---|
| `j1_base_yaw` | (0, 0, 0.140) | +Z | `q1` |
| `j2_shoulder` | (0.045, 0, 0) | −Y | `q2` |
| `j3_elbow` | (0.130, 0, 0) | −Y | `q3 − q2` |
| `j_head_level` | (0.150, 0, 0) | −Y | `−q3` |
| `j4_wrist_roll` | (0.035, 0, −0.01443) | +Z | `q4` |

`j_head_level` is the link rods doing their job — it cancels the forearm's
rotation so the head hangs level, which is why `HEAD_OFFSET` is a horizontal
offset in the FK and not a rotating one. `j4_wrist_roll`'s origin **is** the TCP:
the roll axis passes through it, which is why `fk_tcp` has no `q4` term.

### Where the head-level joint is not the real thing

On the arm, rigid rods hold the head level. Here it is a driven joint, so it
gives up its drive's steady-state error to the gripper's weight, and a head
tilted by θ swings the TCP through `HEAD_OFFSET · sin θ`. That is the **only**
way the simulated TCP can disagree with `fk_tcp` once the arm has settled, and
`check.py` asserts exactly that identity rather than hiding it behind a loose
tolerance — which is what makes the 0.0002 mm figure above a real statement
about the chain.

At the drive stiffness used (12000 N·m/rad) the tilt is under 0.18° and the TCP
artefact under 0.11 mm. Both scale with the physics step — an implicit drive is
effectively softer at a coarser one — so they were half that when the scene ran
at 240 Hz and are what they are at the 60 Hz it runs at now. For scale, the real
arm's measured backlash is 6–9 mm on small reversing moves.

Modelling the rods properly needs a closed kinematic loop, which URDF cannot
express and USD can. Worth doing only if something starts caring about 0.1 mm.

## Frames: what z = 0 is, and where the desk sits

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

### What the previous arrangement broke

Reading `table_z` as the wood put the surface at z = 122 with the arm's base
buried beneath it — and, because the gripper was then drawn with short jaws
rising from the pads, it put the **gripper body's underside at exactly a 20 mm
cube's top face** on every pick. The cube was pinned between the desk and the
gripper while the jaws tried to turn it, which is why a misaligned cube could
not be squared up: closing on one 20° off square turned it 1° and jammed on its
corners at a 27 mm gap.

With full-length tongs the same close turns it **20.0° of 20** and ends
face-gripped at 19.6 mm. `check_grip.py --yaw-error` is the test.

### The tongs are 115 mm because the gripper is

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

## The CAD independently confirms the kinematics

`tools/step_assembly.py` parses the assembly in
`vendor/MT4-STL/MT4 - For All Users.step` (WLKATA's official CAD) and measures
the cylindrical bores shared between parts to recover the joint locations. Every
firmware constant checks out to better than 0.006 mm:

| measurement | CAD | firmware |
|---|---|---|
| upper arm, J2→J3 | 130.001 | 130 |
| forearm, J3→wrist | 150.000 | 150 |
| J2 above base bottom | 140.000 | 140 |
| wrist→TCP | +35.000 / −14.430 | 35 / 14.43 |
| link rod hole centres | 130.000 / 149.999 | 130 / 150 |
| **J1 axis → J2, horizontal** | **46.000** | **45** |

**The one disagreement is `CENCER_OFFSET`: 46 mm in CAD, 45 mm in the
firmware.** The J1 axis is unambiguous in CAD — five separate parts put it at the
same place, including an 8-hole M3 bolt circle centred exactly there. A 1 mm
radial offset is within the noise of the tape-fit that set the park pose, so this
is more likely a rounding in the firmware than a CAD error, but it is a real
1 mm bias on every radial coordinate and worth a decision.

The STEP's own assembly transforms are all identity — it is a flattened HOOPS
export with every part's geometry already in the assembly root frame — so the
transforms are useless but the geometry is authoritative.

### …and it corrects the two bodies J1 joins

The kinematics were right long before the *shapes* were. The base and the
rotating column were sketched to plausible proportions, and read far taller and
blockier than the real machine. The CAD says:

| | model was | CAD (above the desk) |
|---|---|---|
| foot plate | 140 × 120, 8 tall | **110 × 130**, 5 tall |
| pedestal | 104 × 92, running the full 8 → 76 | **110 × 110, stopping at 54** |
| J1 shroud | — | **85 × 72**, 54 → 75 |
| yoke | 96 × 84 to 142, plus a 62 × 64 cap to 162 | **106 × 64**, 76 → 160 |
| shoulder steppers | absent | **two 42 × 84 × 42**, 87 → 129, ±92 wide |
| base centre | on the J1 axis | **20 mm forward of it** |

Total height was close all along — 76 mm of base against the CAD's 75. What made
it read tall was the missing **step at 54 mm**: the real pedestal is a squat
110 × 110 box that necks down to a narrow shroud, where the model ran one
104 × 92 block the whole way up. The two shoulder steppers are the other half of
it — at 184 mm tip to tip they are the widest thing on the machine, and drawing
them as nothing left the column a plain tower.

Both bodies rotate or stand clear of the wood (the rotating body's underside is
54 mm up), so none of this is load-bearing for collision; it is what the arm
looks like, and `rig.DESK_BAY_*` now has the real footprint to clear.

### Real meshes are the obvious next step

The arm currently renders as boxes sized from the CAD bounding boxes:
dimensionally right, visually plain. The registration needed to swap in the
actual STL meshes is already computed — `tools/step_assembly.json` carries each
mesh's rotation and translation into its link frame, found by matching STEP
B-rep vertices against mesh vertices (`Link1.STL` registers at 97% with 0.0000 mm
residual). Two caveats found along the way: `base_link.STL` is a different
revision from this STEP and only matches 54%, and the mesh variants have plain
through-holes where the STEP has stepped bearing seats.

## The scene

Authored by `mt4_sim/scene.py`. Most of the layout is not written down in this
repo at all: `mt4_sim/calibration.py` reads the live rig's
`vision_calibration.json` and hands back the table height, the tags and the
camera, the same way `mt4_sim/chain.py` reads the firmware's kinematics. A
recalibration on the real rig is one `build_scene.py` away from being true here.

- **Work surface** — wood-toned, top face at z = 0 (the plane the arm's own base
  stands on; `Calibration.table_z` = 122 is a TCP height, see *Frames* above),
  running from the measured back edge out past the camera's frame, with a bay for
  the arm. The tone is not decoration: the live `mt4_vision.detect` has no
  "orange" cube colour precisely because the wood table and red cubes' shaded
  faces share that hue band.
- **Wall** well behind the arm, because `calibrate_table_edge.py` needs a
  backdrop to find the desk's edge against. It has to stand off: J1's soft limits
  reach −137°, which swings the gripper back to x = −288.
- **Five ArUco tags** — the ids, positions, orientations and printed size the
  live calibration was fit against. Centres are the arm's own touches; the yaw
  and the 44.3 mm black square are recovered from the pixel corners in
  `raw_marker_observations`, mapped onto the table through the same homography
  the live stack uses.
- **Nine cubes**, 20 mm, in the colours the HSV detector knows, at plastic-on-wood
  friction — see below, it is the number the gripper is most sensitive to.
  Positions come from `tools/spread_cubes.py` rather than by hand: it maps
  everywhere a cube is *permitted* to sit and takes the arrangement with the
  largest smallest gap, which puts the closest pair 99 mm apart where the
  hand-placed set managed 51 mm. Six things bound that region and every one of
  them binds somewhere — the firmware's 140 mm keep-out cylinder, reach at the
  +70 mm transit height as well as at the table, a 300 mm ceiling well inside the
  359 mm the IK will solve, the hull the calibration was actually *fit* over
  (outside it the pixel↔table map is extrapolating), the tag cards, which are
  59 mm across including the quiet zone rather than the 44.3 mm of printed black,
  and the **blob-area gates** above. Two more are available behind `--site`, both
  keyed to where the stack goes: the site's keep-clear radius, so the run never
  shoves a cube aside to start, and the standing column's forearm shadow. They
  are off by default — each is real, but together they cost 37 mm of the closest
  pair for one nominated marker, and enforcing them for all four candidates at
  once leaves 18 of 12800 cells and cubes 5.7 mm apart, well inside the 45 mm
  `PICK_CLEARANCE_MM` wants.
- **Lighting kept deliberately dim.** A bright dome washes saturation out of every
  coloured face, and a red cube under a strong dome falls out of its own hue band
  while still looking obviously red to a human.
- **Scene camera**, 1280×720, at the lens position the rig measured.

### Two physics settings that do not do what they look like

**Friction has to be *bound*, not applied.** `UsdPhysics.MaterialAPI` belongs on
a `UsdShade.Material` prim that colliders reference with a `physics`-purpose
material binding. Applied straight to the collider it writes attributes nothing
reads — a cube carrying `physics:staticFriction = 1.1` that way slides down a
45° ramp exactly as far as a cube with no material at all, which is how a whole
scene can look grippy in USD and be frictionless in the solver.
`arm.define_physics_material` / `bind_physics_material` are the pair that works,
and `check_grip.py` verifies by following the binding rather than reading back
the attribute it just wrote.

**`PhysxSceneAPI.timeStepsPerSecond` does not set the step rate.**
`isaacsim.core.api.World` takes its own `physics_dt` and ignores what the stage
says, so the scene silently ran at whatever `World` defaulted to.
`scene.physics_dt()` is the single source every entry point passes in, and
`scene.PHYSICS_HZ` is the one place to change it.

### Why the gripper can shove a cube square

Three numbers decide whether closing jaws push a misaligned cube into line or
just jam on its corners, and none of them is the grip force:

- **Cube-on-desk friction.** The pair μ is what the cube has to overcome to
  slide. At 0.5 and above it is welded to the table and the jaws stop dead on
  its corners at a 27 mm gap; at 0.35 and below it slides and turns. Plastic on
  wood is 0.2–0.4, so the physical value and the working value agree — the old
  1.1 was rubber-on-rubber, chosen when the grip was weak and needed the help.
- **Jaw armature** (`chain.FINGER_ARMATURE_KG`). A drive clipped at `maxForce`
  has no damping left — the `c·v` term is clipped with it — so it becomes a
  constant-force actuator that bounces off contact at `F·dt/m` per step. On a
  20 g blade at 60 Hz that is 1.25 m/s, and the jaws chatter instead of pushing.
  Armature is the servo's reflected rotor inertia, which is real and dominates
  the blade's own mass; 0.30 kg makes the contact quiet enough to do work.
- **Jaw speed** (`chain.FINGER_MAX_SPEED_M_S`) *used* to matter, and no longer
  does. With the short jaws the only thing that could turn a misaligned cube was
  the *momentum* of a fast blade, and dropping the ceiling to 0.12 m/s stopped
  the squaring entirely. Full-length tongs turn it with a static couple instead:
  20.0° of 20 at 0.5 m/s, 19.9° at 0.1.

Measured on a cube 20° off square: it turns 20.0° of 20 and ends face-gripped at
19.77 mm, 0.63 mm off the TCP.

### The armature moved the damping, and nobody moved it back

*This is the derivation for the drive that came before the current one —
`k = 1000 N/m`, `c = 15`, a 1.5 N cap that the jaws sat on whenever they gripped.
The measurements are sound and the reasoning about armature still holds; what
changed is the drive it was reasoning about. The current one is `k = 150`,
`c = 4`, cap 6.0 N, and it is kept off its cap deliberately — see the jaw
coupling section below for why. What carried over and what did not is at the end
of this section.*

Adding that armature quietly invalidated the number next to it. The jaw drive's
damping was sized as `2·√(k·m) = 2·√(1000 × 0.02) = 8.94`, so `c = 12` was
recorded as ζ = 1.34 — correct for a bare 20 g blade. But armature *is* the
inertia the drive has to accelerate; that is the whole reason it is there. The
effective mass is 0.32 kg, critical damping is 35.8, and `c = 12` was really
**ζ = 0.34**: an underdamped drive, documented as an overdamped one.

It shows up as the jaws cycling. The coupling tendon is roughly 100× stronger
than the drives — measured through a stacking run its spring reaches 164 N and
its damper 58 N, against a 1.5 N force cap — and an arm slew keeps handing the
pair differential velocity. A drive at ζ = 0.34 rings on that instead of
absorbing it, the swing grows until a blade reaches the end of its travel, and
that one-sided stop rectifies it into **both jaws walking shut together**. With
the host holding the gripper open the whole time, the gap collapses from the
commanded 38.4 mm to 4.8 mm and back, at the drives' full 0.15 m/s.

Replaying one stacking run's own command stream — same commands every time, no
vision — the threshold is sharp, and it is about the velocity cap:

| drive `c` | ζ | worst unforced gap error | peak jaw speed |
|---|---|---|---|
| **12 (was)** | 0.34 | **33.60 mm** | 0.1505 m/s — pinned at the cap |
| 14 | 0.39 | 12.38 | 0.1500 — pinned |
| **15 (is)** | 0.42 | **4.25** | 0.1096 |
| 24 | 0.67 | 3.75 | 0.0953 |
| 48 | 1.34 | 3.75 | 0.0877 |
| 80 | 2.24 | 7.66 | 0.0830 |

**ζ > 1 is not reachable, and that is a real tension rather than a preference.**
The other end is a speed floor: a jaw saturating the force cap tops out at
`F/c`, and if that is slower than the firmware advances S (~0.096 m/s per jaw at
360 S/s) the jaws lag their own command through a free-space close.
`tests/test_chain.py` asserts exactly that, and it is what rejected `c = 48`.
The floor bounds `c` below 15.6 while ζ = 1 wants 35.8. Buying the rest would
mean raising the grip force or dropping the armature, so 15 is the most the
floor allows — and it is enough, because the qualitative change is the jaws
coming off the velocity cap.

Paired three-level `stack_cubes.py` runs, same scene, only this constant
differing:

| | worst unforced excursion | gap minimum | jaw speed *in* those episodes | picks |
|---|---|---|---|---|
| `c = 12` | 35.91 / 32.90 / 32.96 mm | 4.8 mm — nearly shut | 0.1505 m/s | 0 missed, 3/3 built |
| `c = 15` | **5.83 / 5.84 / 6.01 mm** | 34.2 mm | 0.036 m/s | 0 missed, 3/3 built |

Over a whole run, including the commanded sweeps, `c = 12` spends 15 steps
pinned at the 0.15 m/s velocity ceiling and `c = 15` spends none.

**What carried over.** The armature argument — that the inertia the drive
accelerates is `FINGER_ARMATURE_KG`, not the blade's 20 g — is the durable part,
and it is still the reason ζ is low: at `k = 150` critical damping is 13.9, so
`c = 4` is ζ = 0.29. The jaws-cycling failure is still this knob and not the
coupling.

**What did not.** The speed floor is gone. `F/c` was the terminal speed of a jaw
*saturating* its cap, and the current drive never saturates; its closing speed is
`k·x/c`, 0.25 m/s at the open stop against the 0.096 m/s the firmware sweeps at.
What replaced it as the bound on `c` is command tracking — a position drive
following a ramp lags by `c·rate/k`, which is 2.6 mm per jaw here — and that is
what `tests/test_chain.py` asserts now, in place of the `F/c` floor. The same
test used to assert the drive was at least critically damped; that was only ever
true against the bare blade, and this section is the reason why, so asserting it
was a false claim passing. It now asserts the tracking bound and a ζ floor.

### The jaws are two blades that should be one mechanism

The real gripper is a scissor: one servo drives both blades through a symmetric
linkage, so the blades cannot move independently and **their midpoint is welded
to J4**. A cube the jaws close on is pushed to the centre rather than walked
across the gripper by whichever blade reaches it first.

The model does not have that mechanism. It has two independent prismatic joints
sharing an origin on `gripper_base` — which *is* the J4 frame — with axes
`(0,+1,0)` and `(0,−1,0)`. So the pair carries two coordinates and only one of
them is held:

    gap = left + right       the drives hold this
    mid = (left - right)/2   nothing holds this

`mid` is a degree of freedom the hardware does not have, and it moves: up to
12.8 mm, both tongs together, the gap unchanged, away from the base on 8 of 9
place-downs in a run. `scripts/check_invariants.py` asserts against it, and
`docs/gripper-fires-open.md` is the investigation — the gripper firing fully open
while commanded shut, and cubes dropped in mid-transit, are both this coordinate
reaching the end of its travel.

**What holds it is the jaw drives, and only while they stay off their force
cap.** Two position drives to a common target restore the midpoint with
`-2k*mid`. Clipped to the same cap in opposite directions they sum to nothing —
measured at 0.001 N against 1.5 N on each side — so a saturating grip leaves the
pair a free 0.6 kg mass with no spring and no damper, keeping whatever sideways
velocity a move onset hands it. That is why the drive is now a soft spring kept
clear of its cap rather than the constant-force closer it used to be; see the jaw
drive section above, and note that the fix costs command tracking.

**A solved constraint would be strictly better. This runtime has none.** Three
were measured, not assumed:

- **`PhysxMimicJointAPI` is applied and then ignored.** It is the obvious tool
  and it is present in the schema. Told left = 0 and right = 24.5, the jaws go to
  exactly 0.00 / 24.50 with the mimic applied — identical to no constraint at
  all, for the `transY` and `linear` axis tokens alike.
- **`PhysxPhysicsRackAndPinionJoint` is exact on free rigid bodies and inert on
  articulation DOFs.** Ramp one rack and an undriven one mirrors it to 0.000 mm.
  On the finger joints it *looks* like it works at ratio ≥ 1e4 — and that is not
  the constraint, it is the pinion's inertia reflected to the rack as
  `I * ratio²`, which at ratio 1e4 lands 2.7 kg on a 0.30 kg armature. The jaws
  are not held together, they are too heavy to move apart; at ratio 1e5 the
  reflected 267 kg stops them closing on a cube at all. Hold the ratio and drop
  the inertia to 1e-10 and the coupling vanishes completely. Sweeping the ratio
  hides this; sweeping the inertia at a fixed ratio shows it.
- **A fixed tendon is a force the step integrates, not a constraint it solves.**
  It settles where it balances the jaw drives rather than holding
  `q_left = q_right` outright.

The tendon is still authored — by `arm.couple_finger_joints`, at scene-build
time, because PhysX reads tendons when it creates the articulation — and its
stiffness and damping are **zero**. It is kept because it is the right *model*
and the wrong *mechanism*, and because the shape of the authoring is what a
working constraint would need. Two things about it are easy to get wrong and were
measured:

- **A fixed tendon spans the *subtree* of the joint it is rooted on.** The two
  finger joints are siblings, so a tendon rooted on one cannot reach the other:
  that arrangement drags the rooted jaw to the rest length and leaves the other
  exactly where it was told. It has to be rooted on a common ancestor —
  `j4_wrist_roll` — carrying gearing 0 so the wrist stays out of the sum.
- **There is no stiffness at which it helps.** Under the drive's force budget it
  cannot resist a 1.5 N contact push; over it, a one-sided stop absorbs the
  reaction on one blade and the force reappears on the other as *gap*, which is
  the grip. Measured with damping off, on a recorded place-down:

| tendon `k` | midpoint range | grip force | |
|---|---|---|---|
| 0 | 12.8 mm | 1.50 N | the defect the drive now fixes |
| 20 | 12.5 mm | 1.50 N | inert |
| 65 | 12.5 mm | 1.50 N | inert — the top of the force budget |
| 200 | 14.7 mm | p95 2.34 N | worse than nothing |
| 600 | 9.9 mm | max 9.83 N | rectifying into the gap |

Damping is worse still at every stiffness, because it drags on the motion that
closes the gap and not only on the ringing: at k = 3e5, c = 100 leaves 0.24 mm of
residual separation and c = 5000 leaves 4.02 mm. A stiff tendon with heavy
damping bolted on to keep it stable is the worst of both, and that pairing —
3e4/400 — is what fired the gripper open and dropped cubes in transit.

**The history here is worth keeping, because it argued the opposite.** While the
tendon was live at 3e4/400 the pair visibly rocked, and that give was measured to
be load-bearing — repeated three-level `stack_cubes.py` runs, varying only the
tendon damping:

| tendon `c` | missed picks per run | stacks built |
|---|---|---|
| 400 | 0, 0, 0, 0, 0, 0 | 6 of 6 |
| 80 | 1, 1, 0 | 3 of 3 |
| 20 | 1, 2, 2 | 3 of 3 |
| 0 | 2, 9, 9, 9 | 3 of 4 |

— and stiffening did the same thing: at k = 1e6 a run missed nine and walked a
green cube from (91, 262) to (115, 151) across the desk. The reading at the time
was that the give is the gripper accommodating vision error, since a real pick
aims at a *detected* position 5.4 mm off on average and 11 mm at worst, and a
rigid pair meeting an off-centre cube squeezes it out sideways.

That reading was half right. The give does accommodate vision error. What it also
did was let the pair walk out from under the wrist, and the missed picks at
`c = 0` were the *uncoupled* pair walking rather than a too-rigid one ejecting
cubes. With the drive held off its force cap the pair stays centred without a
tendon at all, and the tendon-off runs build eight-high columns with zero missed
picks — which no tendon setting ever managed.

**The jaws cycling is not this knob.** Softening the coupling does quiet them,
which is what makes it such an attractive wrong answer. The cause is the drive's
damping ratio, one section up.

Software coupling is not a substitute: projecting the pair back onto L = R would
drive the near jaw straight into the cube it is already touching.

`check_grip.py` passes 5 of 5 on the current drive, with the tendon inert:

| case | cube ends, from the TCP | jaw asymmetry | grip | was (k=1000, cap 1.5 N) |
|---|---|---|---|---|
| square on | 0.09 mm | 0.31 mm | 1.49 N | 0.07 mm / 0.22 mm |
| +20° mis-aimed | 0.50 mm | 0.03 mm | 1.47 N | 0.40 mm / 0.08 mm |
| −20° mis-aimed | 2.06 mm | 0.17 mm | 1.48 N | 2.54 mm / 0.19 mm |
| cube already at 25° | 0.08 mm | 0.21 mm | 1.48 N | 0.10 mm / 0.14 mm |
| cube already at 30° | 0.07 mm | 0.26 mm | 1.49 N | 0.15 mm / 0.18 mm |

The mis-aimed cases still turn the cube square — 0 → 19.8° and 0 → −19.5° — so
the softer spring has not cost the jaws the couple that does that.

Note what this set *cannot* tell you. Every cube here starts at its true
position, perfectly centred between the blades, and on that the coupling can be
made arbitrarily rigid and the numbers only improve — which is exactly how a
setting that cost nine picks a run passed with 0.04 mm of asymmetry. The
asymmetry column is a floor, not a verdict, and a stacking run is what shows the
difference.

So here is the stacking run. Three recorded eight-level trials on marker 0 per
drive, same scene and same site, scored by `scripts/check_invariants.py` — the
violation columns are raw counts over a whole run:

| drive | levels built | missed picks | worst off-axis | midpoint | jaw pinned on a stop | drops |
|---|---|---|---|---|---|---|
| k=1000, cap 1.5 N | 8, 8, 8 | 0, 0, 0 | 3.4, 2.4, 1.7 mm | 2162, 2084, 2123 | 7, 6, 8 | 0, 0, 0 |
| **k=150, cap 6.0 N** | 8, 8, 8 | 0, 0, 0 | 2.7, 2.0, 1.7 mm | **12, 13, 12** | **0, 0, 0** | 0, 0, 0 |

The midpoint column is the change, and the rest of the row is the point: the
soft spring's cost is command tracking, ~0.1 s of lag on a free-space close, and
it buys a 170x reduction in midpoint wander without costing a pick, a level, or
a millimetre of placement. That lag is real and a bench grip cannot see it —
this table is what says it does not matter in practice.


### The camera *is* the rig's camera

The lens sits exactly where `calibrate_camera_nadir.py` put it: nadir (505, 1),
242 mm above the table. Everything else about the camera is recorded nowhere —
the rig's calibration is a homography fit straight from tag pixels to
millimetres and deliberately needs no intrinsics — so `mt4_sim.calibration` fits
it back out of the homography itself, with the lens pinned. For a pinhole, a
plane homography *is* the orientation and the intrinsics.

**All six of the remaining numbers have to be fitted**, not just three. A camera
pinned in space still has three angles, a focal length and a principal point,
and the rig's map needs every one:

| model | reproduces the rig's pixel↔table map to |
|---|---|
| aim + focal length (level, principal point centred) | 21.2 px — 13.9 mm |
| + roll | 17.4 px |
| **+ principal point** | **5.4 px — 4.2 mm** |

The principal point is the term that carries it, and it is the same fact as the
rig's work area sitting low in its own frame: the fit puts the optical axis
nearly horizontal, aimed past the far side of the desk, with the axis crossing
the sensor at (699, 140) of 1280×720 and an **81.8°** horizontal field.
`check.py` prints the fit.

Those 13.9 mm were not a rendering nicety. They landed on **every cube position
the live stack read out of a simulated frame**, which is what `stack_cubes.py`
sends the gripper to. On the default nine-cube layout:

| | cube read-back | tags, through the rig's own calibration | `stack_cubes --marker 2` |
|---|---|---|---|
| aim + focal only | 13.9 mm mean, 21.7 worst | 6.7–21.8 mm | **12 of 20 picks missed** |
| full camera | **5.6 mm mean, 10.8 worst** | **1.6–7.1 mm** | **0 missed** |

A miss is not a retry: the jaws close beside the cube and shove it. One green
cube was walked 105 mm across the desk over seven attempts before the run gave
up on it.

**Pixels stay square, and that is a constraint rather than a simplification.**
`isaacsim.sensors.camera.Camera` rewrites `verticalAperture` to match the
resolution's aspect ratio whenever the two disagree — it warns, and carries on —
so a second focal length fitted here is dropped on the way to the renderer, and
`SceneCamera` quietly stops describing the camera that took the picture. Fitting
one buys 0.7 px on paper and nothing at all in the frame.

What is left is a real disagreement rather than a slack fit. Let the lens float
as well and the map is reproduced **exactly**, at a lens 60 mm from the measured
one — so the rig's homography, which is a least-squares fit over barrel
distortion no projective map can carry, is not quite the map of *any* pinhole
standing where the rig says the lens stands. The sim keeps the measured position,
because it is a measurement and because the parallax of anything with height
hangs off it, and reports the residual.

### Tag textures are content-addressed, and have to be

Kit caches textures by path and does not notice the file changing underneath it.
Change which tags the rig carries, and the renderer will draw the *previous* tag
from a path whose contents have since been rewritten — a scene that is correct in
USD, correct on disk, and wrong in the frame. `markers.write_tag_textures` puts a
hash of the image in the filename so a path can only ever hold one image, and
deletes tags left over from an earlier layout.

## Layout

| Path | What |
|---|---|
| `mt4_sim/chain.py` | model angles ↔ URDF joints, limits, gripper span / stall, `DESK_Z_MM` |
| `mt4_sim/urdf.py` | the URDF: link shapes, masses, joint table |
| `mt4_sim/calibration.py` | reads `vision_calibration.json` as scene geometry: table, tags, camera |
| `mt4_sim/rig.py` | desk extent, colours, cubes — the layout the calibration has no opinion on |
| `mt4_sim/scene.py` | builds the stage |
| `mt4_sim/arm.py` | `SimArm`: drive by model angles, force-capped jaws, read TCP, solve the repo's IK |
| `mt4_sim/camera_feed.py` | shared-memory publisher/reader for `/World/SceneCamera` frames |
| `mt4_sim/sim_capture.py` | duck-typed `VideoCapture` / `FrameStream` over that feed |
| `mt4_sim/markers.py` | renders the ArUco tag textures |
| `mt4_sim/mt4_repo.py` | finds the control repo and puts it on `sys.path` |
| `mt4_sim/firmware/` | the firmware's serial personality: `state`, `planner`, `machine`, `link` |
| `scripts/` | `build_urdf` → `import_urdf` → `build_scene` → `check` / `run_sim` |
| `scripts/serve_firmware.py` | the scene, answering the MT4 protocol on a socket (`--camera` publishes frames) |
| `scripts/run_against_sim.py` | runs any control-repo script against that socket (and camera feed) |
| `tests/test_chain.py` | chain maths against `mt4_jog.kinematics`, no GPU |
| `tests/test_firmware.py` | the protocol, checked with the control repo's own client |
| `tests/test_camera_feed.py` | shared-memory camera feed round-trip, no GPU |
| `tools/step_assembly.py` | STEP assembly parser and bore-based kinematic audit |
| `vendor/MT4-STL/` | WLKATA's official STL + STEP CAD (upstream clone) |
| `assets/`, `out/` | generated USD and renders |

## The firmware, replaced

Everything in the control repo above the serial port — `mt4_vision`'s pick/place
primitives, the task scripts, the MCP tools — talks to one thing: an
`Mt4Client` that opens a COM port, writes `mp 230 -60 122 h 0`, and waits for
`mp done pos ...`. `mt4_sim/firmware/` puts something on the other end of that
port that answers the way the firmware answers, so all of it drives the
simulation with **no change to any of it**.

```powershell
& $env:PY scripts/serve_firmware.py --camera   # the arm + scene camera
& $env:PY scripts/run_against_sim.py --check   # what the client sees
& $env:PY scripts/run_against_sim.py --check-camera
& $env:PY scripts/run_against_sim.py -- Z:\MT4\jog.py   # any control-repo script
& $env:PY scripts/demo_pick_place.py           # pickplace.pick/place, on a cube
& $env:PY scripts/check_firmware.py            # does a real pick move a real cube?
```

`serve_firmware.py` steps the scene, paced to the wall clock — the host is
timing us, so a move that takes 4 s on the bench has to take 4 s here or every
timeout in `Mt4Client` means something different. With `--camera` it also
publishes `/World/SceneCamera` BGR frames on a shared-memory feed
(`MT4_CAMERA_URL=shm://mt4_scene_cam`); `run_against_sim.py` sets that env var
so `mt4_vision.camera` opens the feed itself.

### The counters are the truth, here as there

The real firmware is **open loop**: `pos` is a set of step counters it has been
incrementing since the last home, and there is no encoder to disagree. So this
keeps the counters as the authority too and drives the simulated arm to follow
them — the same relationship the real steppers have to their pulse train. It
also means the two commands that renumber without moving (`setpos`, `j4zero`)
are free on hardware and are not here, so a per-joint bias records what the
counters claim versus where the arm was actually left.

| Faithful | How |
|---|---|
| Timing | One step period per master-axis step, so a leg takes as long as it does on the bench; `speed <us>` changes it the same way; gripper S advances at 360 S/s (finger targets) while grip stations still hold for the 180 S/s duration, so closes finish early and settle before the arm lifts |
| Grip force | The jaws are a constant-force closer: a soft 1000 N/m spring commanded shut, capped at **1.5 N**, so a close past contact stalls at the cap the way the real servo does. That is 19× an 8 g cube's weight — enough to shove a misaligned cube square against the desk — and small enough not to launch it |
| Path shape | `mp`/`mq` chop a straight world line into 2 mm segments and solve each with the control repo's own `ik_position`, routing tangent-arc-tangent around the 140 mm keep-out cylinder |
| Rejections | `err not homed`, `err mp keepout`, `err mp ground z<115.0`, `err mp joints`, `err mq full 8`, `err mq station pose want … at …` — the exact strings the host greps for |
| Queue semantics | `mq` cold-starts when idle and queues when not, a drained queue emits one `mp done`, `mp` mid-flight overrides and drops the queue, a grip station holds everything until the jaws finish |

### Where it is a stand-in and says so

**Homing does not seek.** There are no limit switches on the stage, so `home`
drives to the pose homing ends at over `--home-seconds` rather than the real
seek's tens of seconds of hunting.

**No acceleration ramp.** The firmware ramps a move in and out over ~60 ticks
either side; this runs the whole leg at the step period, so a short leg finishes
a few tens of milliseconds early.

**Floating the drivers does not make the arm fall.** `e0` / `all f` set the flag
`?` reports and stop nothing else, where the real arm goes limp and loses its
counters. Worth knowing because `jog.py` sends `all f` on startup.

**"Done" waits for the drives.** A stepper is wherever its last pulse put it; a
position drive is still a fraction of a degree behind. Reporting `mp done` the
instant the counters arrive left the arm 3.3 mm out and still moving — which a
host that captures a camera frame on that line would photograph. So a completed
path settles to 0.02° before the line goes out, which is what makes the word
mean the same thing at both ends.

### Pointing a script at it

`mt4_jog.serial.open_serial` opens a COM port by name; nothing in the control
repo asks for a URL. `run_against_sim.py` replaces that one function with one
that dials the simulator's socket and then runs the target script exactly as
`python` would — so the script, and `Mt4Client`, never know. If you would rather
patch nothing at all, serve on one half of a virtual null-modem pair
(`--listen COM21`) and point the script at the other half the normal way.

### What it is checked against

`tests/test_firmware.py` is 25 tests that never construct an expected string by
hand: replies are parsed with the control repo's own `mt4_jog.status`, and two
of them drive the machine with a real `Mt4Client` over a real socket, including
a queued pick-and-place path with a firmware grip station.
`scripts/check_firmware.py` goes further and asks the world instead of the
protocol — it runs `mt4_vision.pickplace.pick`/`place` unmodified and then looks
at where the cube ended up on the stage. The force-capped finger drives keep a
close past contact (live calib uses S=255) from crushing the cube the way a
stiff position target to zero opening would.

`scripts/check_grip.py` is the narrower test of the same thing, and it asserts
against both failure directions at once: the jaws must end up symmetric and on
the cube's faces, the cube must never exceed 0.15 m/s or wander 6 mm while they
close, and it must ride the lift. Run it with `--yaw-error` to make the jaws
earn their force by rotating a misaligned cube into line.

## The camera, replaced

`mt4_vision.camera` opens a USB index through OpenCV, or a sim feed when
`MT4_CAMERA_URL=shm://…` is set. `serve_firmware.py --camera` renders
`/World/SceneCamera` and publishes BGR frames on that URL; `run_against_sim.py`
sets the env var so `capture_scene`, the task scripts and the MCP tools see the
simulated desk without monkey-patching. `scripts/check.py` already proved those
frames decode the live ArUco set and read back through `vision_calibration.json`
to within ~22 mm.

```powershell
& $env:PY scripts/serve_firmware.py --camera
& $env:PY scripts/run_against_sim.py --check-camera
& $env:PY scripts/run_against_sim.py -- Z:\MT4\stack_cubes.py --marker 2
```

Marker **4** sits under the camera-park pose and is refused; use 0–3. The scene
stocks nine cubes, three each of red, green and blue, which is what a full
nine-level `stack_cubes` run needs.

## What this does not do

**No envelope guard below the firmware layer.** `SimArm.set_model_angles` checks
the soft joint limits and nothing else. The ground-Z floor and keep-out cylinder
are enforced by `mt4_sim/firmware` on the `mp`/`mq` path, as on the real arm, but
a caller poking `SimArm` directly bypasses them; the desk is a collider, so the
arm stops on it rather than being refused.

**Nominal masses.** The arm is position-driven, so link masses set how hard the
solver works, not where the TCP ends up. They are estimates, not measurements.
