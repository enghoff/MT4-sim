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
| Those tags, read back with the rig's own `vision_calibration.json` | **7–22 mm** and **≤ 3.6°** from where that file says they are taped |
| Cubes clear `mt4_vision.detect`'s own HSV thresholds | **4 of 4**, 2129–3326 px² blobs |

The last three matter more than they look: the simulated scene camera's frames
go straight into the real `mt4_vision.detect` and the real `Calibration` with
nothing adapted. The cube blob areas land inside the 1600–3600 px² the repo
measured on the live rig, and a tag detected in a simulated frame, pushed
through the live homography, comes out where the live rig says that tag is —
which is the strongest available statement that the two scenes are the same
scene.

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

& $env:PY scripts/serve_firmware.py     # be the arm's firmware, on a socket
& $env:PY scripts/check_firmware.py     # a control-repo pick, on a simulated cube
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

At the drive stiffness used (12000 N·m/rad) the tilt is under 0.1° and the TCP
artefact under 0.06 mm. For scale, the real arm's measured backlash is 6–9 mm on
small reversing moves.

Modelling the rods properly needs a closed kinematic loop, which URDF cannot
express and USD can. Worth doing only if something starts caring about 0.06 mm.

## Frames: what z = 0 is, and where the desk sits

The stage is authored in the arm's **home-angle frame**, the frame every
coordinate in the MT4 stack lives in.

**z = 0 is a modelling origin, not a surface.** It is the plane exactly 140 mm
straight below the J2 shoulder pivot, on the J1 turning axis, inherited from the
factory link geometry: `fk_tcp` builds TCP height as shoulder pivot 140, plus the
two link contributions, minus 14.43 mm for the drop from the wrist pivot down to
the gripper pads. Nothing physical sits at that plane and the gripper cannot be
commanded to it — the lowest the pads reach inside the soft joint limits is
z ≈ 37 mm.

**The work surface is at z = 122 mm** — `Calibration.table_z`, read live out of
the rig's own `vision_calibration.json` rather than restated here. It is both
the table surface's robot-frame Z and the TCP Z that grips a cube sitting on it.

So the shoulder pivot clears the work surface by **18 mm**, and everything
below the shoulder is *under* the wood. That is not a modelling choice; it is
what the rig's own numbers say, three times over:

- **The arm cannot reach a surface at its own foot.** The lowest the pads go
  anywhere inside the soft joint limits is z ≈ 37, and out in the annulus where
  the tags are it is z ≈ 100. A desk at z = 0 would be unreachable everywhere,
  and no cube on it could ever be picked.
- **`GROUND_Z_MM` = 115 exists because the limits let the arm go *below* the
  surface.** A firmware floor a few mm under the table is only needed if the
  table is inside the reachable volume, not beneath it.
- **`calibrate_table_edge.py` measured the desk's back edge at x ≈ −76** — behind
  the J1 axis, running through the base's own 140 mm footprint. The desk runs
  *past* the arm; it does not stop in front of it.

The sim therefore draws **one flat surface**, top at z = 122, spanning the
measured back edge out to the frame edges, with a **bay** cut back from the rear
edge for the arm. The bay is not decoration: the rotating column sweeps a 67 mm
radius, and a tabletop through it is a static collider inside the articulation's
swept volume, which jams the base yaw solid.

### The one thing that does not add up

The CAD's `base_link` is **130 mm tall**, and the surface is 18 mm below the
shoulder pivot — so the arm's base sits below the work surface, mounted at its
back edge rather than standing on top of it. If the real rig has the MT4
standing *on* the desk, then `CENCER_HEIGHT` = 140 and `Calibration.table_z` =
122 cannot both be right, and it is `table_z` that would need re-measuring:
120–122 is suspiciously close to `GROUND_Z_MM` = 115, and `docs/CALIBRATION.md`
already warns that a touch which hit the guard clamp instead of the desk records
right about there. Everything downstream of the table height — every pick, the
whole vision map — is self-consistent either way, which is exactly why the
disagreement can sit there unnoticed.

Knobs: `chain.DESK_Z_MM` (read from the calibration), `rig.DESK_FRONT_X_MM` and
`rig.DESK_HALF_Y_MM` for how far the surface runs, `rig.DESK_BAY_*` for the bay.

### The gripper really is that compact

`HEAD_HEIGHT` = 14.43 mm is the whole drop from the wrist pivot to the pads, so
there is room for the jaws below the level plate and nothing else. The jaws are
drawn rising from the pads rather than hanging below them — a cube on the desk is
gripped with the TCP at table height, so a jaw reaching under the pads would be
driven into the desk on every pick. The servo housing sits on top of the plate,
which is where the room is.

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

- **Work surface** — wood-toned, top face at z = 122 (`Calibration.table_z`),
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
- **Four cubes**, 20 mm, in the four colours the HSV detector knows, with grippy
  friction because a grasp on this arm holds by friction alone. Placed where the
  reachable annulus, the camera's actual frame coverage and the tag positions all
  leave room — the real camera sees out to only x ≈ 270, far short of the arm's
  338 mm reach at table height.
- **Lighting kept deliberately dim.** A bright dome washes saturation out of every
  coloured face, and a red cube under a strong dome falls out of its own hue band
  while still looking obviously red to a human.
- **Scene camera**, 1280×720, at the lens position the rig measured.

### The camera *is* the rig's camera

The lens sits exactly where `calibrate_camera_nadir.py` put it: nadir (505, 1),
242 mm above the table. Where it points and how wide it sees are recorded
nowhere — the rig's calibration is a homography fit straight from tag pixels to
millimetres and deliberately needs no intrinsics — so `mt4_sim.calibration` fits
those three numbers back out of the homography itself, with the lens pinned. For
a pinhole, a plane homography *is* the aim and the focal length.

It comes out aimed at (−33, −36) with a **76° horizontal field**: nearly past
the desk, which is why the work area sits in the lower half of the frame in the
rig's own captures and now does in the sim's. That reproduces the rig's
pixel↔table map to **21 px rms (14 mm)**, and `check.py` prints it.

The residual is the real lens's barrel distortion, which no pinhole can express.
Letting the lens position float as well cuts it to 7 px — but lands 70 mm from
where the rig measured the lens, which is distortion being absorbed as a wrong
camera position. The sim keeps the measured position and reports the error.

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
| `mt4_sim/chain.py` | model angles ↔ URDF joints, limits, gripper span, `DESK_Z_MM` |
| `mt4_sim/urdf.py` | the URDF: link shapes, masses, joint table |
| `mt4_sim/calibration.py` | reads `vision_calibration.json` as scene geometry: table, tags, camera |
| `mt4_sim/rig.py` | desk extent, colours, cubes — the layout the calibration has no opinion on |
| `mt4_sim/scene.py` | builds the stage |
| `mt4_sim/arm.py` | `SimArm`: drive by model angles, read TCP, solve the repo's IK |
| `mt4_sim/markers.py` | renders the ArUco tag textures |
| `mt4_sim/mt4_repo.py` | finds the control repo and puts it on `sys.path` |
| `mt4_sim/firmware/` | the firmware's serial personality: `state`, `planner`, `machine`, `link` |
| `scripts/` | `build_urdf` → `import_urdf` → `build_scene` → `check` / `run_sim` |
| `scripts/serve_firmware.py` | the scene, answering the MT4 protocol on a socket |
| `scripts/run_against_sim.py` | runs any control-repo script against that socket |
| `tests/test_chain.py` | chain maths against `mt4_jog.kinematics`, no GPU |
| `tests/test_firmware.py` | the protocol, checked with the control repo's own client |
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
& $env:PY scripts/serve_firmware.py            # the arm, on a socket
& $env:PY scripts/run_against_sim.py --check   # what the client sees
& $env:PY scripts/run_against_sim.py -- Z:\MT4\jog.py   # any control-repo script
& $env:PY scripts/demo_pick_place.py           # pickplace.pick/place, on a cube
& $env:PY scripts/check_firmware.py            # does a real pick move a real cube?
```

`serve_firmware.py` steps the scene, paced to the wall clock — the host is
timing us, so a move that takes 4 s on the bench has to take 4 s here or every
timeout in `Mt4Client` means something different.

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
| Timing | One step period per master-axis step, so a leg takes as long as it does on the bench; `speed <us>` changes it the same way; the gripper sweeps at the firmware's 120 S/s |
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
at where the cube ended up on the stage. It lands 12 mm from the place target,
and that residual is the gripper, not the protocol: the calibration closes to
S=255, which the jaw-span model puts past zero opening, so the simulated fingers
squeeze a 20 mm cube the real servo would just stall against.

## What this does not do

**No camera.** This is the other half of the substitution and it is not built.
`mt4_vision` opens a USB camera through OpenCV, so anything that *detects* — the
task scripts, the calibration routines — still needs a real one. The scene
camera is already rendered and already in the rig's own frame
(`scripts/check.py` proves a tag decoded from it reads back through the live
calibration to within 22 mm), so what is missing is the plumbing that serves
those frames where `mt4_vision.camera` looks for them, not the frames.

**No envelope guard below the firmware layer.** `SimArm.set_model_angles` checks
the soft joint limits and nothing else. The ground-Z floor and keep-out cylinder
are enforced by `mt4_sim/firmware` on the `mp`/`mq` path, as on the real arm, but
a caller poking `SimArm` directly bypasses them; the desk is a collider, so the
arm stops on it rather than being refused.

**Nominal masses.** The arm is position-driven, so link masses set how hard the
solver works, not where the TCP ends up. They are estimates, not measurements.
