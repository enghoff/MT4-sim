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

`scripts/check.py` runs four checks against the live stack's own code, and all
four pass:

| Check | Result |
|---|---|
| The URDF chain reproduces `mt4_jog.kinematics.fk_tcp` | **0.0002 mm** unexplained across the soft-limit box |
| Position drives settle at the commanded pose | **≤ 0.04°** per joint |
| The simulated camera's ArUco tags decode | **6 of 6**, via `cv2.aruco` DICT_4X4_50 |
| Cubes clear `mt4_vision.detect`'s own HSV thresholds | **4 of 4**, 2157–4066 px² blobs |

That last pair matters more than it looks: the simulated scene camera's frames
go straight into the real `mt4_vision.detect` with nothing adapted. The cube
blob areas land inside the 1600–3600 px² the repo measured on the live rig.

## Quick start

```powershell
$env:PY = "Z:\IsaacSim\venv\Scripts\python.exe"

& $env:PY scripts/build_urdf.py     # kinematics -> assets/mt4.urdf
& $env:PY scripts/import_urdf.py    # -> assets/mt4/mt4.usda   (~3 min first run)
& $env:PY scripts/build_scene.py    # -> assets/mt4_scene.usda
& $env:PY scripts/check.py          # verify + render to out/

& $env:PY scripts/run_sim.py            # open the GUI, arm parks and holds
& $env:PY scripts/run_sim.py --demo     # tour the desk: hover every tag and cube
& $env:PY -m unittest discover -s tests # chain maths, no GPU needed
```

The control repo must be reachable. It defaults to the sibling `Z:\MT4`;
override with `MT4_REPO`.

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

## Frames, and the base height

The stage is authored in the arm's **home-angle frame**, the frame every
coordinate in the MT4 stack lives in. Two consequences, one of them a question
for whoever knows the rig:

- **The desk surface is at z = 120 mm.** That is where the live rig measured
  desk contact (2026-08-04, camera-tracked descent), and `Calibration.table_z`
  calls it both the table surface's robot-frame Z and the TCP Z that grips a cube
  sitting on it.
- **So the shoulder pivot is 20 mm above the desk**, because `CENCER_HEIGHT` puts
  it at z = 140. The real MT4's base column is 130 mm tall, so this cannot be
  physically true — the model's z origin is offset from the arm's mounting plane
  by something the calibration silently absorbs.

The sim keeps the model frame, because that is what makes coordinates
interchangeable with the live rig, and renders the base as the 20 mm plinth the
frame implies. **If the true offset is known, the base and gripper can be drawn
at their real heights with a fixed visual offset and no change to the
kinematics.** Until then the arm looks squat around the base and the gripper is
compressed: `HEAD_HEIGHT` = 14.43 mm puts the fingertips just under the wrist,
where the real gripper hangs a good deal lower.

`mt4_sim.chain.DESK_Z_MM` is the single place this is set.

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

Authored by `mt4_sim/scene.py` from the layout table in `mt4_sim/rig.py`.

- **Desk** — wood-toned, top face at z = 120. The tone is not decoration: the
  live `mt4_vision.detect` has no "orange" cube colour precisely because the wood
  table and red cubes' shaded faces share that hue band.
- **Wall** behind the arm, because `calibrate_table_edge.py` needs it visible.
- **Six ArUco tags**, real DICT_4X4_50 codes rendered at 768 px with a quiet
  zone, 50 mm squares, laid out across the reachable annulus.
- **Four cubes**, 20 mm, in the four colours the HSV detector knows, with grippy
  friction because a grasp on this arm holds by friction alone.
- **Lighting kept deliberately dim.** A bright dome washes saturation out of every
  coloured face, and a red cube under a strong dome falls out of its own hue band
  while still looking obviously red to a human.
- **Scene camera**, 1280×720, mounted obliquely off the far +X side and aimed back
  across the desk.

### The camera is not the rig's camera

The live mount was measured at nadir (518, −35), lens 244 mm above the table.
Reproducing that literally needs about a 120° field to cover the work area, which
is a fisheye. The real rig's intrinsics are recorded nowhere — its calibration is
a homography fit straight from tag pixels to robot millimetres and deliberately
needs none — so the sim keeps the *character* of the mount (off-desk nadir on the
+X side, steeply oblique) at 420 mm, where a 50° lens covers everything.
`check.py` prints both geometries side by side. `rig.CAM_POSITION_MM` /
`CAM_TARGET_MM` are the knobs.

## Layout

| Path | What |
|---|---|
| `mt4_sim/chain.py` | model angles ↔ URDF joints, limits, gripper span, `DESK_Z_MM` |
| `mt4_sim/urdf.py` | the URDF: link shapes, masses, joint table |
| `mt4_sim/rig.py` | desk, tags, cubes, camera — the scene's layout numbers |
| `mt4_sim/scene.py` | builds the stage |
| `mt4_sim/arm.py` | `SimArm`: drive by model angles, read TCP, solve the repo's IK |
| `mt4_sim/markers.py` | renders the ArUco tag textures |
| `mt4_sim/mt4_repo.py` | finds the control repo and puts it on `sys.path` |
| `scripts/` | `build_urdf` → `import_urdf` → `build_scene` → `check` / `run_sim` |
| `tests/test_chain.py` | chain maths against `mt4_jog.kinematics`, no GPU |
| `tools/step_assembly.py` | STEP assembly parser and bore-based kinematic audit |
| `vendor/MT4-STL/` | WLKATA's official STL + STEP CAD (upstream clone) |
| `assets/`, `out/` | generated USD and renders |

## What this does not do

**No motion planning.** The firmware's `mp`/`mq` interpolate straight
world-frame lines, route around the J1 keep-out cylinder and validate every
segment. `SimArm` drives joints to a target and lets physics get there. Reach
failures come back the same way the real client's do — `move_to_tcp` returns
`None` off the control repo's own `ik_position` — but nothing here refuses a path
that would clip the keep-out.

**No envelope guard.** `set_model_angles` checks the firmware's soft joint
limits and nothing else. The ground-Z floor and keep-out cylinder that gate all
four real control paths are not reimplemented; the desk is a collider, so the
arm stops on it rather than being refused.

**No entity layer.** `mt4_vision.entities`, the pick/place primitives and the
MCP tools all talk to a serial `Mt4Client`. Nothing here pretends to be one, so
the task scripts do not drive the sim yet. A shim that answers `pos`/`mp`/`g` on
a socket is the natural bridge, and would make `stack_cubes.py` run against this
scene unmodified.

**Nominal masses.** The arm is position-driven, so link masses set how hard the
solver works, not where the TCP ends up. They are estimates, not measurements.
