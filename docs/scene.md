# The scene

Authored by `mt4_sim/scene.py`. Most of the layout is not written down in this
repo at all: `mt4_sim/calibration.py` reads the live rig's
`vision_calibration.json` and hands back the table height, the tags and the
camera, the same way `mt4_sim/chain.py` reads the firmware's kinematics. A
recalibration on the real rig is one `build_scene.py` away from being true here.

- **Work surface** — wood-toned, top face at z = 0 (the plane the arm's own base
  stands on; `Calibration.table_z` = 122 is a TCP height, see [frames.md](frames.md)),
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
  friction — see [gripper.md](gripper.md), it is the number the gripper is most
  sensitive to. Positions come from `tools/spread_cubes.py` rather than by hand:
  it maps everywhere a cube is *permitted* to sit and takes the arrangement with
  the largest smallest gap, which puts the closest pair 99 mm apart where the
  hand-placed set managed 51 mm. Six things bound that region and every one of
  them binds somewhere — the firmware's 140 mm keep-out cylinder, reach at the
  +70 mm transit height as well as at the table, a 300 mm ceiling well inside the
  359 mm the IK will solve, the hull the calibration was actually *fit* over
  (outside it the pixel↔table map is extrapolating), the tag cards, which are
  59 mm across including the quiet zone rather than the 44.3 mm of printed black,
  and the **blob-area gates** in [verification.md](verification.md). Two more are
  available behind `--site`, both keyed to where the stack goes: the site's
  keep-clear radius, so the run never shoves a cube aside to start, and the
  standing column's forearm shadow. They are off by default — each is real, but
  together they cost 37 mm of the closest pair for one nominated marker, and
  enforcing them for all four candidates at once leaves 18 of 12800 cells and
  cubes 5.7 mm apart, well inside the 45 mm `PICK_CLEARANCE_MM` wants.
- **Lighting kept deliberately dim.** A bright dome washes saturation out of every
  coloured face, and a red cube under a strong dome falls out of its own hue band
  while still looking obviously red to a human.
- **Scene camera**, 1280×720, at the lens position the rig measured — see
  [camera.md](camera.md).

## Two physics settings that do not do what they look like

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
`chain.physics_dt()` is the single source every entry point passes in, and
`chain.PHYSICS_HZ` is the one place to change it. It lives in `chain` rather
than `scene` because the jaw servo's speed ceiling is measured in wind-up per
step, and the checks that assert that bound run without the Kit runtime that
importing `scene` needs. `scene` re-exports both.

## Tag textures are content-addressed, and have to be

Kit caches textures by path and does not notice the file changing underneath it.
Change which tags the rig carries, and the renderer will draw the *previous* tag
from a path whose contents have since been rewritten — a scene that is correct in
USD, correct on disk, and wrong in the frame. `markers.write_tag_textures` puts a
hash of the image in the filename so a path can only ever hold one image, and
deletes tags left over from an earlier layout.

## Related

- [frames.md](frames.md) — the coordinate frame everything here is authored in.
- [camera.md](camera.md) — the scene camera, and how it is fitted to the rig's.
- [gripper.md](gripper.md) — the physics the cubes and jaws actually run under.
