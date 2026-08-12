# The camera

Two things under one heading: how the simulated lens was fitted to the real
one, and how the live vision stack is pointed at it.

## The camera *is* the rig's camera

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

## Related

- [firmware.md](firmware.md) — the other half of the substitution, the serial port.
- [scene.md](scene.md) — the desk, tags and lighting the camera looks at.
- [verification.md](verification.md) — what the frames are checked against.
