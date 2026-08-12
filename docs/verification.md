# What is verified

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

## The blob areas are the one number outside what the rig measured

The spread is the point: pushed through the rig's own cube-top map, a 20 mm
cube's top face covers **281 px² at the far side of the work region and 2573 px²
at the near side** — a factor of 9 — because the camera is 242 mm up and steeply
oblique. `mt4_vision` gates cube blobs on **fixed** areas (`MIN_BLOB_AREA` 800,
`MAX_BLOB_AREA` 6000, `PICK_MIN_AREA` 400, `PICK_MAX_AREA` 5000) measured on
cubes sitting on the markers, where the top face is 417–1361 px². Cubes further
out image past the cap and are dropped as phantoms — not mislocated, *gone*.

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

## Beyond `check.py`

- `tests/` — chain maths against `mt4_jog.kinematics`, the serial protocol
  checked with the control repo's own client, and the camera feed's
  shared-memory round trip. No GPU needed.
- `scripts/check_firmware.py` — runs `mt4_vision.pickplace.pick`/`place`
  unmodified and then looks at where the cube ended up on the stage. See
  [firmware.md](firmware.md).
- `scripts/check_grip.py` — the narrow grip test, including `--yaw-error` to
  make the jaws rotate a misaligned cube into line. See [gripper.md](gripper.md).
- `scripts/check_invariants.py` — assertions about the jaws that must hold in
  *any* run, written to be independent of whatever hypothesis is being tested.
  See [gripper-fires-open.md](gripper-fires-open.md) for why it exists.
