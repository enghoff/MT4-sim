# The gripper fires fully open while commanded shut

Found 2026-08-09 in the recorded marker-0 stacking run (`out/stack_marker0.mp4`,
log `jaws_rec0.csv`). Cause established the same day: the coupling tendon buys
symmetry by opening the gap, and at `FINGER_COUPLING_STIFFNESS = 3e4` it
outmuscles the 1.5 N jaw drive by an order of magnitude, so an off-centre grip is
released in mid-air. Fixed by dropping the stiffness to 1e3; the durable part of
the reasoning lives in `mt4_sim/chain.py` beside the constant, and what follows
is the investigation that got there.

## What happens

While carrying a cube between two waypoints, ~150 mm above the desk, both jaws
travel from a 20 mm hold to their fully-open stop in 0.12 s, at the joint's
`maxJointVelocity` cap the whole way, and then slam shut again at the same cap.
The cube falls 33 mm. The gripper is commanded **fully shut** for every one of
those steps.

```
   t       S    cmd gap    left    right     gap      vl       vr
16.500   255      0.00   10.244   9.989   20.233  -0.011  +0.089   holding the cube
16.517   255      0.00    9.914  11.199   21.113  +0.038  +0.125
16.533   255      0.00   10.783  13.216   23.999  +0.129  +0.150
16.550   255      0.00   12.908  15.494   28.402  +0.151  +0.150
16.567   255      0.00   15.209  17.729   32.938  +0.151  +0.150
16.583   255      0.00   17.501  19.955   37.456  +0.151  +0.150
16.600   255      0.00   19.789  22.169   41.959  +0.151  +0.150   cube already falling
16.617   255      0.00   22.069  24.375   46.444  +0.151  +0.150
16.633   255      0.00   24.102  24.535   48.638  +0.127  -0.000   both at the open stop
16.650   255      0.00   24.535  23.185   47.720  -0.000  -0.100
16.700   255      0.00   17.910  15.671   33.581  -0.150  -0.150   slamming shut again
16.800   255      0.00    2.864   0.660    3.524  -0.150  -0.145   shut on nothing
```

`left`/`right` are the two prismatic finger joints in mm, each the half-gap;
`gap = left + right`. `cmd gap` is what `finger_positions_for_s(S)` asks for.
The joint range is 0 (blades touching) to 24.535 mm (fully open), so 48.638 mm
is both jaws hard against their upper stops.

The host's actual open command (`g 140`) does not execute until t≈17.9, more than
a second after the cube is already lying on the desk. Nothing in the command
stream asks for any of this.

## The two seconds before it

The fire-open does not come out of nowhere. The cube is grabbed 7.5 mm off the
jaw centreline — that is the vision error, tracked separately — and over the
carry the jaw pair walks sideways across it while the arm stands still:

```
   t   cube-TCP dx    dy    dist |   gap   left  right |  tcp x   tcp y  cube z
14.10       6.98   2.77   7.51 | 19.76   9.82   9.94 |   90.7  -262.0  10.00  gripped
14.60       6.92   2.85   7.48 | 20.16  10.07  10.09 |   90.7  -262.1  22.12  lift
14.90       6.73   3.35   7.52 | 19.62  10.02   9.59 |   90.7  -262.0  42.93
15.10       6.34   4.38   7.71 | 19.23  10.72   8.51 |   90.7  -262.0  42.92  walking out
15.40       3.55  11.50  12.04 | 16.82  15.92   0.90 |   91.1  -261.9  42.88  wedged
16.10       3.65  11.26  11.84 | 16.84  15.69   1.15 |   91.1  -261.9  42.89  parked there
16.20       3.79  11.27  11.89 | 17.00  15.95   1.05 |   92.9  -263.2  42.93  traverse begins
16.50       7.19   1.96   7.45 | 20.23  10.24   9.99 |  106.8  -273.7  42.90  re-centring
16.60       8.26  -3.81   9.10 | 41.96  19.79  22.17 |  111.6  -277.3  10.18  gone
```

Between t=14.8 and 16.1 the TCP moves 0.4 mm — the arm is stationary — while the
cube's offset from it grows by 8.7 mm. That is the cube moving *inside* the
gripper, and it ends up wedged: the right jaw at 0.90–1.15 mm, effectively on its
shut stop, against the left at 15.7 mm, with the gap at 16.84 mm — narrower than
the cube. Then the arm accelerates into the traverse, the pair re-centres over
~0.1 s (left 13.95 → 10.24, right 4.07 → 9.99), and immediately after crossing
symmetry the fire-open begins.

A gap under 20 mm looked at first like the blades penetrating the cube, which
would have made the recovery impulse a candidate energy source. It is not that.
The replay below measures the cube's support half-width along the grip axis
explicitly rather than assuming 10 mm — a cube at any orientation is *wider*
across two parallel blades, never narrower — and the blades' actual overlap with
it never exceeds 0.30 mm. The narrow gap is the pair gripping an off-centre cube
near the blade tips, not the solver losing the collision.

## What it turned out to be

The fire-open is the *end* of the failure, not the start of it, and reading the
10 Hz ground-truth samples instead of the 60 Hz jaw log is what hid that. At full
rate, with the arm standing still and the command constant:

```
   t    asym     gap    drive effort   pen_l   cube z
14.80  -0.018  19.720     -1.478       0.304   42.86   holding at the 1.5 N cap
14.90  -0.096  19.734     -1.420       0.289   42.93
15.00  -0.324  19.784     -1.260       0.235   42.93
15.10  -0.904  19.924     -0.853       0.086   42.94
15.15  -1.457  20.219     -0.392      -0.206   42.94
15.20  -2.327  20.268     +0.026      -0.246   41.27   let go, free-falling
15.25  -3.667  24.982     +0.171      -3.611   18.27
```

The grip force bleeds smoothly to zero over 0.35 s and reverses. The applied
drive target is `0.000` on both jaws throughout and `grip_force_n()` reports the
1.5 N cap, so the command is not the problem. `pen_l` — the left blade's inner
face measured against the cube's own face, computed off the stage — decays in
lockstep with the effort and crosses zero exactly when the cube starts to fall.
The blades let go. Everything after t=15.25 is a gripper flailing at empty air.

The cause is `asym`. The two jaws move in *opposite* directions (left 9.851 →
8.971 while right 9.869 → 11.297): the pair is translating sideways along the
cube. The tendon's whole job is `q_left - q_right = 0`, and with a cube between
the blades the closing direction is blocked, so the only way it can reduce an
asymmetry is to push the lagging jaw *outward*. That opens the gap, and the gap
is the grip.

Run the identical trace with the tendon removed and the pair still wanders — the
asymmetry reaches 6 mm — but the gap holds at 19.709 mm and the effort stays
pinned at −1.499 N for the whole carry, and the cube arrives. It is not the
wander that drops the cube. It is the tendon converting the wander into gap.

## The bisection

Each row is the recorded clearing move replayed byte-identically through an
in-process `Mt4Machine` (`scratchpad/contact_probe.py wire_clear.txt`), ~20 s per
run and deterministic to the digit. Intended destination (132.7, −292.5).

| change | outcome | cube ends |
|---|---|---|
| none (`k=3e4, c=400`) | dropped in mid-air at t=15.20 | (98.4, −263.0) |
| tendon off (`k=0, c=0`) | held the whole carry | (139.9, −291.6) |
| `k=1e3` | held | (140.0, −292.1) |
| `k=3e3` | held | (140.0, −292.2) |
| `k=1e4` | dropped at t=15.45 | (96.9, −264.2) |
| `k=3e5`, `k=3e6` | fired fully open before the pick | never lifted |
| `c=0` (k unchanged) | fired open at t=14.0 | never lifted |
| armature 0.3 → 0.02 | held | (140.2, −293.0) |
| force cap 1.5 → 3 N | held | (140.0, −292.3) |
| force cap 1.5 → 5 N | dropped at t=15.05 | (100.7, −269.0) |
| force cap 1.5 → 15 N | held, but crushes the gap to 17.4 mm | (140.0, −292.2) |

Stiffness is monotonic and clean; the force cap is not (3 N holds, 5 N drops,
15 N holds), which is what a marginal system looks like when you push on the
wrong knob. Damping matters too but only below `k=3e3`: at `k=3e3` the cube
drops at `c=100`, at `k=1e3` it does not, which is why 1e3 rather than 3e3.

The residual 7.4 mm of x error in every run that held is the off-centre grab
vision handed it, not the tendon — it is the same 7.5 mm the cube starts the
carry with.

Two candidates were ruled out rather than fixed. Blade penetration is real but
small (0.30 mm) and is the *consequence* of the grip, not a source of energy —
it decays with the grip force rather than discharging. `PhysxMimicJointAPI`,
which would make the coupling an exact constraint and remove the trade
altogether, is present in the schema and ignored by this runtime; that was
already measured and is recorded in `README.md`.

## Notes for whoever picks this up

The recording and the logs share no timestamps. `out/stack_marker0.mp4` is 2629
frames at 10 fps = 262.90 s; the log spans 563.17 s. `RunPreview` writes at
10 fps while the arm moves and 1 fps while it stands still (`WAIT_SPEEDUP = 10`
in the control repo's `mt4_vision/instruct_view.py`) and plays back at 10, so the
mapping is neither an offset nor a scale. The alignment used above was recovered
by tracking the cube's own pixel height in the video (on the desk through 13.3 s,
rising at 13.4, held ~58 px up, back down at 15.3) and matching it to the cube's
z in the log (leaves at t=14.6, lands at t=16.6) — the same 2.0 s carry, giving
video ≈ log − 1.25 s *in that stretch only*.

That reconstruction should not have been necessary. Logging `(frame_index,
sim_t)` per written frame, or burning the sim clock into the canvas, makes it
exact and costs nothing.

The instruments are in the scratchpad and are worth keeping: `contact_probe.py`
(the replay above, with blade-versus-cube geometry, applied drive targets and
measured efforts) and `wire_clear.txt` / `wire_lvl1.txt`, the command traces
extracted from the recorded run's `note` column. A whole experiment is 20 s.
