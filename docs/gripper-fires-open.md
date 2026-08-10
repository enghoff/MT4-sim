# The jaws have a coordinate the real gripper does not

Found 2026-08-09 as "the gripper fires fully open while commanded shut", in the
recorded marker-0 stacking run (`out/stack_marker0.mp4`, log `jaws_rec0.csv`).
It turned out to be one of three symptoms of a single modelling error, and the
file keeps its name because `mt4_sim/chain.py` links to it.

**The error.** The two blades are independent prismatic joints sharing an origin
on `gripper_base` — which *is* the J4 frame — with axes `(0,+1,0)` and
`(0,-1,0)`. So the pair has two coordinates:

    gap = left + right       the drives hold this
    mid = (left - right)/2   nothing holds this

`mid` is where the pair sits along the grip axis, measured from the wrist. On the
hardware it is a weld: one servo, a symmetric scissor, and the blades cannot move
independently at all. Here it is free, and it moves — up to 12.8 mm, both tongs
together, the gap unchanged, and away from the robot's base on 8 of 9 place-downs
in a run.

Everything below is downstream of that. The durable version lives in `chain.py`
beside the constants; this is the investigation.

## Three symptoms, one coordinate

**1. Fires fully open.** Carrying a cube ~150 mm above the desk, both jaws travel
from a 20 mm hold to their fully-open stop in 0.12 s, on the joint velocity cap
the whole way, then slam shut again. The cube falls 33 mm. The gripper is
commanded **fully shut** for every step.

```
   t       S    cmd gap    left    right     gap      vl       vr
16.500   255      0.00   10.244   9.989   20.233  -0.011  +0.089   holding the cube
16.533   255      0.00   10.783  13.216   23.999  +0.129  +0.150
16.583   255      0.00   17.501  19.955   37.456  +0.151  +0.150
16.633   255      0.00   24.102  24.535   48.638  +0.127  -0.000   both on the open stop
16.800   255      0.00    2.864   0.660    3.524  -0.150  -0.145   shut on nothing
```

The joint range is 0 (blades touching) to 24.535 mm, so 48.638 mm is both jaws
hard against their upper stops. The host's actual open command (`g 140`) does not
execute until t≈17.9, over a second after the cube is on the desk.

**2. Drops a cube in transit.** The same thing at the other end of the travel:
commanded shut with the left jaw against its 24.535 mm stop, the gap opened
19.7 → 25.6 mm and a cube fell 128 mm onto the column.

**3. The midpoint walks.** The one that is visible without instruments, and the
one that named the cause. A place-down, 60 Hz, arm descending with x/y locked on
the marker:

```
     t   tcp_z |    left   right     gap |     mid   d(mid)
 47.30   150.9 |  18.073   1.503  19.576 |  +8.285   +0.000   parked
 47.40   149.2 |  16.861   2.717  19.578 |  +7.072   -1.202   descent starts
 47.70   140.1 |  13.184   6.446  19.630 |  +3.369   -1.118
 48.10   127.8 |   9.223  10.415  19.638 |  -0.596   -0.937
 48.30   125.8 |   7.460  12.133  19.593 |  -2.336   -0.837   still moving; descent over
```

The gap never moves. The pair slides 10.6 mm at a dead-constant 10 mm/s, starts
when the descent starts, and *keeps going after the descent stops*.

Measured two independent ways — the articulation's joint values, and each blade's
world origin minus `gripper_base`'s projected on the grip axis — those agree to
**0.0001 mm**, with **0.0025 mm** of off-axis motion. It is a rigid translation
of both tongs relative to J4, not an artifact of the arithmetic.

## Why nothing holds it

Two position drives to a common target *do* restore the midpoint, with force
`-2k*mid`. But only while they can still modulate. Gripping, the host commands
S=255 — a negative span — so both drives sit hard on the force cap. Measured
through the whole slide:

```
   step      t   tcp_z      vz  |     mid   d(mid)/dt   |  eff_l   eff_r   net_y
   2839  47.317  150.89    -0.2 |  +8.274     -0.59 mm/s | -1.499 -1.495 +0.0031
   2841  47.350  150.86    -0.6 |  +8.101     -9.10 mm/s | -1.439 -1.686 -0.2474
   2844  47.400  149.66   -28.4 |  +7.330    -16.11 mm/s | -1.499 -1.536 -0.0372
   2850  47.500  146.63   -30.5 |  +5.904    -13.11 mm/s | -1.505 -1.505 -0.0001
```

Both efforts pinned at the 1.5 N cap, opposed, net **0.001 N**. The restoring
term is not weak, it is identically zero, and the pair is a free 0.6 kg mass
(mostly `FINGER_ARMATURE_KG`) with no spring and no damper.

Step 2841 is the whole event: one step *before* the TCP moves, the efforts split
to -1.439 / -1.686, net -0.247 N — 165x the baseline. That impulse on 0.6 kg
predicts Δv = 6.9 mm/s; the measured jump is 7.8. Then the net returns to zero
and the pair **coasts**. The descent never pushes it; nothing ever takes the
velocity back.

## Why the tendon is not the fix

The tendon holds `q_left - q_right = 0`, which is the midpoint, so it looks like
exactly the right thing. It is a force the step integrates rather than a
constraint it solves, and there is no stiffness that works:

| tendon k (c=0) | midpoint range | grip force | |
|---|---|---|---|
| 0 | 12.8 mm | 1.50 N | the defect |
| 20 | 12.5 mm | 1.50 N | inert |
| 65 | 12.5 mm | 1.50 N | inert — this is `chain.py`'s own force budget |
| 200 | 14.7 mm | p95 2.34 N | worse than nothing |
| 600 | 9.9 mm | **max 9.83 N** | rectifying into the gap |
| 3e4 | — | — | fires open (symptom 1) |

Under the drive cap it cannot resist a 1.5 N contact push. Over it, a one-sided
stop absorbs the reaction on one blade and the force appears on the other as
*gap* — which is the grip. That is symptoms 1 and 2, at opposite ends of travel.

## What a real constraint would need, and why there isn't one

`mid` should not exist. Three mechanisms were measured against this articulation
and none of them removes it:

* **`PhysxMimicJointAPI`** — applied and ignored. Told left=0 and right=24.5 the
  jaws go to exactly 0.00 / 24.50, the same as with nothing applied.

* **`PhysxPhysicsRackAndPinionJoint`** — the scissor's actual mechanism, and the
  most interesting failure. On free rigid bodies it is exact: ramp one rack and
  an undriven one mirrors it with 0.000 mm of error. On the articulation's finger
  joints it *appears* to work at ratio ≥ 1e4 — jaws told to opposite ends of
  travel end up together, midpoint +0.00 mm.

  It is not working. A gear reflects its pinion's inertia to the rack as
  `I * ratio²`, and a 4 mm pinion at ratio 1e4 lands **2.7 kg** on a 0.30 kg
  armature. The jaws are not held together, they are too heavy to move apart. At
  ratio 1e5 the reflected 267 kg stops them closing on a cube at all — which
  reads as "even more coupled" and is really "seized".

  Hold the ratio and vary only the inertia, and it collapses:

  | ratio | pinion inertia | midpoint |
  |---|---|---|
  | 1e4 | default (2.7e-8) | +0.00 mm — "coupled" |
  | 1e4 | 1e-10 | -12.27 mm — ignored |
  | 1e4 | 1e-12 | -12.27 mm — ignored |
  | 1e5 | 1e-12 | -12.27 mm — ignored |

  A constraint does not care what the pinion weighs. **Sweeping the ratio is what
  hides this; holding the ratio and sweeping the inertia is what shows it.**

* **the fixed tendon** — a force, not a constraint. See above.

## The fix, and what it costs

Keep the drives off their cap, so `-2k*mid` survives. The grip force was always
sized from the task (1.5 N, ~19x the cube's weight); it now comes from the spring
rather than the clip, with `k` derived from it: half the clear opening a 20 mm
cube leaves each jaw is 10 mm, and 1.5 N over 10 mm is 150 N/m. The cap moves to
6.0 N, clear of the 3.68 N the spring asks for at the open stop.

| drive | midpoint range | grip force | command lag |
|---|---|---|---|
| k=1000, cap 1.5 N (was) | 12.8 mm | 1.50 N | 1.5 mm |
| k=150, cap 6.0 N (this) | **0.5 mm** | 1.46 N | **5.9 mm** |

**The cost is command tracking and it is not tunable away.** A position drive
following a ramp lags by `c*rate/k`; the softness that buys the midpoint its
spring is the softness that makes the jaws trail a commanded sweep — about 0.1 s
late on a free-space close. Damping trades the two directly (c=15: 0.36 mm of
midpoint against 10.0 mm of lag; c=2.25: 0.87 against 5.5), and 4.0 is the knee.

This is a mitigation standing in for a constraint the runtime will not provide.
If a future runtime honours `PhysxMimicJointAPI` on articulation DOFs, that is
strictly better and the drive can go back to being stiff.

## What actually replaced it: limit the pair, not the blade

*Added 2026-08-10. The section above is the design this superseded; its diagnosis
of the coordinate is unchanged and is the reason the current one is shaped the
way it is.*

The soft spring bought the midpoint by giving up two things that turned out not
to need giving up.

**It made the grip force depend on the object.** `k` was pinned at 150 N/m
because the grip was `k` times the opening the object left — 1.5 N on a 20 mm
cube, 0.75 N on a 10 mm one — for a machine whose entire behaviour is "stall at
the torque limit".

**It made the jaws ring.** At `k = 150` critical damping is 13.9 and the tracking
budget allowed `c = 4`, so ζ = 0.29. An open to `g 140` overshot by **7.11 mm of
gap and rang for 0.43 s through seven reversals**. Measure it at the open stop
and you see nothing at all — the blade is against its travel limit and *cannot*
overshoot — which is why this sat unnoticed.

Both come off the same observation. **The per-joint clamp is what destroys
`mid`, not the limiting.** So limit the pair instead: `SimArm._servo_command`
clamps how far the loop's reference may lead the blades' *mean* opening, and
hands both blades that one target. The sum splits,

    F_left + F_right = 2k*clip(error)   the servo, at its torque limit
    F_left - F_right = -2k*mid          the scissor, full k, never clipped

so the grip is the torque limit whatever the object, and the midpoint keeps its
restoring term at a stiffness the spring design could not afford.

The ringing needed the second half: **feed the reference's rate forward**, so the
drive damps `v - rate` rather than `v`. The `c*rate/k` lag is what made this file
say ζ > 1 was unreachable, and it is an artifact of driving a servo with position
alone. Rate is the servo's own signal.

| drive | midpoint range | grip | ramp lag | overshoot | settle | reversals |
|---|---|---|---|---|---|---|
| k=1000, cap 1.5 N | 12.8 mm | 1.50 N | — | — | — | — |
| k=150, cap 6.0 N | 0.5 mm | 1.46 N | 9.03 mm | 7.11 mm | 0.43 s | 7 |
| **servo, k=600** | **0.17 mm** | **1.50 N** | **1.28 mm** | **1.24 mm** | **0.067 s** | **0** |

Two things bound the servo, both worth knowing before touching it:

* **Wind-up is a speed limit.** The reference leads by at most one wind-up
  length, so a blade advances at most that far per step: the ceiling is
  `F_grip / (k*dt)`. At `k = 1000` that is 0.090 m/s against the firmware's
  0.096 m/s sweep and the jaws fall behind for the whole ramp — 5.6 mm of it.
  `k = 600` clears it by 1.57x.
* **The rate fed forward has to be the reference's, not the host's.** Stalled on
  a cube the reference is pinned and its rate falls to zero on its own; the
  host's does not, and 3.8 N of `c*rate` on a 1.5 N grip is not a grip. Feeding
  it forward only while unsaturated fails the other way — a sweep is saturated
  for most of its length, so the drive pushes a constant `F` against its own
  damper and tops out at `F/c`, 0.037 m/s, which is the old speed floor back
  again.

**`physxJoint:jointFriction` was measured here too**, since a geared servo is
non-backdrivable and friction would hold `mid` for free. It is honoured but only
barely: 3.0 N takes the open overshoot from 7.11 mm to 6.13 mm and does not slow
the close at all, where 3 N of Coulomb friction should stop a blade the drive is
pushing with 1 N outright. Nothing is built on it.

**What it costs.** The torque limit is symmetric, as a servo's is, so the jaws
push *out* of an obstruction with 1.5 N where the soft spring could reach its
6 N cap. Replaying a run whose pick had already missed — the host lowers a shut,
empty gripper onto the cube already on the column, and the blades end up inside
it — the spring shoves the cube 6 mm aside and opens, and the servo stalls at a
13 mm gap. Both are recovering from an interpenetration rather than doing
physics; six live trials never reached the state. It is the thing to look at
first if a run ever wedges with the jaws part-open.

## Two measurements that were lying

Both were found by numbers disagreeing with runs already watched, which is the
only reason they were found at all.

**`check_payload_not_dropped` counted every commanded place-down as a drop.**
"Held in a shut gripper and losing altitude" describes a drop, and it equally
describes lowering a cube 24 mm onto a column. It reported nine violations a run
— eight levels plus a clearing hop — on runs that placed all nine cubes
perfectly, which made the drop rate look flat at ~50 per 100 grip-closes at every
tendon setting and hid the fact that removing the tendon had fixed it:

| tendon k | drops per 100 grip-closes, corrected |
|---|---|
| 3e4 | 20, 20, 24 |
| 1e3 | 38, 46, 29 |
| 0 | **0, 0, 0** |

The fix is to require the fall be one the TCP did not make with it.

**`check_grip.py` reported the force cap as the grip force**, printing "6.0 N"
three lines above a measured press of 1.49 N. Harmless while the cap *was* the
grip; a trap the moment it stopped being.

## Notes for whoever picks this up

The recording and the logs share no timestamps. `out/stack_marker0.mp4` is 2629
frames at 10 fps = 262.90 s; the log spans 563.17 s. `LiveFeed` in the control
repo's `mt4_vision/preview.py` captures, detects and annotates per tick and
sleeps only the remainder, so it drops frames under load — the compression is
load-dependent, not a fixed idle speed-up, and no offset or scale recovers it.
The alignment used above came from tracking the cube's own pixel height in the
video and matching it to the cube's z in the log, giving video ≈ log − 1.25 s
*in that stretch only*.

That should not have been necessary. `MT4_FRAME_LOG` now writes
`(frame_index, monotonic)` per written frame, against the log's `wall` column.

The instruments are in the scratchpad and are worth keeping. A whole experiment
is 20 s:

* `contact_probe.py` — replay with blade-versus-cube geometry, applied drive
  targets and measured efforts.
* `midpoint_probe.py` — replay with the midpoint measured from joints *and* from
  the stage, its world direction relative to the base, and `--pinion` /
  `--pinion-inertia` for the constraint experiments.
* `rnp_track.py`, `rnp_articulation.py` — does this runtime enforce a coupling,
  on free bodies and on articulation DOFs.
* `wire_clear.txt`, `wire_lvl1.txt`, `wire_place1.txt`, `wire_drop138.txt` —
  command traces extracted from recorded runs' `note` columns.
