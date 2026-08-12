# The gripper

The hardest part of this model to get right, and the part with the most
measurement behind it. The real gripper is one servo driving a symmetric
scissor; the model is two prismatic joints that have to be made to behave like
one mechanism. Everything below is why the constants in `mt4_sim/chain.py` are
what they are.

[gripper-fires-open.md](gripper-fires-open.md) is the investigation that named
the central defect; this file is the durable version.

## Why the gripper can shove a cube square

Three numbers decide whether closing jaws push a misaligned cube into line or
just jam on its corners, and none of them is the grip force:

- **Cube-on-desk friction.** The pair μ is what the cube has to overcome to
  slide. At 0.5 and above it is welded to the table and the jaws stop dead on
  its corners at a 27 mm gap; at 0.35 and below it slides and turns. Plastic on
  wood is 0.2–0.4, so the physical value and the working value agree — the old
  1.1 was rubber-on-rubber, chosen when the grip was weak and needed the help.
- **Jaw armature** (`chain.FINGER_ARMATURE_KG`). Armature is the servo's
  reflected rotor inertia, which is real and dominates the blade's own mass;
  0.30 kg makes the contact quiet enough to do work. It was introduced against a
  drive clipped at `maxForce`, which has no damping left — the `c·v` term is
  clipped with it — so it became a constant-force actuator that bounced off
  contact at `F·dt/m` per step, 1.25 m/s on a 20 g blade at 60 Hz. The drive no
  longer clips, so that argument no longer stands on its own; the measured one
  below does.
- **Jaw speed** (`chain.FINGER_MAX_SPEED_M_S`) *used* to matter, and no longer
  does. With the short jaws the only thing that could turn a misaligned cube was
  the *momentum* of a fast blade, and dropping the ceiling to 0.12 m/s stopped
  the squaring entirely. Full-length tongs turn it with a static couple instead:
  20.0° of 20 at 0.5 m/s, 19.9° at 0.1.

Measured on a cube 20° off square: it turns 20.0° of 20 and ends face-gripped at
19.77 mm, 0.63 mm off the TCP.

## The armature moved the damping, and nobody moved it back

*This is the derivation for a drive two designs back — `k = 1000 N/m`, `c = 15`,
a 1.5 N cap that the jaws sat on whenever they gripped. The measurements are
sound and the reasoning about armature still holds; what changed is the drive it
was reasoning about. The current one is a servo — `k = 600`, `c = 40`, the
torque limit applied to the pair rather than to each blade — and the section
after this one is that design. What carried over and what did not is at the end
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
and it is still what sets ζ. The jaws-cycling failure is still this knob and not
the coupling.

**What did not.** Both bounds on `c` are gone, and with them the conclusion.
`F/c` was the terminal speed of a jaw *saturating* its cap, and no drive since
saturates per joint. Command tracking replaced it — a position drive following a
ramp lags by `c·rate/k` — and that was the bound that made **ζ > 1 unreachable**,
which is the claim this section ends on and the reason the jaws rang.

That bound was an artifact of driving a servo with position alone. Feed the
commanded rate forward and the drive damps `v − rate` instead of `v`; the drag
term the lag was paying for is gone, and damping stops costing tracking at all.
Measured on the same open, `c = 13.9` lags 15.73 mm without feed-forward and
3.51 mm with it. ζ = 1.44 is where the drive sits now, and `tests/test_chain.py`
asserts ζ > 1 — the thing the drive is actually for — in place of both.

## The jaws are a servo, and the torque limit is on the pair

The real gripper is one servo driving a symmetric scissor. It runs a position
loop; when the blades meet the object the loop cannot null its error, the motor
saturates, and the jaws become a **constant-force actuator at the servo's torque
limit** — a force that does not depend on how wide the object is.

Modelling that is not the same as writing `maxForce` on the drive, and the
difference is the whole design. A PhysX drive clamps *per joint*, and the pair
has two coordinates: `gap = left + right`, which the servo drives, and
`mid = (left − right)/2`, which the scissor welds to the wrist. Both blades
closing sit on the same cap in opposite directions, so the restoring term on
`mid` is not weak but **identically zero** — 0.001 N measured, against 1.5 N on
each blade — and the pair walks out from under the wrist carrying the payload.
That is the next section.

So the limit is applied to the gap and not to each blade. `SimArm.set_gripper_s`
limits how far the loop's reference may lead the blades' *mean* opening:

    half_gap = (left + right) / 2
    error    = commanded − half_gap
    target   = half_gap + clip(error, ±FINGER_WINDUP_M)      # both blades

Handing both blades that one target splits cleanly into the two modes:

    F_left + F_right = 2k·clip(error)   the servo, limited to the torque limit
    F_left − F_right = −2k·mid          the scissor, full stiffness, never clipped

which is the actual machine: a force-limited actuator on the coordinate it
drives, structure on the coordinate it does not. The drive's own `maxForce` stays
slack at 20 N and is a numerical backstop — a drive that reaches it is a drive
whose midpoint term has gone to zero. Nothing is latched: the target is
recomputed from the measured opening every step, so a cube that rotates flat and
pushes the blades apart is followed rather than abandoned.

**Two things this buys that the soft spring did not.** The grip is one number —
1.5 N on anything more than 5 mm inside the commanded opening, where before it
was `k` times the opening the object left, so 1.5 N on a 20 mm cube but 0.75 N
on a 10 mm one. And the loop can be damped for the armature it carries, because
the rate is fed forward, so the ringing is gone. On an open to `g 140` — the S
the host actually sends, not the open stop, where a blade against its travel
limit cannot overshoot at all:

| drive | ramp lag | overshoot | settle | reversals |
|---|---|---|---|---|
| k=150, c=4 (was) | 9.03 mm | **7.11 mm** | 0.43 s | 7 |
| **k=600, c=40, servo** | **1.28 mm** | **1.24 mm** | 0.067 s | 0 |

**Wind-up is a speed limit as well as a force limit**, and that is what bounds
the loop gain. The reference may lead the blades by at most `FINGER_WINDUP_M`, so
a blade can advance at most one wind-up length per physics step: the ceiling is
`FINGER_GRIP_FORCE_N / (k·dt)`. At `k = 1000` that is 0.090 m/s against the
firmware's 0.096 m/s sweep, and the jaws fall behind for the whole length of the
ramp — 5.6 mm of it, measured. `k = 600` clears the sweep by 1.57×, and
`tests/test_chain.py` asserts the margin.

**`physxJoint:jointFriction` was measured too**, since a geared servo is
non-backdrivable and friction would hold `mid` for free. It is honoured, but only
just: 3.0 N of it takes the open overshoot from 7.11 mm to 6.13 mm and does not
slow the close at all, where 3 N of Coulomb friction should stop a blade the
drive is pushing with 1 N outright. Nothing is built on it.

**What it costs.** The torque limit is symmetric, as a servo's is, so the jaws
can push *out* of an obstruction with 1.5 N where the old drive could use its
whole 6 N cap. That shows up only in a degenerate state — replaying a run whose
pick had already missed, the host lowers a shut and empty gripper onto the cube
already on the column, and the blades end up inside it. The old drive shoves the
cube 6 mm aside and opens; this one stalls at a 13 mm gap. Both are recovering
from the same interpenetration, and the live trials below never reach it.

## The jaws are two blades that should be one mechanism

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
[gripper-fires-open.md](gripper-fires-open.md) is the investigation — the gripper
firing fully open while commanded shut, and cubes dropped in mid-transit, are
both this coordinate reaching the end of its travel.

**What holds it is the jaw drives, and only while they stay off their force
cap.** Two position drives to a common target restore the midpoint with
`-2k*mid`. Clipped to the same cap in opposite directions they sum to nothing —
measured at 0.001 N against 1.5 N on each side — so a saturating grip leaves the
pair a free 0.6 kg mass with no spring and no damper, keeping whatever sideways
velocity a move onset hands it.

That is why the servo's torque limit is applied to the pair rather than to each
blade: it is exactly the arrangement that limits the grip without ever clipping a
joint, so `-2k*mid` survives at the full `k`. Replaying a place-down that used to
walk the pair 10.6 mm, the midpoint now holds to **-0.117 … +0.056 mm**, measured
two independent ways — from the joint values and from each blade's world origin
against `gripper_base` — agreeing to 0.0001 mm.

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

## What the checks say

`check_grip.py` passes 7 of 7 on the servo, with the tendon inert:

| case | cube ends, from the TCP | jaw asymmetry | grip | peak cube speed |
|---|---|---|---|---|
| square on | 0.23 mm | 0.04 mm | 1.50 N | 0.037 m/s |
| +8° mis-aimed | 0.52 mm | 0.02 mm | 1.50 N | 0.053 m/s |
| +12° mis-aimed | 0.68 mm | 0.03 mm | 1.50 N | 0.046 m/s |
| +20° mis-aimed | 1.93 mm | 0.01 mm | 1.50 N | 0.082 m/s |
| −20° mis-aimed | 0.99 mm | 0.02 mm | 1.50 N | 0.098 m/s |
| cube already at 25° | 0.21 mm | 0.02 mm | 1.50 N | 0.042 m/s |
| cube already at 30° | 0.19 mm | 0.08 mm | 1.50 N | 0.038 m/s |

Every mis-aimed case turns the cube square and ends **0.1–0.3° off the jaws**,
including ±20°, which the previous drive left stably corner-gripped. The grip
column reading 1.50 N seven times is the torque limit doing what a torque limit
does. The speed column is the other half: a limited position loop cannot push
harder than 1.5 N, so the worst shove is 0.098 m/s against the 1.5 m/s at which
this check calls a cube launched.

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

The servo was scored the same way, three three-level trials each on marker 2,
paired against the soft spring it replaces on the same scene and site:

| drive | levels built | missed picks | worst off-axis | strays | invariants |
|---|---|---|---|---|---|
| k=150, cap 6.0 N | 3, 3, 3 | 0, 0, 0 | 0.4, 0.2, 0.2 mm | 0, 0, 0 | 2 of 3 clean; one dropped a cube 20 mm at t=93 |
| **servo, k=600** | 3, 3, 3 | 0, 0, 0 | **0.2, 0.2, 0.1 mm** | 0, 0, 0 | **3 of 3 clean** |

Then three more sites, one trial each, because a site fixes the reach and
bearing the arm swings through and which cubes it can pick on the way:

| site | levels built | missed picks | worst off-axis | invariants |
|---|---|---|---|---|
| marker 0 | 3 | 0 | 0.4 mm | clean |
| marker 1 | 3 | 0 | 0.3 mm | clean |
| marker 3 | 3 | 0 | 1.2 mm | clean |

Marker 4 cannot be used and it is the control repo that says so: it sits at
(205.2, 9.9), under the camera park pose at (200, 0), and `stack_cubes.py`
refuses it — *the arm parks there between moves and would hit the stack*.

Read the whole set honestly: at three levels both drives stack every time, and
the trial is not where the servo earns its place — the ringing, the
width-independent grip and the 0.17 mm midpoint are. What the trials establish
is that none of it cost a pick, a level, or a millimetre, over four sites, which
is the thing a bench check cannot say.

They also broke `check_jaw_midpoint_fixed`, which is worth its own line. It
failed marker 3 on **both** drives, and the old one yielded 2.6x further
(3.32 mm against 1.26 mm) — because a pick aims at a detected position and the
pair gives while capturing an off-centre cube, which is the behaviour the
coupling section argues *for*. The check had been calibrated on a place-down
replay that contains no capture, so it was asserting something no correct run
satisfies. It now bounds how far the midpoint yields and how long it takes to
come back, and still fails the original 12.8 mm walk on both counts.

## Related

- [gripper-fires-open.md](gripper-fires-open.md) — the investigation behind all
  of this.
- [frames.md](frames.md) — why the tongs are 115 mm long, which is what lets
  them reach a cube at all.
- [firmware.md](firmware.md) — how the host commands the jaws (`g <S>`).
