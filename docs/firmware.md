# The firmware, replaced

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

## The counters are the truth, here as there

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
| Grip force | The jaws are a servo: a 600 N/m position loop whose wind-up is limited, so a close past contact stalls at **1.5 N** whatever the object's width, the way a real servo does. That is 19× an 8 g cube's weight — enough to shove a misaligned cube square against the desk — and small enough not to launch it. The limit is applied to the pair, not to each blade, which is what keeps the blades' midpoint on the wrist |
| Path shape | `mp`/`mq` chop a straight world line into 2 mm segments and solve each with the control repo's own `ik_position`, routing tangent-arc-tangent around the 140 mm keep-out cylinder |
| Rejections | `err not homed`, `err mp keepout`, `err mp ground z<115.0`, `err mp joints`, `err mq full 8`, `err mq station pose want … at …` — the exact strings the host greps for |
| Queue semantics | `mq` cold-starts when idle and queues when not, a drained queue emits one `mp done`, `mp` mid-flight overrides and drops the queue, a grip station holds everything until the jaws finish |

## Where it is a stand-in and says so

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

## Pointing a script at it

`mt4_jog.serial.open_serial` opens a COM port by name; nothing in the control
repo asks for a URL. `run_against_sim.py` replaces that one function with one
that dials the simulator's socket and then runs the target script exactly as
`python` would — so the script, and `Mt4Client`, never know. If you would rather
patch nothing at all, serve on one half of a virtual null-modem pair
(`--listen COM21`) and point the script at the other half the normal way.

## What it is checked against

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

## Related

- [camera.md](camera.md) — the other half of the substitution, the scene camera.
- [gripper.md](gripper.md) — why the grip force is what it is.
