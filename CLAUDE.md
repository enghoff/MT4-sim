# Working on this repo

## Reporting

Write for someone who does not know this codebase. Lead with what happened and
what it means; keep it to a few sentences. Name a file, symbol or constant only
when the reader has to go look at it.

Do not narrate the investigation — no tool-by-tool log, no list of what was
ruled out. State the finding, the evidence for it, and what is still unknown.

Detail belongs in the scratchpad instruments and their docstrings, not in the
reply.

## Measurement

Run `scripts/check_invariants.py` on every log before analysing anything. Add a
check whenever a new class of failure turns up.

Validate a metric against a known-good case before citing a number from it. A
check that fails on a correct run is a broken check, not a finding.

Match the sample rate to the event. The 60 Hz jaw log, not the 10 Hz cube log,
for anything shorter than a second.

Ground truth is the cube poses on the stage. `stack_cubes.py` places by dead
reckoning and never looks again, so its console output is a claim.

Use `verify_stack2.py`, not `verify_stack.py`.

Normalise before comparing runs — per grip-close, not per run. Run lengths
differ by 3x.

A fix is not proven by the case it was derived from. Confirm on an independent
case before writing "fixed".

## Reproducing

Replay, don't re-run. Extract the wire trace from a log's `note` column into an
in-process `Mt4Machine`: deterministic, ~20 s against ~10 min for a real trial,
and any constant can be varied against identical commands.

## Trials

Pre-flight, lock, bound the time, redirect stdin. `stack_cubes.py` blocks on
stdin when it runs out of reachable cubes; stopping a batch leaves its
descendants running.

Recording needs `MT4_FRAME_LOG` (control repo) alongside the log's `wall`
column. Video time is not run time.
