# The CAD independently confirms the kinematics

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

## …and it corrects the two bodies J1 joins

The kinematics were right long before the *shapes* were. The base and the
rotating column were sketched to plausible proportions, and read far taller and
blockier than the real machine. The CAD says:

| | model was | CAD (above the desk) |
|---|---|---|
| foot plate | 140 × 120, 8 tall | **110 × 130**, 5 tall |
| pedestal | 104 × 92, running the full 8 → 76 | **110 × 110, stopping at 54** |
| J1 shroud | — | **85 × 72**, 54 → 75 |
| yoke | 96 × 84 to 142, plus a 62 × 64 cap to 162 | **106 × 64**, 76 → 160 |
| shoulder steppers | absent | **two 42 × 84 × 42**, 87 → 129, ±92 wide |
| base centre | on the J1 axis | **20 mm forward of it** |

Total height was close all along — 76 mm of base against the CAD's 75. What made
it read tall was the missing **step at 54 mm**: the real pedestal is a squat
110 × 110 box that necks down to a narrow shroud, where the model ran one
104 × 92 block the whole way up. The two shoulder steppers are the other half of
it — at 184 mm tip to tip they are the widest thing on the machine, and drawing
them as nothing left the column a plain tower.

Both bodies rotate or stand clear of the wood (the rotating body's underside is
54 mm up), so none of this is load-bearing for collision; it is what the arm
looks like, and `rig.DESK_BAY_*` now has the real footprint to clear.

## Real meshes are the obvious next step

The arm currently renders as boxes sized from the CAD bounding boxes:
dimensionally right, visually plain. The registration needed to swap in the
actual STL meshes is already computed — `tools/step_assembly.json` carries each
mesh's rotation and translation into its link frame, found by matching STEP
B-rep vertices against mesh vertices (`Link1.STL` registers at 97% with 0.0000 mm
residual). Two caveats found along the way: `base_link.STL` is a different
revision from this STEP and only matches 54%, and the mesh variants have plain
through-holes where the STEP has stepped bearing seats.

## Related

- [kinematics.md](kinematics.md) — the chain these measurements confirm.
