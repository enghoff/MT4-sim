# The parallel linkage, as a serial chain

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

## Where the head-level joint is not the real thing

On the arm, rigid rods hold the head level. Here it is a driven joint, so it
gives up its drive's steady-state error to the gripper's weight, and a head
tilted by θ swings the TCP through `HEAD_OFFSET · sin θ`. That is the **only**
way the simulated TCP can disagree with `fk_tcp` once the arm has settled, and
`check.py` asserts exactly that identity rather than hiding it behind a loose
tolerance — which is what makes the 0.0002 mm figure in
[verification.md](verification.md) a real statement about the chain.

At the drive stiffness used (12000 N·m/rad) the tilt is under 0.18° and the TCP
artefact under 0.11 mm. Both scale with the physics step — an implicit drive is
effectively softer at a coarser one — so they were half that when the scene ran
at 240 Hz and are what they are at the 60 Hz it runs at now. For scale, the real
arm's measured backlash is 6–9 mm on small reversing moves.

Modelling the rods properly needs a closed kinematic loop, which URDF cannot
express and USD can. Worth doing only if something starts caring about 0.1 mm.

## Related

- [frames.md](frames.md) — what z = 0 is, and why `table_z` is not the desk.
- [cad-audit.md](cad-audit.md) — the same constants, recovered independently
  from WLKATA's official CAD.
