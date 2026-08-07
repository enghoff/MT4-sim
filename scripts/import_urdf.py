"""Convert assets/mt4.urdf to USD with Isaac Sim's URDF importer (headless).

Writes ``assets/mt4/mt4.usda`` plus its payload layers. Run
``scripts/build_urdf.py`` first.
"""

from __future__ import annotations

import shutil
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from mt4_sim.chain import ARM_JOINT_NAMES, FINGER_JOINT_NAMES  # noqa: E402
from mt4_sim.paths import ARM_USD, ASSETS, URDF  # noqa: E402

# A URDF says nothing about drive gains, and the converter leaves stiffness
# unset -- a "position" drive with zero stiffness has only damping, so the arm
# sags to wherever gravity puts it and ignores its target entirely.
#
# The MT4 is stepper-driven with no compliance: it either holds the commanded
# pose or loses steps. So the arm joints get stiffness high enough that gravity
# torque (worst case ~1 Nm at the shoulder) costs well under a tenth of a
# degree.
#
# This override is per *radian* -- the converter divides by 180/pi on the way
# into USD, which stores angular drives per degree. 12000 N.m/rad is 209
# N.m/deg, so 1 Nm of gravity torque droops about 0.005 deg.
#
# It has to be this stiff because of `j_head_level`. On the real arm the link
# rods hold the head platform level mechanically; here it is a driven joint, so
# it gives up its steady-state error to the gripper's weight, and a tilted head
# swings the TCP through the 35mm HEAD_OFFSET. At 2000 N.m/rad that cost 0.2mm
# of TCP error at rest -- the only way the simulated TCP can disagree with
# `fk_tcp` once the arm has settled.
#
# Damping is not set here: the converter takes it from the URDF's
# <dynamics damping>, ignoring override_joint_damping. See mt4_sim.urdf.
ARM_STIFFNESS = 12000.0

# The jaws are the exception: a grasp holds by squeezing, so the finger drives
# are deliberately soft and force-limited. N per metre, so 4000 N/m closing 1mm
# past contact presses with about 4N.
FINGER_STIFFNESS = 4000.0

from isaacsim import SimulationApp  # noqa: E402

app = SimulationApp({"headless": True})

from isaacsim.asset.importer.urdf import URDFImporter, URDFImporterConfig  # noqa: E402


def main() -> int:
    if not URDF.is_file():
        print(f"missing {URDF} -- run scripts/build_urdf.py first")
        return 1

    # The converter names its output directory after the robot, so clearing the
    # arm's own directory leaves the rest of assets/ (textures, scene) alone.
    if ARM_USD.parent.exists():
        shutil.rmtree(ARM_USD.parent)
    ASSETS.mkdir(parents=True, exist_ok=True)

    config = URDFImporterConfig(
        urdf_path=str(URDF),
        usd_path=str(ASSETS),
        fix_base=True,
        merge_fixed_joints=False,
        # The URDF carries hand-authored convex boxes for both roles, so the
        # collider is exactly the shape the visual shows.
        collision_from_visuals=False,
        collision_type="Convex Hull",
        allow_self_collision=False,
        joint_drive_type="position",
        joint_target_type="position",
        override_joint_stiffness={
            **dict.fromkeys(ARM_JOINT_NAMES, ARM_STIFFNESS),
            **dict.fromkeys(FINGER_JOINT_NAMES, FINGER_STIFFNESS),
        },
    )

    out = Path(URDFImporter(config).import_urdf())
    print(f"imported -> {out}")
    if out.resolve() != ARM_USD.resolve():
        print(f"WARNING: expected {ARM_USD}, converter wrote {out}")
    for p in sorted(ARM_USD.parent.rglob("*")):
        if p.is_file():
            print(f"  {p.relative_to(ARM_USD.parent)}  {p.stat().st_size:,} bytes")
    return 0


if __name__ == "__main__":
    code = main()
    app.close()
    sys.exit(code)
