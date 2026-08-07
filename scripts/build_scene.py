"""Write assets/mt4_scene.usda: the desk, ArUco tags, cubes and scene camera.

Headless. Run after scripts/import_urdf.py.
"""

from __future__ import annotations

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from mt4_sim.paths import ARM_USD, SCENE_USD, TEXTURES  # noqa: E402

from isaacsim import SimulationApp  # noqa: E402

app = SimulationApp({"headless": True})

from pxr import Usd  # noqa: E402

from mt4_sim import scene  # noqa: E402


def main() -> int:
    if not ARM_USD.is_file():
        print(f"missing {ARM_USD} -- run scripts/import_urdf.py first")
        return 1

    SCENE_USD.parent.mkdir(parents=True, exist_ok=True)
    SCENE_USD.unlink(missing_ok=True)
    stage = Usd.Stage.CreateNew(str(SCENE_USD))
    parked = scene.build(stage, ARM_USD, TEXTURES)
    stage.GetRootLayer().Save()

    print(f"wrote {SCENE_USD}")
    print("park pose written to:")
    for line in parked:
        print(f"  {line}")
    print("stage:")
    for prim in stage.Traverse():
        depth = prim.GetPath().pathElementCount - 1
        if depth <= 3:
            print(f"{'  ' * depth}{prim.GetName()}  <{prim.GetTypeName()}>")
    return 0


if __name__ == "__main__":
    code = main()
    app.close()
    sys.exit(code)
