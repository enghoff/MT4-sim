"""Where this project's generated assets live."""

from __future__ import annotations

from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
ASSETS = ROOT / "assets"

URDF = ASSETS / "mt4.urdf"
# The URDF converter writes into a directory named after the robot, so the
# import target is ASSETS and the layer lands one level down.
ARM_USD = ASSETS / "mt4" / "mt4.usda"
TEXTURES = ASSETS / "textures"
SCENE_USD = ASSETS / "mt4_scene.usda"
