"""Write assets/mt4.urdf from the control repo's kinematic constants."""

from __future__ import annotations

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from mt4_sim.urdf import write_urdf

OUT = Path(__file__).resolve().parent.parent / "assets" / "mt4.urdf"

if __name__ == "__main__":
    print(f"wrote {write_urdf(OUT)}")
