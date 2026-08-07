"""Render ArUco tag textures for the simulated desk.

The real rig's whole coordinate mapping rests on decoding printed DICT_4X4_50
tags, so the simulated tags are real tags: generated from the same dictionary
and rendered at enough pixels that a 720p camera looking at them obliquely still
has several pixels per code cell.
"""

from __future__ import annotations

import hashlib
from pathlib import Path

import cv2
import numpy as np

from mt4_sim.rig import MARKER_DICT, MARKERS

# One code cell of a 4x4 tag spans a sixth of the printed square (4 data cells
# plus the 1-cell black border each side). At 768px the cell is 128px, so the
# oblique view still resolves cells after perspective foreshortening.
TAG_PIXELS = 768
# ArUco needs a light quiet zone around the black border to find the tag at all.
QUIET_CELLS = 1

_DICTS = {
    "4x4_50": cv2.aruco.DICT_4X4_50,
    "4x4_100": cv2.aruco.DICT_4X4_100,
}


def tag_image(tag_id: int, dict_name: str = MARKER_DICT) -> np.ndarray:
    """One tag as an RGB image, black tag on a white quiet zone."""
    dictionary = cv2.aruco.getPredefinedDictionary(_DICTS[dict_name])
    tag = cv2.aruco.generateImageMarker(dictionary, tag_id, TAG_PIXELS)

    pad = round(TAG_PIXELS * QUIET_CELLS / 6.0)
    canvas = np.full((TAG_PIXELS + 2 * pad, TAG_PIXELS + 2 * pad), 255, dtype=np.uint8)
    canvas[pad : pad + TAG_PIXELS, pad : pad + TAG_PIXELS] = tag
    return cv2.cvtColor(canvas, cv2.COLOR_GRAY2RGB)


def quiet_zone_fraction() -> float:
    """How much of the rendered card's side is quiet zone rather than tag.

    The card must be scaled so the *tag* is ``MARKER_SIZE_MM``, not the card.
    """
    pad = round(TAG_PIXELS * QUIET_CELLS / 6.0)
    return (TAG_PIXELS + 2 * pad) / TAG_PIXELS


def write_tag_textures(out_dir: Path) -> dict[int, Path]:
    """Write one PNG per rig marker. Returns tag id -> path.

    The filename carries a hash of the image, which is not decoration. Kit
    caches textures by path and does not notice the file changing underneath
    it: change which tags the rig carries, and the renderer will happily draw
    the *previous* tag from a path whose contents have since been rewritten --
    a scene that is correct in USD, correct on disk, and wrong in the frame,
    which is exactly as confusing to debug as it sounds. A path that can only
    ever hold one image makes that impossible.
    """
    out_dir.mkdir(parents=True, exist_ok=True)

    written: dict[int, Path] = {}
    for marker in MARKERS:
        ok, buffer = cv2.imencode(".png", tag_image(marker.tag_id))
        if not ok:
            raise RuntimeError(f"could not encode tag {marker.tag_id}")
        data = buffer.tobytes()
        digest = hashlib.sha1(data).hexdigest()[:10]
        path = out_dir / f"aruco_{MARKER_DICT}_{marker.tag_id}_{digest}.png"
        path.write_bytes(data)
        written[marker.tag_id] = path

    # Tags from an earlier layout would otherwise pile up unreferenced.
    keep = set(written.values())
    for stale in out_dir.glob(f"aruco_{MARKER_DICT}_*.png"):
        if stale not in keep:
            stale.unlink()

    return written
