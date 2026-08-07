"""Run a control-repo script against the simulated arm, unmodified.

``mt4_jog.serial.open_serial`` opens a COM port by name. Nothing in the control
repo asks for a URL, so this replaces that one function with one that dials the
simulator's socket, and then runs the target script exactly as `python` would.

The scene camera is simpler: set ``MT4_CAMERA_URL=shm://mt4_scene_cam`` (this
launcher does that by default) and ``mt4_vision.camera`` opens the sim feed
itself -- see ``mt4_vision.sim_feed``.

    python scripts/serve_firmware.py --camera             # in one terminal
    python scripts/run_against_sim.py -- Z:\\MT4\\jog.py    # in another
    python scripts/run_against_sim.py -m mt4_vision.console
    python scripts/run_against_sim.py --check             # just prove the link
    python scripts/run_against_sim.py --check-camera      # prove the frame feed
"""

from __future__ import annotations

import argparse
import os
import runpy
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))

from mt4_sim import mt4_repo  # noqa: E402,F401  (puts the control repo on sys.path)
from mt4_sim.camera_feed import shm_url  # noqa: E402

DEFAULT_URL = "socket://127.0.0.1:5570"
DEFAULT_CAMERA = shm_url()


def _normalise(url: str) -> str:
    """Accept whatever `serve_firmware.py` printed, hand back a pyserial URL."""
    if url.startswith("tcp://"):
        return "socket://" + url[len("tcp://") :]
    if "://" not in url:
        return "socket://" + url
    return url


def patch_serial(url: str) -> None:
    """Point every path that opens the arm's port at the simulator instead."""
    import serial

    import mt4_jog.ports as ports
    import mt4_jog.serial as jog_serial

    def open_simulated(port: str | None = None, baud: int = 115200):
        return serial.serial_for_url(url, baudrate=baud, timeout=0.5)

    jog_serial.open_serial = open_simulated
    # Port auto-detection would go hunting for a CH340 that is not there. It is
    # never reached once open_serial is replaced, but jog.py prints what
    # resolve_port returns before connecting, so give it something true.
    ports.resolve_port = lambda port=None, **_: port or url
    ports.find_mt4_port = lambda **_: url
    ports.probe_port = lambda *_args, **_kwargs: True

    # Modules that did `from mt4_jog.serial import open_serial` at import time
    # hold their own reference; rebind it wherever it has already landed.
    for name, module in list(sys.modules.items()):
        if name.startswith(("mt4_jog", "mt4_vision", "mt4_mcp")):
            if getattr(module, "open_serial", None) is not None:
                module.open_serial = open_simulated


def point_camera_at_sim(feed_url: str) -> None:
    """Make ``mt4_vision.camera`` open the sim feed via ``MT4_CAMERA_URL``."""
    os.environ["MT4_CAMERA_URL"] = feed_url
    # Modules that already imported DEFAULT_CAMERA_URL need the live value too.
    import mt4_vision.camera as vision_camera

    vision_camera.DEFAULT_CAMERA_URL = feed_url


def check(url: str) -> int:
    """Connect the control repo's own client and report what it finds."""
    from mt4_jog.client import Mt4Client, Mt4ClientError

    client = Mt4Client(port=url)
    try:
        status = client.get_status()
    except Mt4ClientError as exc:
        print(f"could not reach the simulated arm at {url}: {exc}")
        print("is scripts/serve_firmware.py running?")
        return 1
    print(f"connected to {url}")
    print(f"  homed   {status.homed}")
    print(f"  mode    {status.mode} / orient {status.orient} / speed {status.speed_us}us")
    print(f"  joints  {status.joints}")
    print(f"  tcp     {status.tcp.as_dict() if status.tcp else None}")
    client.close()
    return 0


def check_camera(feed_url: str) -> int:
    """Pull one frame through mt4_vision.camera and try to decode tags."""
    import cv2

    from mt4_vision import camera as vision_camera
    from mt4_vision.detect import detect_markers

    from mt4_sim import rig

    try:
        frame = vision_camera.capture_frame(url=feed_url)
    except Exception as exc:  # noqa: BLE001 - surface whatever the feed raised
        print(f"could not read the simulated camera at {feed_url}: {exc}")
        print("is scripts/serve_firmware.py --camera running?")
        return 1

    print(f"camera feed {feed_url}")
    print(f"  frame   {frame.shape[1]}x{frame.shape[0]} BGR")
    markers = detect_markers(frame)
    found = sorted(m.marker_id for m in markers)
    expected = sorted(m.tag_id for m in rig.MARKERS)
    print(f"  ArUco   expected {expected}")
    print(f"  ArUco   decoded  {found}")
    missing = [t for t in expected if t not in found]
    if missing:
        print(f"  FAIL    tags {missing} missing")
        return 1
    out = ROOT / "out" / "sim_camera_feed.png"
    out.parent.mkdir(parents=True, exist_ok=True)
    cv2.imwrite(str(out), frame)
    print(f"  wrote   {out}")
    return 0


def main() -> int:
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    ap.add_argument(
        "--url",
        default=os.environ.get("MT4_SIM_URL", DEFAULT_URL),
        help=f"where serve_firmware.py is listening (default {DEFAULT_URL})",
    )
    ap.add_argument(
        "--camera",
        default=os.environ.get("MT4_CAMERA_URL")
        or os.environ.get("MT4_SIM_CAMERA", DEFAULT_CAMERA),
        help=f"scene-camera feed URL (default {DEFAULT_CAMERA})",
    )
    ap.add_argument(
        "--no-camera",
        action="store_true",
        help="do not set MT4_CAMERA_URL (leave the real USB device alone)",
    )
    ap.add_argument("-m", "--module", help="run a module, as `python -m` would")
    ap.add_argument(
        "--check", action="store_true", help="connect, print the arm's status, exit"
    )
    ap.add_argument(
        "--check-camera",
        action="store_true",
        help="grab one simulated frame, decode ArUco, exit",
    )
    ap.add_argument("target", nargs="*", help="script path, then its own arguments")
    args = ap.parse_args()

    url = _normalise(args.url)
    patch_serial(url)
    if not args.no_camera:
        point_camera_at_sim(args.camera)

    if args.check:
        return check(url)
    if args.check_camera:
        if args.no_camera:
            ap.error("--check-camera needs the camera feed; omit --no-camera")
        return check_camera(args.camera)

    if args.module:
        sys.argv = [args.module, *args.target]
        runpy.run_module(args.module, run_name="__main__", alter_sys=True)
        return 0

    if not args.target:
        ap.error("give a script to run, -m MODULE, --check, or --check-camera")

    script = Path(args.target[0]).resolve()
    if not script.is_file():
        ap.error(f"no such script: {script}")
    # A script run this way must see the world it expects: its own directory on
    # sys.path and its own name in argv[0], exactly as `python <script>` gives it.
    sys.path.insert(0, str(script.parent))
    sys.argv = [str(script), *args.target[1:]]
    runpy.run_path(str(script), run_name="__main__")
    return 0


if __name__ == "__main__":
    sys.exit(main())
