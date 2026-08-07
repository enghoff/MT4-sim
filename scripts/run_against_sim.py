"""Run a control-repo script against the simulated arm, unmodified.

``mt4_jog.serial.open_serial`` opens a COM port by name. Nothing in the control
repo asks for a URL, so this replaces that one function with one that dials the
simulator's socket, and then runs the target script exactly as `python` would.
The script does not know, `Mt4Client` does not know, and nothing in the control
repo has to change to make it so.

    python scripts/serve_firmware.py                      # in one terminal
    python scripts/run_against_sim.py -- Z:\\MT4\\jog.py    # in another
    python scripts/run_against_sim.py -m mt4_vision.console
    python scripts/run_against_sim.py --check             # just prove the link

The address comes from ``--url``, or ``MT4_SIM_URL``, and defaults to the one
``serve_firmware.py`` prints. If you would rather not have anything patched at
all -- a virtual null-modem pair, say -- serve on one half of it
(``--listen COM21``) and point the script at the other half the normal way; then
this launcher is not needed.
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

DEFAULT_URL = "socket://127.0.0.1:5570"


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


def main() -> int:
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    ap.add_argument(
        "--url",
        default=os.environ.get("MT4_SIM_URL", DEFAULT_URL),
        help=f"where serve_firmware.py is listening (default {DEFAULT_URL})",
    )
    ap.add_argument("-m", "--module", help="run a module, as `python -m` would")
    ap.add_argument(
        "--check", action="store_true", help="connect, print the arm's status, exit"
    )
    ap.add_argument("target", nargs="*", help="script path, then its own arguments")
    args = ap.parse_args()

    url = _normalise(args.url)
    patch_serial(url)

    if args.check:
        return check(url)

    if args.module:
        sys.argv = [args.module, *args.target]
        runpy.run_module(args.module, run_name="__main__", alter_sys=True)
        return 0

    if not args.target:
        ap.error("give a script to run, -m MODULE, or --check")

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
