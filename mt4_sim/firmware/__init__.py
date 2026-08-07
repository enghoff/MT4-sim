"""A stand-in for the MT4's firmware, driving the simulated arm.

``mt4_jog.client.Mt4Client`` opens a serial port, writes ``mp 230 -60 122 h 0``
and waits for ``mp done pos ...``. Everything in the control repo above that --
the vision stack, the pick/place primitives, the task scripts -- is built on it
and knows nothing else. Put something on the other end of the port that answers
the way the firmware answers, and all of it drives the simulation instead.

    from mt4_sim.firmware import Mt4Machine, open_link

    machine = Mt4Machine(arm)          # arm: mt4_sim.arm.SimArm
    link = open_link("tcp://127.0.0.1:5570")
    ...
    for line in link.poll():
        link.send_all(machine.handle_line(line))
    machine.tick(dt)
    link.send_all(machine.drain())

``scripts/serve_firmware.py`` is that loop wrapped around Isaac Sim, and
``scripts/run_against_sim.py`` points an unmodified control-repo script at it.
"""

from mt4_sim.firmware.link import LineLink, SerialLineLink, TcpLineLink, open_link
from mt4_sim.firmware.machine import BOOT_BANNER, ArmDriver, Mt4Machine
from mt4_sim.firmware.planner import Leg, plan_leg, validate_request
from mt4_sim.firmware.state import FirmwareState, Gripper

__all__ = [
    "BOOT_BANNER",
    "ArmDriver",
    "FirmwareState",
    "Gripper",
    "Leg",
    "LineLink",
    "Mt4Machine",
    "SerialLineLink",
    "TcpLineLink",
    "open_link",
    "plan_leg",
    "validate_request",
]
