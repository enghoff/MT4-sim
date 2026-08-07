"""Where the host's bytes come from.

The firmware layer is a line in, lines out state machine; this is the pipe. Two
pipes, in fact, because there are two ways to point the control repo at a
simulated arm:

* **A TCP socket** (default). Nothing to install: ``pyserial`` speaks
  ``socket://host:port`` natively, so a host that opens its port through
  ``serial_for_url`` reaches this with no driver in between. That is what
  ``scripts/run_against_sim.py`` arranges for scripts that hard-code a COM port.
* **A real serial port**, for a virtual null-modem pair (com0com and friends).
  Point this end at one half and the control repo at the other, and even code
  that enumerates ports and probes them finds an MT4.

Either way this is one connection at a time, like the port it stands in for.
"""

from __future__ import annotations

import queue
import socket
import threading


class LineLink:
    """One host connection, as lines."""

    def poll(self) -> list[str]:
        """Lines the host has sent since the last call."""
        raise NotImplementedError

    def send(self, line: str) -> None:
        raise NotImplementedError

    def send_all(self, lines) -> None:
        for line in lines:
            self.send(line)

    @property
    def connected(self) -> bool:
        raise NotImplementedError

    def take_connect(self) -> bool:
        """True once per new host connection, so the caller can greet it."""
        raise NotImplementedError

    def close(self) -> None:
        raise NotImplementedError

    @property
    def description(self) -> str:
        raise NotImplementedError


class TcpLineLink(LineLink):
    """A listening socket that behaves like a serial port with one host on it."""

    def __init__(self, host: str = "127.0.0.1", port: int = 5570) -> None:
        self._incoming: queue.Queue[str] = queue.Queue()
        self._connects: queue.Queue[bool] = queue.Queue()
        self._client: socket.socket | None = None
        self._client_lock = threading.Lock()
        self._closing = False

        self._server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self._server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._server.bind((host, port))
        self._server.listen(1)
        self.host, self.port = self._server.getsockname()

        self._thread = threading.Thread(target=self._serve, name="mt4-link", daemon=True)
        self._thread.start()

    @property
    def description(self) -> str:
        return f"socket://{self.host}:{self.port}"

    def _serve(self) -> None:
        while not self._closing:
            try:
                client, _ = self._server.accept()
            except OSError:
                return
            client.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            with self._client_lock:
                if self._client is not None:
                    # One host at a time, same as the port this replaces.
                    try:
                        self._client.close()
                    except OSError:
                        pass
                self._client = client
            self._connects.put(True)
            self._read_client(client)

    def _read_client(self, client: socket.socket) -> None:
        buffer = ""
        while not self._closing:
            try:
                chunk = client.recv(4096)
            except OSError:
                break
            if not chunk:
                break
            buffer += chunk.decode("utf-8", "replace")
            while "\n" in buffer:
                line, buffer = buffer.split("\n", 1)
                self._incoming.put(line.rstrip("\r"))
        with self._client_lock:
            if self._client is client:
                self._client = None
        try:
            client.close()
        except OSError:
            pass

    def poll(self) -> list[str]:
        lines = []
        while True:
            try:
                lines.append(self._incoming.get_nowait())
            except queue.Empty:
                return lines

    def send(self, line: str) -> None:
        with self._client_lock:
            client = self._client
        if client is None:
            return
        try:
            client.sendall((line + "\n").encode("ascii", "replace"))
        except OSError:
            with self._client_lock:
                if self._client is client:
                    self._client = None

    @property
    def connected(self) -> bool:
        with self._client_lock:
            return self._client is not None

    def take_connect(self) -> bool:
        try:
            self._connects.get_nowait()
            return True
        except queue.Empty:
            return False

    def close(self) -> None:
        self._closing = True
        with self._client_lock:
            if self._client is not None:
                try:
                    self._client.close()
                except OSError:
                    pass
                self._client = None
        try:
            self._server.close()
        except OSError:
            pass


class SerialLineLink(LineLink):
    """One end of a virtual null-modem pair."""

    def __init__(self, port: str, baud: int = 115200) -> None:
        import serial

        self._serial = serial.Serial(port=port, baudrate=baud, timeout=0)
        self._buffer = ""
        self._greeted = False

    @property
    def description(self) -> str:
        return str(self._serial.port)

    def poll(self) -> list[str]:
        waiting = self._serial.in_waiting
        if waiting:
            self._buffer += self._serial.read(waiting).decode("utf-8", "replace")
        lines = []
        while "\n" in self._buffer:
            line, self._buffer = self._buffer.split("\n", 1)
            lines.append(line.rstrip("\r"))
        return lines

    def send(self, line: str) -> None:
        self._serial.write((line + "\n").encode("ascii", "replace"))

    @property
    def connected(self) -> bool:
        return bool(self._serial.is_open)

    def take_connect(self) -> bool:
        # A serial pair has no connect event; greet once, when the link opens.
        if self._greeted:
            return False
        self._greeted = True
        return True

    def close(self) -> None:
        try:
            self._serial.close()
        except Exception:
            pass


def open_link(spec: str) -> LineLink:
    """``tcp://host:port`` / ``host:port`` / ``:port`` -> TCP; anything else a port."""
    target = spec
    for prefix in ("tcp://", "socket://"):
        if target.startswith(prefix):
            target = target[len(prefix) :]
            break
    else:
        if ":" not in target:
            return SerialLineLink(target)

    host, _, port = target.rpartition(":")
    return TcpLineLink(host or "127.0.0.1", int(port))
