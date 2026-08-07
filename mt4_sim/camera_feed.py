"""Publish the scene camera as a cross-process BGR frame stream.

The Isaac process renders ``/World/SceneCamera``; the control process reads the
same frames through ``MT4_CAMERA_URL=shm://...`` (see ``mt4_vision.camera``) or
the duck-typed capture in :mod:`mt4_sim.sim_capture`.

Layout of the shared block (kept in lockstep with ``mt4_vision.sim_feed``)::

    offset  0  uint64  sequence number (monotone; written last)
    offset  8  uint32  width
    offset 12  uint32  height
    offset 16  uint32  channels (always 3 = BGR)
    offset 20  uint32  frame bytes
    offset 64  uint8[] BGR pixels

A reader copies the frame, then re-checks the sequence number; a mismatch means
a publisher write overlapped the copy and the read is retried.
"""

from __future__ import annotations

import struct
import time
from dataclasses import dataclass
from multiprocessing import shared_memory
from typing import BinaryIO

import numpy as np

DEFAULT_SHM_NAME = "mt4_scene_cam"
HEADER_FORMAT = "<QIIII"
HEADER_SIZE = 64
assert struct.calcsize(HEADER_FORMAT) <= HEADER_SIZE


def shm_url(name: str = DEFAULT_SHM_NAME) -> str:
    return f"shm://{name}"


def parse_shm_url(url: str) -> str:
    if not url.startswith("shm://"):
        raise ValueError(f"camera feed URL must look like shm://name (got {url!r})")
    name = url[len("shm://") :].strip()
    if not name:
        raise ValueError(f"empty shared-memory name in {url!r}")
    return name


@dataclass
class FrameMeta:
    seq: int
    width: int
    height: int
    channels: int
    nbytes: int


def _rgb_to_bgr(rgb) -> np.ndarray | None:
    """Convert an Isaac ``get_rgb`` result to contiguous HxWx3 BGR, or None if not ready."""
    import cv2

    if rgb is None:
        return None
    arr = np.asarray(rgb)
    if arr.size == 0 or arr.ndim < 2:
        return None
    # Annotator sometimes returns a flat / wrong-shaped buffer during warm-up.
    if arr.ndim == 2:
        return None
    if arr.shape[-1] < 3:
        return None
    bgr = cv2.cvtColor(np.asarray(arr[..., :3], dtype=np.uint8), cv2.COLOR_RGB2BGR)
    if not bgr.flags["C_CONTIGUOUS"]:
        bgr = np.ascontiguousarray(bgr)
    return bgr


class SceneCameraPublisher:
    """Owns the Isaac sensor and the shared-memory block it writes into."""

    def __init__(
        self,
        *,
        prim_path: str = "/World/SceneCamera",
        resolution: tuple[int, int] = (1280, 720),
        shm_name: str = DEFAULT_SHM_NAME,
        warm_frames: int = 40,
    ) -> None:
        from isaacsim.sensors.camera import Camera

        self._resolution = (int(resolution[0]), int(resolution[1]))
        self._name = shm_name
        self._seq = 0
        width, height = self._resolution
        self._frame_bytes = width * height * 3
        self._size = HEADER_SIZE + self._frame_bytes

        try:
            stale = shared_memory.SharedMemory(name=self._name)
        except FileNotFoundError:
            stale = None
        if stale is not None:
            stale.close()
            stale.unlink()

        self._shm = shared_memory.SharedMemory(name=self._name, create=True, size=self._size)
        self._buf = self._shm.buf
        self._write_header(0, width, height, 3, self._frame_bytes)

        self._camera = Camera(prim_path=prim_path, resolution=self._resolution)
        self._camera.initialize()
        self._warm_frames = warm_frames
        self._warmed = False

    @property
    def url(self) -> str:
        return shm_url(self._name)

    @property
    def resolution(self) -> tuple[int, int]:
        return self._resolution

    def warm(self, world) -> None:
        """Spend the frames RTX needs before the first image is trustworthy.

        Matches ``scripts/check.py``: render first, *then* read. Publishing
        during the warm-up window fails because the RGB annotator is still None.
        """
        for _ in range(self._warm_frames):
            world.step(render=True)

        # A few more tries once the annotator should be live; headless can lag.
        for _ in range(max(60, self._warm_frames)):
            if self.try_publish() is not None:
                self._warmed = True
                return
            world.step(render=True)

        raise RuntimeError(
            "scene camera never produced a frame after warm-up; "
            "is /World/SceneCamera present and the RTX renderer alive?"
        )

    def try_publish(self) -> FrameMeta | None:
        """Publish the latest frame if the annotator has one; else return None."""
        bgr = _rgb_to_bgr(self._camera.get_rgb())
        if bgr is None:
            return None
        if bgr.shape[1] != self._resolution[0] or bgr.shape[0] != self._resolution[1]:
            import cv2

            bgr = cv2.resize(bgr, self._resolution, interpolation=cv2.INTER_AREA)
            if not bgr.flags["C_CONTIGUOUS"]:
                bgr = np.ascontiguousarray(bgr)
        if bgr.nbytes != self._frame_bytes:
            return None

        self._seq += 1
        # seq written last so a reader that sees the new seq also sees the pixels.
        dst = np.ndarray(
            self._frame_bytes, dtype=np.uint8, buffer=self._shm.buf, offset=HEADER_SIZE
        )
        dst[:] = bgr.reshape(-1)
        self._write_header(
            self._seq, self._resolution[0], self._resolution[1], 3, self._frame_bytes
        )
        return FrameMeta(self._seq, *self._resolution, 3, self._frame_bytes)

    def publish(self) -> FrameMeta:
        meta = self.try_publish()
        if meta is None:
            raise RuntimeError("scene camera returned no frame")
        return meta

    def _write_header(
        self, seq: int, width: int, height: int, channels: int, nbytes: int
    ) -> None:
        header = struct.pack(HEADER_FORMAT, seq, width, height, channels, nbytes)
        self._buf[: len(header)] = header

    def close(self) -> None:
        try:
            self._shm.close()
            self._shm.unlink()
        except FileNotFoundError:
            pass


class SceneCameraReader:
    """Attach to a publisher's shared-memory block and pull BGR frames."""

    def __init__(self, url: str = shm_url()) -> None:
        self._name = parse_shm_url(url)
        self._shm: shared_memory.SharedMemory | None = None
        self._opened = False

    @property
    def url(self) -> str:
        return shm_url(self._name)

    def open(self, *, timeout_s: float = 30.0) -> None:
        deadline = time.monotonic() + timeout_s
        last_error: Exception | None = None
        while time.monotonic() < deadline:
            try:
                self._shm = shared_memory.SharedMemory(name=self._name)
                self._opened = True
                return
            except FileNotFoundError as exc:
                last_error = exc
                time.sleep(0.05)
        raise TimeoutError(
            f"no camera feed at {self.url} after {timeout_s:.0f}s "
            f"(is serve_firmware.py --camera running?)"
        ) from last_error

    def is_opened(self) -> bool:
        return self._opened and self._shm is not None

    def read_meta(self) -> FrameMeta:
        assert self._shm is not None
        seq, width, height, channels, nbytes = struct.unpack_from(
            HEADER_FORMAT, self._shm.buf, 0
        )
        return FrameMeta(seq, width, height, channels, nbytes)

    def read(self, *, retries: int = 8) -> tuple[FrameMeta, np.ndarray]:
        """Return ``(meta, bgr)`` for a consistent snapshot."""
        if self._shm is None:
            raise RuntimeError("camera feed is not open")
        last: Exception | None = None
        for _ in range(retries):
            try:
                meta = self.read_meta()
                if meta.seq == 0 or meta.nbytes <= 0:
                    time.sleep(0.01)
                    continue
                start = HEADER_SIZE
                end = start + meta.nbytes
                raw = bytes(self._shm.buf[start:end])
                again = self.read_meta()
                if again.seq != meta.seq or again.nbytes != meta.nbytes:
                    continue
                frame = np.frombuffer(raw, dtype=np.uint8).reshape(
                    (meta.height, meta.width, meta.channels)
                )
                return again, frame.copy()
            except Exception as exc:  # noqa: BLE001 - reshape/size races
                last = exc
                time.sleep(0.0)
        raise RuntimeError(f"could not read a consistent camera frame from {self.url}") from last

    def close(self) -> None:
        self._opened = False
        if self._shm is not None:
            self._shm.close()
            self._shm = None


def save_preview(path: str | BinaryIO, frame: np.ndarray) -> None:
    import cv2

    cv2.imwrite(str(path), frame)
