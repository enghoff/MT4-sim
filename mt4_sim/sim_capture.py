"""Duck-typed OpenCV capture that reads the simulated scene camera.

``mt4_vision.camera`` opens a USB device with ``cv2.VideoCapture`` and expects
``grab`` / ``read`` / ``isOpened`` / ``release``. This provides the same surface
over :class:`mt4_sim.camera_feed.SceneCameraReader`, so
``scripts/run_against_sim.py`` can patch the control repo without changing it.
"""

from __future__ import annotations

import threading
import time

import numpy as np

from mt4_sim.camera_feed import SceneCameraReader, shm_url


class SimVideoCapture:
    """Stand-in for ``cv2.VideoCapture`` backed by the sim's shared-memory feed."""

    def __init__(self, url: str = shm_url(), *, warmup_reads: int = 2) -> None:
        self._reader = SceneCameraReader(url)
        self._reader.open()
        self._opened = True
        self._frame: np.ndarray | None = None
        self._seq = 0
        # USB open warms auto-exposure with many full reads; the sim feed is
        # already lit, so a couple of reads just prove the publisher is alive.
        for _ in range(max(0, warmup_reads)):
            self.read()

    def isOpened(self) -> bool:  # noqa: N802 - OpenCV's spelling
        return self._opened and self._reader.is_opened()

    def set(self, _prop: int, _value: float) -> bool:
        return True

    def get(self, _prop: int) -> float:
        return 0.0

    def grab(self) -> bool:
        if not self.isOpened():
            return False
        try:
            meta, frame = self._reader.read()
        except RuntimeError:
            return False
        self._frame = frame
        self._seq = meta.seq
        return True

    def read(self) -> tuple[bool, np.ndarray | None]:
        if not self.grab():
            return False, None
        return True, self._frame

    def release(self) -> None:
        self._opened = False
        self._reader.close()


class SimFrameStream:
    """``mt4_vision.camera.FrameStream`` equivalent over the shared-memory feed.

    Drains the publisher at a steady rate so ``fresh()`` can wait for a frame
    that started after the call -- the same contract the USB streamer keeps.
    """

    def __init__(self, url: str = shm_url()) -> None:
        self._cap = SimVideoCapture(url, warmup_reads=1)
        self._cond = threading.Condition()
        self._frame: np.ndarray | None = None
        self._seq = 0
        self._stopped = False
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()

    def _loop(self) -> None:
        while not self._stopped:
            ok, frame = self._cap.read()
            if not ok or frame is None:
                time.sleep(0.01)
                continue
            with self._cond:
                self._frame = frame
                self._seq += 1
                self._cond.notify_all()

    def fresh(self, min_advance: int = 2, timeout_s: float = 5.0) -> np.ndarray:
        with self._cond:
            target = self._seq + min_advance
            while self._seq < target and not self._stopped:
                if not self._cond.wait(timeout=timeout_s):
                    raise RuntimeError("simulated frame stream stalled")
            if self._frame is None:
                raise RuntimeError("simulated frame stream produced no frames")
            return self._frame.copy()

    def close(self) -> None:
        self._stopped = True
        with self._cond:
            self._cond.notify_all()
        self._thread.join(timeout=2.0)
        self._cap.release()
