"""Shared-memory camera feed round-trip, no Isaac Sim required."""

from __future__ import annotations

import struct
import unittest
from multiprocessing import shared_memory

import numpy as np

from mt4_sim.camera_feed import (
    HEADER_FORMAT,
    HEADER_SIZE,
    SceneCameraReader,
    parse_shm_url,
    shm_url,
)


class ShmUrl(unittest.TestCase):
    def test_round_trips(self):
        self.assertEqual(parse_shm_url(shm_url("cam")), "cam")

    def test_rejects_other_schemes(self):
        with self.assertRaises(ValueError):
            parse_shm_url("tcp://127.0.0.1:9")


class SharedMemoryRoundTrip(unittest.TestCase):
    def test_reader_sees_published_pixels(self):
        name = "mt4_test_cam_feed"
        width, height = 16, 10
        frame_bytes = width * height * 3
        size = HEADER_SIZE + frame_bytes

        try:
            stale = shared_memory.SharedMemory(name=name)
        except FileNotFoundError:
            stale = None
        if stale is not None:
            stale.close()
            stale.unlink()

        shm = shared_memory.SharedMemory(name=name, create=True, size=size)
        try:
            pixels = np.arange(frame_bytes, dtype=np.uint8)
            shm.buf[HEADER_SIZE : HEADER_SIZE + frame_bytes] = pixels.tobytes()
            shm.buf[: struct.calcsize(HEADER_FORMAT)] = struct.pack(
                HEADER_FORMAT, 7, width, height, 3, frame_bytes
            )

            reader = SceneCameraReader(shm_url(name))
            reader.open(timeout_s=1.0)
            meta, frame = reader.read()
            reader.close()

            self.assertEqual(meta.seq, 7)
            self.assertEqual(frame.shape, (height, width, 3))
            self.assertTrue(np.array_equal(frame.reshape(-1), pixels))
        finally:
            shm.close()
            shm.unlink()


if __name__ == "__main__":
    unittest.main()
