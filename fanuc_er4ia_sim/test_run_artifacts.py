"""Atomic artifact writer tests."""

import json
import tempfile
import unittest
from pathlib import Path

import numpy as np

from run_artifacts import LiveFramePublisher, atomic_write_json, read_json


class RunArtifactsTests(unittest.TestCase):
    def test_atomic_json_round_trip(self):
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "nested" / "step_state.json"
            atomic_write_json(path, {"phase": "ready"})
            self.assertEqual(read_json(path)["phase"], "ready")
            atomic_write_json(path, {"phase": "executing"})
            self.assertEqual(read_json(path)["phase"], "executing")

    def test_live_frame_uses_png_encoder_and_replaces_atomically(self):
        with tempfile.TemporaryDirectory() as tmp:
            run = Path(tmp)
            image = np.zeros((8, 8, 3), dtype=np.uint8)
            image[:, :, 1] = 255

            publisher = LiveFramePublisher(run, min_interval_s=0)
            publisher.maybe_publish(image)

            frame = run / "live_frame.png"
            self.assertTrue(frame.is_file())
            self.assertTrue(frame.read_bytes().startswith(b"\x89PNG\r\n\x1a\n"))
            self.assertEqual(list(run.glob(".live_frame.*.png")), [])


if __name__ == "__main__":
    unittest.main()
