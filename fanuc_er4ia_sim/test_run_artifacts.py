"""Atomic artifact writer tests."""

import json
import tempfile
import unittest
from pathlib import Path

from run_artifacts import atomic_write_json, read_json


class RunArtifactsTests(unittest.TestCase):
    def test_atomic_json_round_trip(self):
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "nested" / "step_state.json"
            atomic_write_json(path, {"phase": "ready"})
            self.assertEqual(read_json(path)["phase"], "ready")
            atomic_write_json(path, {"phase": "executing"})
            self.assertEqual(read_json(path)["phase"], "executing")


if __name__ == "__main__":
    unittest.main()
