"""Tests for run-directory zip bundles served to remote viewers."""

import json
import tempfile
import unittest
import zipfile
from pathlib import Path

from run_artifacts import atomic_write_json
from run_bundle import BUNDLE_NAME, build_bundle, bundle_ready, ensure_bundle


class RunBundleTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.run = Path(self.temp.name) / "demo_run"
        self.run.mkdir()
        atomic_write_json(
            self.run / "detections.json",
            {"objects": [{"category": "cube", "detection_id": 0}]},
        )

    def test_not_ready_without_final_report(self):
        self.assertFalse(bundle_ready(self.run))
        with self.assertRaises(FileNotFoundError):
            build_bundle(self.run)

    def test_builds_zip_without_frame_directories(self):
        atomic_write_json(
            self.run / "execution_report.json",
            {"picks": [], "task_complete": True},
        )
        (self.run / "sort_table.mp4").write_bytes(b"mp4")
        frames = self.run / "frames"
        frames.mkdir()
        (frames / "0001.png").write_bytes(b"png")
        archive = build_bundle(self.run)
        self.assertEqual(archive.name, BUNDLE_NAME)
        with zipfile.ZipFile(archive) as bundle:
            names = set(bundle.namelist())
        self.assertIn("execution_report.json", names)
        self.assertIn("sort_table.mp4", names)
        self.assertTrue(all(not name.startswith("frames/") for name in names))

    def test_ensure_bundle_refreshes_when_artifacts_change(self):
        atomic_write_json(
            self.run / "execution_report.json",
            {"picks": [], "task_complete": True},
        )
        first = ensure_bundle(self.run)
        atomic_write_json(self.run / "step_state.json", {"phase": "done"})
        second = ensure_bundle(self.run)
        self.assertEqual(first, second)
        self.assertGreaterEqual(second.stat().st_mtime, first.stat().st_mtime)


if __name__ == "__main__":
    unittest.main()
