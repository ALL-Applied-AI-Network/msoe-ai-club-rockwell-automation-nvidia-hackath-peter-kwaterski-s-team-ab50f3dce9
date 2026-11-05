"""Tests for run-directory zip bundles served to remote viewers."""

import json
import tempfile
import unittest
import zipfile
from pathlib import Path

from run_artifacts import atomic_write_json
from run_bundle import (
    BUNDLE_NAME,
    FINALIZED_MARKER,
    build_bundle,
    bundle_ready,
    ensure_bundle,
    mark_run_finalized,
)


def _finalize_run(run: Path, *, report=None):
    atomic_write_json(
        run / "execution_report.json",
        report if report is not None else {"picks": [], "task_complete": True},
    )
    (run / "sort_table.mp4").write_bytes(b"mp4-table")
    (run / "sort_side.mp4").write_bytes(b"mp4-side")
    mark_run_finalized(run)


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

    def test_not_ready_with_report_only(self):
        atomic_write_json(
            self.run / "execution_report.json",
            {"picks": [], "task_complete": True},
        )
        self.assertFalse(bundle_ready(self.run))
        with self.assertRaises(FileNotFoundError):
            build_bundle(self.run)

    def test_builds_zip_without_frame_directories(self):
        _finalize_run(self.run)
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
        _finalize_run(self.run)
        first = ensure_bundle(self.run)
        atomic_write_json(self.run / "step_state.json", {"phase": "done"})
        second = ensure_bundle(self.run)
        self.assertEqual(first, second)
        self.assertGreaterEqual(second.stat().st_mtime, first.stat().st_mtime)

    def test_mark_finalized_writes_marker(self):
        atomic_write_json(
            self.run / "execution_report.json",
            {"picks": [], "task_complete": True},
        )
        (self.run / "sort_table.mp4").write_bytes(b"a")
        (self.run / "sort_side.mp4").write_bytes(b"b")
        mark_run_finalized(self.run)
        self.assertTrue((self.run / FINALIZED_MARKER).is_file())
        self.assertTrue(bundle_ready(self.run))


    def test_prepare_run_download(self):
        _finalize_run(self.run, report={"picks": [], "task_complete": True})
        (self.run / "extra.txt").write_text("x", encoding="utf-8")
        from run_bundle import prepare_run_download

        archive = prepare_run_download(self.run)
        self.assertEqual(archive.name, BUNDLE_NAME)
        self.assertTrue(bundle_ready(self.run))


if __name__ == "__main__":
    unittest.main()
