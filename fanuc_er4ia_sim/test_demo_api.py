"""Tests for the browser demo control API without Isaac Sim or Cosmos."""

import json
import tempfile
import threading
import time
import unittest
import urllib.error
import urllib.request
from pathlib import Path
from unittest.mock import patch

from demo_api import DemoAPIHandler, RunManager, create_server


FAKE_CUBE_PIPELINE = """#!/usr/bin/env bash
set -euo pipefail
run=$1
printf '%s' "${INJECT_MISSED_GRASP:-}" > "$run/injected.txt"
printf '%s' '{"picks": [{"detection_id": 0, "category": "cube", "tracked_object": "SortCube0", "pickup_verified": true, "placement_verified": true}], "task_complete": true, "abort_reason": null}' > "$run/execution_report.json"
printf 'fake-video' > "$run/sort_table.mp4"
printf 'fake-video' > "$run/sort_side.mp4"
"""

FAKE_LEGO_PIPELINE = """#!/usr/bin/env bash
set -euo pipefail
run=$1
printf '%s' '{"task_complete": true, "verified_pickups": 10, "verified_placements": 10, "connector_locks": 10, "ik_failures": 0, "events": []}' > "$run/execution_report.json"
"""


class DemoAPITests(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory()
        self.addCleanup(self.temporary.cleanup)
        root = Path(self.temporary.name)
        self.sim = root / "sim"
        self.runs = root / "runs"
        self.sim.mkdir()
        (self.sim / "run_step_pipeline.sh").write_text(FAKE_CUBE_PIPELINE, encoding="utf-8")
        (self.sim / "run_lego_pipeline.sh").write_text(FAKE_LEGO_PIPELINE, encoding="utf-8")

    @staticmethod
    def wait_for(manager, expected="succeeded"):
        deadline = time.monotonic() + 3
        while time.monotonic() < deadline:
            if manager.status()["process"]["state"] == expected:
                return manager.status()
            time.sleep(0.01)
        raise AssertionError(f"manager did not reach {expected}")

    def test_cube_fault_is_validated_and_passed_without_shell_interpolation(self):
        manager = RunManager(self.sim, self.runs)
        started = manager.start({"task_type": "cube_stack", "fault": "drop", "pick_index": 2})
        status = self.wait_for(manager)
        run = self.runs / started["run_id"]
        self.assertEqual((run / "injected.txt").read_text(), "2:drop")
        self.assertEqual(status["media"]["videos"][0]["name"], "sort_table.mp4")
        self.assertEqual(status["fault"], "drop")

    def test_lego_is_undiscoverable_and_rejected_by_default(self):
        manager = RunManager(self.sim, self.runs)
        self.assertEqual([task["id"] for task in manager.capabilities()["tasks"]], ["cube_stack"])
        with self.assertRaisesRegex(ValueError, "Unknown task_type"):
            manager.start({"task_type": "lego_assembly", "fault": "none", "pick_index": 0})

    def test_video_requires_success_report_and_every_expected_render(self):
        manager = RunManager(self.sim, self.runs)
        run = self.runs / "manual_gate_test"
        run.mkdir()
        manager.run_dir = run
        manager.metadata = {
            "run_id": run.name,
            "task_type": "cube_stack",
            "task_label": "Cube stacking",
            "state": "running",
        }
        (run / "execution_report.json").write_text('{"task_complete": true}', encoding="utf-8")
        (run / "sort_table.mp4").write_bytes(b"table")

        self.assertEqual(manager.ready_video_names(), [])
        self.assertIsNone(manager.video_path("sort_table.mp4"))
        self.assertEqual(manager.status()["media"]["render_state"], "rendering")

        manager.metadata["state"] = "succeeded"
        self.assertEqual(manager.ready_video_names(), [])
        self.assertEqual(manager.status()["media"]["render_state"], "failed")

        (run / "sort_side.mp4").write_bytes(b"side")
        self.assertEqual(manager.ready_video_names(), ["sort_table.mp4", "sort_side.mp4"])
        self.assertEqual(manager.status()["media"]["render_state"], "ready")
        self.assertEqual(manager.video_path("sort_table.mp4"), run / "sort_table.mp4")

    def test_nominal_run_clears_operator_fault_environment(self):
        manager = RunManager(self.sim, self.runs)
        with patch.dict("os.environ", {"INJECT_MISSED_GRASP": "5:drop"}):
            started = manager.start({"task_type": "cube_stack", "fault": "none", "pick_index": 0})
        self.wait_for(manager)
        self.assertEqual((self.runs / started["run_id"] / "injected.txt").read_text(), "")

    def test_explicit_flag_reveals_experimental_task(self):
        manager = RunManager(self.sim, self.runs, enable_experimental=True)
        self.assertEqual(
            [task["id"] for task in manager.capabilities()["tasks"]],
            ["cube_stack", "lego_assembly"],
        )

    def test_abandoned_video_read_does_not_raise_broken_pipe(self):
        video = Path(self.temporary.name) / "review.mp4"
        video.write_bytes(b"video-bytes")

        class ClosedClient:
            def write(self, _block):
                raise BrokenPipeError("browser closed speculative request")

        handler = object.__new__(DemoAPIHandler)
        handler.headers = {}
        handler.wfile = ClosedClient()
        handler.send_response = lambda _status: None
        handler.send_header = lambda _name, _value: None
        handler.end_headers = lambda: None

        handler._file(video)

    def test_http_api_starts_run_and_rejects_hidden_task(self):
        server = create_server(self.sim, self.runs, "127.0.0.1", 0)
        thread = threading.Thread(target=server.serve_forever, daemon=True)
        thread.start()
        self.addCleanup(server.server_close)
        self.addCleanup(server.shutdown)
        base = f"http://127.0.0.1:{server.server_address[1]}"

        with urllib.request.urlopen(f"{base}/api/capabilities") as response:
            capabilities = json.loads(response.read())
        self.assertEqual([task["id"] for task in capabilities["tasks"]], ["cube_stack"])

        hidden = urllib.request.Request(
            f"{base}/api/runs",
            data=json.dumps({"task_type": "lego_assembly", "fault": "none", "pick_index": 0}).encode(),
            headers={"Content-Type": "application/json"},
            method="POST",
        )
        with self.assertRaises(urllib.error.HTTPError) as caught:
            urllib.request.urlopen(hidden)
        self.assertEqual(caught.exception.code, 400)
        caught.exception.close()

        launch = urllib.request.Request(
            f"{base}/api/runs",
            data=json.dumps({"task_type": "cube_stack", "fault": "none", "pick_index": 0}).encode(),
            headers={"Content-Type": "application/json"},
            method="POST",
        )
        with urllib.request.urlopen(launch) as response:
            self.assertEqual(response.status, 202)


if __name__ == "__main__":
    unittest.main()
