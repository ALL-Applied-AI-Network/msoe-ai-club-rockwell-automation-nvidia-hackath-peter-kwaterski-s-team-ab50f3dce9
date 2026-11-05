"""HTTP twin stream server tests."""

import json
import socket
import tempfile
import threading
import unittest
from http.server import ThreadingHTTPServer
from pathlib import Path

from run_artifacts import atomic_write_json
from run_bundle import mark_run_finalized
from twin_stream_server import TwinStreamHandler


class TwinStreamServerTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.run = Path(self.temp.name) / "live_run"
        self.run.mkdir()
        atomic_write_json(
            self.run / "detections.json",
            {"objects": [{"category": "cube", "detection_id": 0}]},
        )
        sock = socket.socket()
        sock.bind(("127.0.0.1", 0))
        self.port = sock.getsockname()[1]
        sock.close()
        TwinStreamHandler.run_dir = self.run
        self.server = ThreadingHTTPServer(("127.0.0.1", self.port), TwinStreamHandler)
        self.thread = threading.Thread(target=self.server.serve_forever, daemon=True)
        self.thread.start()

    def tearDown(self):
        self.server.shutdown()
        self.server.server_close()

    def test_health_and_step_state(self):
        import urllib.request

        with urllib.request.urlopen(f"http://127.0.0.1:{self.port}/health") as resp:
            payload = json.loads(resp.read().decode("utf-8"))
        self.assertTrue(payload["ok"])
        atomic_write_json(self.run / "step_state.json", {"phase": "executing"})
        with urllib.request.urlopen(
            f"http://127.0.0.1:{self.port}/step_state"
        ) as resp:
            state = json.loads(resp.read().decode("utf-8"))
        self.assertEqual(state["phase"], "executing")

    def test_bundle_zip_after_final_report(self):
        import urllib.request

        with urllib.request.urlopen(f"http://127.0.0.1:{self.port}/status") as resp:
            payload = json.loads(resp.read().decode("utf-8"))
        self.assertIsNone(payload.get("bundle_url"))
        atomic_write_json(
            self.run / "execution_report.json",
            {
                "picks": [
                    {
                        "detection_id": 0,
                        "category": "cube",
                        "tracked_object": "SortCube0",
                        "pickup_verified": True,
                        "placement_verified": False,
                    }
                ],
                "task_complete": False,
                "abort_reason": "test abort",
            },
        )
        (self.run / "review.txt").write_text("artifacts", encoding="utf-8")
        with urllib.request.urlopen(f"http://127.0.0.1:{self.port}/status") as resp:
            payload = json.loads(resp.read().decode("utf-8"))
        self.assertIsNone(payload.get("bundle_url"))
        (self.run / "sort_table.mp4").write_bytes(b"mp4-table")
        (self.run / "sort_side.mp4").write_bytes(b"mp4-side")
        mark_run_finalized(self.run)
        with urllib.request.urlopen(f"http://127.0.0.1:{self.port}/status") as resp:
            payload = json.loads(resp.read().decode("utf-8"))
        self.assertEqual(payload["bundle_url"], "/bundle.zip")
        with urllib.request.urlopen(f"http://127.0.0.1:{self.port}/bundle.zip") as resp:
            body = resp.read()
        self.assertTrue(body.startswith(b"PK"))
        self.assertTrue((self.run / "run_bundle.zip").is_file())


if __name__ == "__main__":
    unittest.main()
