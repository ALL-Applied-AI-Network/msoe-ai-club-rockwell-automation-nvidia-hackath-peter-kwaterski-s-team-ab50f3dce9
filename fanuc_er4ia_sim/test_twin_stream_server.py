"""HTTP twin stream server tests."""

import json
import socket
import tempfile
import threading
import unittest
from http.server import ThreadingHTTPServer
from pathlib import Path

from run_artifacts import atomic_write_json
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


if __name__ == "__main__":
    unittest.main()
