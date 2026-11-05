import json
import os
import tempfile
import unittest
from pathlib import Path

os.environ["REAL_ROBOT_ENABLE_COSMOS"] = "0"

from screw_policy.artifacts import RolloutLogger
from screw_policy.metrics import aggregate_rollouts, finalize_act_latencies


class TestMetrics(unittest.TestCase):
    def test_finalize_latencies(self):
        out = finalize_act_latencies([1.0, 2.0, 100.0])
        self.assertAlmostEqual(out["mean"], 34.333, places=2)
        self.assertEqual(out["max"], 100.0)

    def test_rollout_logger_writes_metrics(self):
        with tempfile.TemporaryDirectory() as tmp:
            logger = RolloutLogger(root=Path(tmp))
            logger.start_rollout(
                {"trial_id": "t-test", "status": "running"},
                expected_screws=8,
                cosmos_enabled=False,
            )
            logger.log_step(
                {"phase": "HOVER", "event": "hover", "drops": 0},
                (1, 7),
                act_ms=3.5,
                cosmos_ms=None,
                tcp_row=[400, 0, -280, -180, 0, 0, 0],
            )
            path = logger.finalize(
                end_reason="unit_test",
                task_complete=False,
                screws_dropped=2,
            )
            self.assertIsNotNone(path)
            data = json.loads(path.read_text())
            self.assertEqual(data["screws_dropped"], 2)
            self.assertEqual(data["total_act_calls"], 1)
            self.assertTrue(path.parent.joinpath("PRESENTATION.md").is_file())
            agg = aggregate_rollouts([path])
            self.assertEqual(agg["rollout_count"], 1)


if __name__ == "__main__":
    unittest.main()
