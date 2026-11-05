"""Replay orchestration tests using a deterministic fake simulator callback."""

import json
from pathlib import Path
import tempfile
import unittest

import scene_config as C
from recovery_policy import run_recovery


def valid_plan():
    return {
        "source": "Cosmos3-Nano",
        "task": "sort",
        "open_gap_m": 2 * C.JAW_OPEN,
        "picks": [
            {
                "detection_id": 0,
                "category": "cube",
                "xy": [0.31, 0.12],
                "pick_z": 0.01,
                "place_xy": [0.4, 0.1],
                "place_z": 0.01,
                "yaw_deg": 25.0,
                "spread_dir": [-0.4226182617, 0.9063077870, 0.0],
                "place_spread_dir": [0.0, 1.0, 0.0],
            }
        ],
    }


class RecoveryRunnerTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.run = Path(self.temp.name) / "baseline"
        self.run.mkdir()
        (self.run / "sort_plan.json").write_text(
            json.dumps(valid_plan()), encoding="utf-8"
        )
        for name in ("scene_state.json", "ground_truth.json", "detections.json"):
            (self.run / name).write_text("{}", encoding="utf-8")

    def fake_simulator(self, succeed_after_retry=True, return_code=0):
        calls = []

        def execute(run_dir):
            run_dir = Path(run_dir)
            calls.append(run_dir)
            plan = json.loads((run_dir / "sort_plan.json").read_text(encoding="utf-8"))
            recovered = bool(plan.get("recovery_history"))
            succeeded = recovered and succeed_after_retry
            events = [
                {
                    "detection_id": 0,
                    "tracked_object": "SortCube0",
                    "objects_lifted": ["SortCube0"] if succeeded else [],
                    "pickup_verified": succeeded,
                    "placement_verified": succeeded,
                    "stack_support_verified": True,
                    "trajectory_ik_failures": 0,
                }
            ]
            (run_dir / "execution_events.json").write_text(
                json.dumps(events), encoding="utf-8"
            )
            (run_dir / "execution_report.json").write_text(
                json.dumps({"task_complete": succeeded}), encoding="utf-8"
            )
            return return_code

        return calls, execute

    def test_failed_pickup_replays_in_new_folder_and_records_success(self):
        calls, simulator = self.fake_simulator()

        log = run_recovery(self.run, simulator=simulator)

        retry = Path(f"{self.run}_retry1")
        self.assertEqual(calls, [self.run.resolve(), retry.resolve()])
        self.assertTrue(log["final_complete"])
        self.assertEqual(log["retries"], 1)
        retry_plan = json.loads((retry / "sort_plan.json").read_text(encoding="utf-8"))
        self.assertEqual(retry_plan["recovery_history"][0]["action"], "flip_spread")
        self.assertTrue((retry / "scene_state.json").is_file())
        retry_report = json.loads(
            (retry / "execution_report.json").read_text(encoding="utf-8")
        )
        self.assertTrue(retry_report["task_complete"])
        original_plan = json.loads((self.run / "sort_plan.json").read_text(encoding="utf-8"))
        self.assertNotIn("recovery_history", original_plan)
        saved_log = json.loads((self.run / "recovery_log.json").read_text(encoding="utf-8"))
        self.assertTrue(saved_log["final_complete"])

    def test_retry_limit_stops_without_an_unbounded_loop(self):
        calls, simulator = self.fake_simulator(succeed_after_retry=False)

        log = run_recovery(self.run, simulator=simulator, max_retries=2)

        self.assertEqual(len(calls), 3)
        self.assertEqual(log["retries"], 2)
        self.assertFalse(log["final_complete"])
        self.assertEqual(log["steps"][-1]["action"], "stop")
        second_retry = Path(f"{self.run}_retry2")
        second_plan = json.loads(
            (second_retry / "sort_plan.json").read_text(encoding="utf-8")
        )
        self.assertEqual(second_plan["recovery_history"][-1]["action"], "nudge")
        self.assertFalse(Path(f"{self.run}_retry3").exists())

    def test_placement_failure_stops_without_creating_retry_folder(self):
        def execute(run_dir):
            events = [
                {
                    "detection_id": 0,
                    "tracked_object": "SortCube0",
                    "objects_lifted": ["SortCube0"],
                    "pickup_verified": True,
                    "placement_verified": False,
                }
            ]
            (Path(run_dir) / "execution_events.json").write_text(
                json.dumps(events), encoding="utf-8"
            )
            (Path(run_dir) / "execution_report.json").write_text(
                '{"task_complete": false}', encoding="utf-8"
            )
            return 0

        log = run_recovery(self.run, simulator=execute)

        self.assertFalse(log["final_complete"])
        self.assertEqual(log["steps"][0]["failure"]["failure_type"], "placement_failed")
        self.assertFalse(Path(f"{self.run}_retry1").exists())

    def test_never_overwrites_an_existing_retry_directory(self):
        Path(f"{self.run}_retry1").mkdir()
        _, simulator = self.fake_simulator()

        with self.assertRaises(FileExistsError):
            run_recovery(self.run, simulator=simulator)


if __name__ == "__main__":
    unittest.main()
