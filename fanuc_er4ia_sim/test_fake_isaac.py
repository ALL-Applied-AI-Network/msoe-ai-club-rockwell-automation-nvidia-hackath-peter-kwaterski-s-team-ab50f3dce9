"""Integration test for the recovery runner and simplified fake Isaac adapter."""

import json
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest

from fake_isaac.make_fake_run import make_run
from recovery_policy import run_recovery

SIM = Path(__file__).resolve().parent
LAUNCHER = SIM / "fake_isaac" / "launch.py"
SORT_SCENE = SIM / "sort_scene.py"


class FakeIsaacIntegrationTests(unittest.TestCase):
    def test_bad_pick_is_remeasured_and_replayed_to_completion(self):
        with tempfile.TemporaryDirectory() as temp_dir:
            run = make_run(Path(temp_dir) / "baseline", fault=(2, 2, 0, 0))
            calls = []

            def execute(run_dir):
                run_dir = Path(run_dir)
                calls.append(run_dir)
                result = subprocess.run(
                    [
                        sys.executable,
                        str(LAUNCHER),
                        str(SORT_SCENE),
                        "--run",
                        str(run_dir),
                        "--physics-only",
                    ],
                    cwd=SIM,
                    capture_output=True,
                    text=True,
                    check=False,
                )
                (run_dir / "sort.log").write_text(
                    result.stdout + result.stderr, encoding="utf-8"
                )
                return result.returncode

            result = run_recovery(run, simulator=execute)

            self.assertTrue(result["final_complete"], result)
            self.assertEqual(len(calls), 2)
            self.assertEqual(result["steps"][0]["failure"]["failure_type"], "missed_grasp")
            self.assertEqual(result["steps"][0]["action"], "remeasure")
            self.assertTrue(
                (Path(f"{run}_retry1") / "execution_report.json").is_file()
            )
            baseline_plan = json.loads(
                (run / "sort_plan.json").read_text(encoding="utf-8")
            )
            self.assertNotIn("recovery_history", baseline_plan)


if __name__ == "__main__":
    unittest.main()
