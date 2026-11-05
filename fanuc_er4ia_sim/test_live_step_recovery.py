"""Live step recovery host helper tests."""

import json
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch

import scene_config as C

from live_step_recovery import run_live_recovery


def cube_plan():
    pick = dict(
        detection_id=0,
        category="cube",
        colour="yellow",
        stack_level=0,
        xy=[0.31, 0.12],
        yaw_deg=0.0,
        spread_dir=[0.0, 1.0, 0.0],
        place_xy=list(C.CUBE_STACK_XY),
        place_z=C.SURFACE_DZ + C.CUBE_TIP_CLEARANCE,
        pick_z=C.SURFACE_DZ + C.CUBE_TIP_CLEARANCE,
        expected_centre_z=C.SURFACE_DZ + 0.5 * C.CUBE_EDGE,
        place_spread_dir=[0.0, 1.0, 0.0],
        pose_confidence=0.95,
    )
    return {
        "source": "Cosmos3-Nano",
        "task": "cube_stack",
        "open_gap_m": 2 * C.JAW_OPEN,
        "picks": [pick],
        "stack_colours_bottom_to_top": list(C.CUBE_STACK_COLOURS),
    }


class LiveStepRecoveryTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.run = Path(self.temp.name)
        plan = cube_plan()
        (self.run / "sort_plan.json").write_text(json.dumps(plan), encoding="utf-8")
        events = [
            {
                "detection_id": 0,
                "category": "cube",
                "tracked_object": "SortCube0",
                "objects_lifted": [],
                "pickup_verified": False,
                "placement_verified": False,
                "trajectory_ik_failures": 0,
                "stack_level": 0,
            }
        ]
        (self.run / "execution_events.json").write_text(
            json.dumps(events), encoding="utf-8"
        )
        (self.run / "camera.json").write_text("{}", encoding="utf-8")

    @patch("live_step_recovery._observation", return_value=(None, None))
    @patch(
        "live_step_recovery.decide",
        return_value={
            "failure": {"failure_type": "missed_grasp", "event_index": 0},
            "action": "flip_spread",
            "plan": None,
            "skipped": ["remeasure"],
        },
    )
    def test_writes_decision_artifact(self, _decide, _observation):
        result = run_live_recovery(self.run)
        self.assertEqual(result["action"], "flip_spread")
        self.assertTrue((self.run / "live_recovery_decision.json").is_file())


if __name__ == "__main__":
    unittest.main()
