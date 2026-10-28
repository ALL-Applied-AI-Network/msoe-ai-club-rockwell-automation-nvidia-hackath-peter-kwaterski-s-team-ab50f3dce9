"""Tests for recovery_policy.py using JSON fixtures (no Isaac, no GPU)."""

import copy
import json
import unittest
from pathlib import Path

import cv2
import numpy as np

import scene_config as C
from cube_pose import silhouette
from recovery_policy import (MAX_RETRIES, apply_action, classify, decide,
                             next_action, recovery_rate, square_yaw)
from sort_scene import validate_plan
from test_support import camera_model

FIXTURES = Path(__file__).resolve().parent / "fixtures" / "recovery"
COMPLETE = {"task_complete": True}
INCOMPLETE = {"task_complete": False}


def load(name):
    return json.loads((FIXTURES / name).read_text())


class ClassifyTest(unittest.TestCase):
    """Part 1: find the first failure and name it."""

    def setUp(self):
        self.ok = load("success_events.json")
        self.plan = load("sort_plan.json")

    def failed_at(self, i, **changes):
        events = copy.deepcopy(self.ok[: i + 1])
        events[i].update(changes)
        return classify(events, self.plan, INCOMPLETE)

    def test_success(self):
        self.assertEqual(classify(self.ok, self.plan, COMPLETE)["failure_type"], "none")

    def test_missed_grasp_fixture(self):
        f = classify(load("missed_grasp_events.json"), self.plan, INCOMPLETE)
        self.assertEqual((f["stage"], f["event_index"], f["failure_type"]),
                         ("pickup", 2, "missed_grasp"))
        self.assertEqual(f["detection_id"], 4)  # list position 2 is cube id 4

    def test_wrong_object(self):
        f = self.failed_at(3, pickup_verified=False, objects_lifted=["SortCap0"])
        self.assertEqual((f["event_index"], f["failure_type"]), (3, "wrong_object"))

    def test_two_objects_lifted_is_wrong_object(self):
        f = self.failed_at(1, objects_lifted=["SortCube1", "SortCube2"])
        self.assertEqual(f["failure_type"], "wrong_object")

    def test_placement_failed(self):
        f = self.failed_at(4, placement_verified=False)
        self.assertEqual((f["stage"], f["failure_type"]), ("placement", "placement_failed"))

    def test_support_failed(self):
        f = self.failed_at(5, stack_support_verified=False)
        self.assertEqual(f["failure_type"], "support_failed")

    def test_untracked_and_ik(self):
        self.assertEqual(self.failed_at(0, tracked_object=None)["failure_type"], "untracked_target")
        self.assertEqual(self.failed_at(0, trajectory_ik_failures=2)["failure_type"], "ik_failure")

    def test_first_failure_wins(self):
        events = copy.deepcopy(self.ok)
        events[1]["pickup_verified"] = False
        events[1]["objects_lifted"] = []
        events[4]["placement_verified"] = False
        self.assertEqual(classify(events, self.plan, INCOMPLETE)["event_index"], 1)

    def test_crash_without_report(self):
        f = classify(self.ok[:3], self.plan, None)
        self.assertEqual((f["failure_type"], f["event_index"]), ("execution_crash", 3))

    def test_all_events_fine_but_report_incomplete(self):
        self.assertEqual(classify(self.ok, self.plan, INCOMPLETE)["failure_type"], "incomplete")


class ActionTest(unittest.TestCase):
    """Part 2: bounded choice of action and corrected plans the executor accepts."""

    def setUp(self):
        self.plan = load("sort_plan.json")
        self.miss = classify(load("missed_grasp_events.json"), self.plan, INCOMPLETE)

    def test_ladder_is_bounded(self):
        actions = [next_action(self.miss, n) for n in range(MAX_RETRIES + 1)]
        self.assertEqual(actions, ["remeasure", "flip_spread", "nudge", "stop"])

    def test_unsafe_failures_are_not_retried(self):
        for kind in ("placement_failed", "support_failed", "wrong_object", "execution_crash"):
            self.assertEqual(next_action({"failure_type": kind}, 0), "stop")
        self.assertEqual(next_action({"failure_type": "none"}, 0), "done")

    def test_flip_turns_jaws_90_degrees_and_stays_valid(self):
        new = apply_action(self.plan, self.miss, "flip_spread")
        validate_plan(new)
        old_s = np.array(self.plan["picks"][2]["spread_dir"])
        new_s = np.array(new["picks"][2]["spread_dir"])
        self.assertAlmostEqual(float(old_s @ new_s), 0.0)  # perpendicular: other two faces

    def test_nudge_moves_2mm_and_stays_valid(self):
        new = apply_action(self.plan, self.miss, "nudge")
        validate_plan(new)
        moved = np.subtract(new["picks"][2]["xy"], self.plan["picks"][2]["xy"])
        self.assertAlmostEqual(float(np.linalg.norm(moved)), 0.002)

    def test_only_the_failed_pick_changes(self):
        new = apply_action(self.plan, self.miss, "nudge")
        for i, (a, b) in enumerate(zip(self.plan["picks"], new["picks"])):
            if i != 2:
                self.assertEqual(a, b)

    def test_pure_inputs_unchanged(self):
        before = copy.deepcopy(self.plan)
        apply_action(self.plan, self.miss, "flip_spread")
        decide(load("missed_grasp_events.json"), self.plan, INCOMPLETE)
        self.assertEqual(self.plan, before)

    def test_remeasure_uses_the_image(self):
        cam = camera_model(C.TABLE_CAM)
        pick = self.plan["picks"][2]
        true_yaw = square_yaw(pick["yaw_deg"] + 25)  # plan was 25 degrees off
        img = np.full((cam.H, cam.W, 3), 200, np.uint8)
        img[silhouette(cam, np.array(pick["xy"]), true_yaw, (cam.H, cam.W)) > 0] = (165, 65, 35)
        new = apply_action(self.plan, self.miss, "remeasure", img, cam)
        validate_plan(new)
        self.assertLess(abs(square_yaw(new["picks"][2]["yaw_deg"] - true_yaw)), 3)

    def test_remeasure_without_cube_in_image_is_skipped(self):
        cam = camera_model(C.TABLE_CAM)
        blank = np.full((cam.H, cam.W, 3), 200, np.uint8)
        self.assertIsNone(apply_action(self.plan, self.miss, "remeasure", blank, cam))
        result = decide(load("missed_grasp_events.json"), self.plan, INCOMPLETE, blank, cam)
        self.assertEqual((result["action"], result["skipped"]), ("flip_spread", ["remeasure"]))
        self.assertEqual(result["plan"]["recovery_rungs_used"], 2)

    def test_decide_walks_down_the_ladder_then_stops(self):
        plan, seen = self.plan, []
        events = load("missed_grasp_events.json")
        while True:
            r = decide(events, plan, INCOMPLETE)  # no image: remeasure is skipped
            seen.append(r["action"])
            if r["plan"] is None:
                break
            plan = r["plan"]
        self.assertEqual(seen, ["flip_spread", "nudge", "stop"])
        self.assertEqual(len(plan["recovery_history"]), 2)

    def test_recovery_rate(self):
        miss = {"failure_type": "missed_grasp"}
        logs = [{"first_failure": miss, "final_complete": True},
                {"first_failure": miss, "final_complete": False},
                {"first_failure": {"failure_type": "none"}, "final_complete": True}]
        self.assertEqual(recovery_rate(logs), {"first_attempt_failures": 2, "recovered": 1, "rate": 0.5})


if __name__ == "__main__":
    unittest.main()
