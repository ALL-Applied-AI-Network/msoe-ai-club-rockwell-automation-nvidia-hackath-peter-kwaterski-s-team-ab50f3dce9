"""Host-side tests for failure classification and bounded retry decisions."""

import copy
import sys
import types
import unittest
from unittest.mock import patch

from recovery_policy import (
    MAX_PICKUP_RETRIES,
    RETRY_LADDER,
    apply_action,
    classify_event,
    classify_failure,
    decide,
    next_action,
    propose_recovery_action,
    recovery_rate,
)


def event(**changes):
    value = {
        "detection_id": 7,
        "tracked_object": "SortCube0",
        "objects_lifted": ["SortCube0"],
        "pickup_verified": True,
        "placement_verified": True,
        "stack_support_verified": True,
        "trajectory_ik_failures": 0,
    }
    value.update(changes)
    return value


def plan():
    return {
        "source": "Cosmos3-Nano",
        "task": "cube_stack",
        "picks": [
            {
                "detection_id": 7,
                "category": "cube",
                "colour": "yellow",
                "xy": [0.31, 0.12],
                "place_xy": [0.4, 0.1],
                "yaw_deg": 25.0,
                "spread_dir": [-0.4226182617, 0.9063077870, 0.0],
                "place_spread_dir": [0.0, 1.0, 0.0],
            },
            {
                "detection_id": 9,
                "category": "cube",
                "colour": "yellow",
                "xy": [0.34, 0.12],
                "place_xy": [0.4, 0.1],
                "yaw_deg": 0.0,
                "spread_dir": [0.0, 1.0, 0.0],
                "place_spread_dir": [0.0, 1.0, 0.0],
            },
        ],
    }


class ClassificationTests(unittest.TestCase):
    def test_successful_events_have_no_failure(self):
        self.assertIsNone(classify_failure([event()]))
        self.assertEqual(
            classify_failure(
                [event()], {"picks": [{}]}, {"task_complete": True}
            )["failure_type"],
            "none",
        )

    def test_first_failure_wins_and_distinguishes_missed_or_wrong_object(self):
        events = [
            event(),
            event(
                detection_id=9,
                tracked_object="SortCube1",
                pickup_verified=False,
                objects_lifted=[],
            ),
            event(detection_id=11, pickup_verified=False, objects_lifted=["SortCap0"]),
        ]
        failure = classify_failure(events)
        self.assertEqual(
            failure,
            {
                "stage": "pickup",
                "event_index": 1,
                "failure_type": "missed_grasp",
                "detection_id": 9,
            },
        )
        self.assertEqual(
            classify_event(events[2]),
            "wrong_object",
        )

    def test_failure_categories(self):
        cases = (
            (event(tracked_object=None), "untracked_target"),
            (event(trajectory_ik_failures=1), "ik_failure"),
            (event(pickup_verified=False, objects_lifted=[]), "missed_grasp"),
            (event(objects_lifted=["SortCube0", "SortCube1"]), "wrong_object"),
            (event(placement_verified=False), "placement_failed"),
            (event(stack_support_verified=False), "support_failed"),
            (event(collision_detected=True), "collision_detected"),
            (event(drop_detected=True), "dropped_object"),
            (event(state_uncertain=True), "state_uncertain"),
        )
        for sample, expected in cases:
            with self.subTest(expected=expected):
                self.assertEqual(classify_event(sample), expected)

    def test_failure_report_detection_and_incomplete_events(self):
        self.assertEqual(
            classify_failure([], {"picks": [{}]}, None)["failure_type"],
            "execution_crash",
        )
        self.assertEqual(
            classify_failure(
                [event()], {"picks": [{}, {}]}, {"task_complete": False}
            )["failure_type"],
            "execution_crash",
        )
        self.assertEqual(
            classify_failure(
                [event()], {"picks": [{}]}, {"task_complete": False}
            )["failure_type"],
            "incomplete",
        )

    def test_success_report_cannot_override_failed_event(self):
        result = classify_failure(
            [event(pickup_verified=False, objects_lifted=[])],
            {"picks": [{}]},
            {"task_complete": True},
        )
        self.assertEqual(result["failure_type"], "missed_grasp")

    def test_missing_report_stops_even_when_a_pickup_failure_was_recorded(self):
        result = classify_failure(
            [event(pickup_verified=False, objects_lifted=[])],
            {"picks": [{}]},
            None,
        )
        self.assertEqual(result["failure_type"], "execution_crash")

    def test_invalid_inputs_are_explicit(self):
        with self.assertRaises(TypeError):
            classify_failure({"pickup_verified": False})
        with self.assertRaisesRegex(TypeError, r"events\[0\]"):
            classify_failure([None])
        with self.assertRaisesRegex(ValueError, "objects_lifted"):
            classify_event(event(objects_lifted=None))


class RetryDecisionTests(unittest.TestCase):
    def setUp(self):
        self.plan = plan()
        self.failure = {
            "stage": "pickup",
            "event_index": 0,
            "failure_type": "missed_grasp",
            "detection_id": 7,
        }

    def test_retries_are_hard_bounded_and_only_for_missed_grasps(self):
        for kind in (
            "wrong_object",
            "placement_failed",
            "support_failed",
            "ik_failure",
            "execution_crash",
            "collision_detected",
            "dropped_object",
            "state_uncertain",
        ):
            with self.subTest(kind=kind):
                self.assertEqual(
                    next_action({"failure_type": kind}, 0),
                    "stop",
                )
        self.assertEqual(next_action({"failure_type": "none"}, 0), "done")
        self.assertEqual(
            next_action(self.failure, MAX_PICKUP_RETRIES), "stop"
        )
        self.assertEqual(MAX_PICKUP_RETRIES, 2)

    def test_action_ladder_keeps_two_retry_limit(self):
        self.assertEqual(RETRY_LADDER, ("remeasure", "flip_spread", "nudge"))
        first = decide(
            [event(pickup_verified=False, objects_lifted=[])],
            self.plan,
            {"task_complete": False},
        )
        self.assertEqual(first["action"], "flip_spread")
        self.assertEqual(first["skipped"], ["remeasure"])
        self.assertEqual(first["plan"]["recovery_rungs_used"], 2)
        self.assertEqual(first["plan"]["recovery_retries"], 1)

        second = decide(
            [event(pickup_verified=False, objects_lifted=[])],
            first["plan"],
            {"task_complete": False},
        )
        self.assertEqual(second["action"], "nudge")
        self.assertEqual(second["plan"]["recovery_retries"], 2)
        self.assertEqual(second["plan"]["recovery_rungs_used"], 3)

        third = decide(
            [event(pickup_verified=False, objects_lifted=[])],
            second["plan"],
            {"task_complete": False},
        )
        self.assertEqual(third["action"], "stop")
        self.assertIsNone(third["plan"])

    def test_missing_image_skips_remeasure_without_using_a_retry(self):
        decision = decide(
            [event(pickup_verified=False, objects_lifted=[])],
            self.plan,
            {"task_complete": False},
            max_retries=1,
        )
        self.assertEqual(decision["action"], "flip_spread")
        self.assertEqual(decision["plan"]["recovery_retries"], 1)
        self.assertEqual(decision["plan"]["recovery_rungs_used"], 2)

    def test_remeasure_uses_image_refinement_and_preserves_input(self):
        fake_pose = types.ModuleType("cube_pose")
        fake_pose.refine_pose = lambda *args, **kwargs: ([0.311, 0.121], 31.0, 0.92)
        original = copy.deepcopy(self.plan)
        with patch.dict(sys.modules, {"cube_pose": fake_pose}):
            candidate = apply_action(
                self.plan,
                self.failure,
                "remeasure",
                image=object(),
                cam=object(),
            )
        self.assertEqual(candidate["picks"][0]["xy"], [0.311, 0.121])
        self.assertAlmostEqual(candidate["picks"][0]["yaw_deg"], 31.0)
        self.assertEqual(candidate["picks"][1], self.plan["picks"][1])
        self.assertEqual(self.plan, original)

    def test_weak_remeasure_skips_to_flip(self):
        fake_pose = types.ModuleType("cube_pose")
        fake_pose.refine_pose = lambda *args, **kwargs: ([0.31, 0.12], 25.0, 0.2)
        with patch.dict(sys.modules, {"cube_pose": fake_pose}):
            result = decide(
                [event(pickup_verified=False, objects_lifted=[])],
                self.plan,
                {"task_complete": False},
                image=object(),
                cam=object(),
            )
        self.assertEqual(result["action"], "flip_spread")
        self.assertEqual(result["skipped"], ["remeasure"])

    def test_flip_produces_plan_compatible_wrist_yaw(self):
        candidate = apply_action(self.plan, self.failure, "flip_spread")
        updated = candidate["picks"][0]
        self.assertEqual(updated["yaw_deg"], self.plan["picks"][0]["yaw_deg"] + 90)
        self.assertAlmostEqual(
            updated["spread_dir"][0],
            -__import__("math").sin(__import__("math").radians(updated["yaw_deg"])),
        )
        self.assertAlmostEqual(
            updated["spread_dir"][1],
            __import__("math").cos(__import__("math").radians(updated["yaw_deg"])),
        )

    def test_nudge_is_two_mm_along_grasp_spread(self):
        candidate = apply_action(self.plan, self.failure, "nudge")
        old = self.plan["picks"][0]["xy"]
        new = candidate["picks"][0]["xy"]
        dx, dy = new[0] - old[0], new[1] - old[1]
        self.assertAlmostEqual((dx * dx + dy * dy) ** 0.5, 0.002)
        self.assertAlmostEqual(dx / dy, self.plan["picks"][0]["spread_dir"][0] /
                               self.plan["picks"][0]["spread_dir"][1])

    def test_mismatch_and_invalid_plan_do_not_silently_change_pick(self):
        mismatch = {**self.failure, "detection_id": 99}
        with self.assertRaisesRegex(ValueError, "disagree"):
            apply_action(self.plan, mismatch, "flip_spread")
        invalid = copy.deepcopy(self.plan)
        invalid["picks"][0]["xy"] = [float("nan"), 0.12]
        with self.assertRaisesRegex(ValueError, "finite XY"):
            apply_action(invalid, self.failure, "nudge")

    def test_legacy_proposal_remains_bounded_and_pure(self):
        original = copy.deepcopy(self.plan)
        result = propose_recovery_action(
            self.plan,
            {
                "failure_type": "pickup_failed",
                "event_index": 0,
                "detection_id": 7,
            },
            retry_number=1,
        )
        self.assertEqual(result["candidate_plan"]["picks"][0]["xy"], [0.312, 0.12])
        self.assertEqual(self.plan, original)

    def test_recovery_rate_uses_only_first_attempt_pickup_failures(self):
        logs = [
            {"first_failure": {"failure_type": "missed_grasp"}, "final_complete": True},
            {"first_failure": {"failure_type": "missed_grasp"}, "final_complete": False},
            {"first_failure": {"failure_type": "placement_failed"}, "final_complete": True},
        ]
        self.assertEqual(
            recovery_rate(logs),
            {"first_attempt_failures": 2, "recovered": 1, "rate": 0.5},
        )


if __name__ == "__main__":
    unittest.main()
