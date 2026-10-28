"""Pure-policy tests using execution-event-shaped fixtures."""

import copy
import unittest

from recovery_policy import (
    MAX_PICKUP_RETRIES,
    classify_failure,
    propose_recovery_action,
)


class RecoveryClassificationTests(unittest.TestCase):
    def test_successful_events_have_no_failure(self):
        events = [
            {
                "detection_id": "cube-1",
                "pickup_verified": True,
                "placement_verified": True,
                "stack_support_verified": True,
            }
        ]

        self.assertIsNone(classify_failure(events))

    def test_classifies_first_failed_pickup_and_preserves_identity(self):
        events = [
            {
                "detection_id": "cube-1",
                "pickup_verified": True,
                "placement_verified": True,
            },
            {
                "detection_id": "cube-2",
                "pickup_verified": False,
                "placement_verified": False,
            },
            {
                "detection_id": "cube-3",
                "pickup_verified": False,
            },
        ]

        self.assertEqual(
            classify_failure(events),
            {
                "stage": "execution",
                "event_index": 1,
                "failure_type": "pickup_failed",
                "detection_id": "cube-2",
            },
        )

    def test_collision_drop_and_uncertain_state_are_classified_explicitly(self):
        for field, failure_type in (
            ("collision_detected", "collision_detected"),
            ("drop_detected", "dropped_object"),
            ("state_uncertain", "state_uncertain"),
        ):
            with self.subTest(field=field):
                self.assertEqual(
                    classify_failure([{"detection_id": "cube-1", field: True}])[
                        "failure_type"
                    ],
                    failure_type,
                )

    def test_classifies_placement_stack_and_ik_failures(self):
        cases = (
            ({"pickup_verified": True, "placement_verified": False}, "placement_failed"),
            (
                {
                    "pickup_verified": True,
                    "placement_verified": True,
                    "stack_support_verified": False,
                },
                "stack_support_failed",
            ),
            ({"trajectory_ik_failures": 1}, "trajectory_ik_failure"),
        )
        for event, expected in cases:
            with self.subTest(expected=expected):
                self.assertEqual(classify_failure([event])["failure_type"], expected)

    def test_rejects_malformed_event_collection(self):
        with self.assertRaises(TypeError):
            classify_failure({"pickup_verified": False})
        with self.assertRaisesRegex(TypeError, r"events\[0\]"):
            classify_failure([None])


class RetryProposalTests(unittest.TestCase):
    def setUp(self):
        self.plan = {
            "source": "Cosmos3-Nano",
            "task": "cube_stack",
            "picks": [
                {
                    "detection_id": "cube-1",
                    "category": "cube",
                    "xy": [0.31, 0.12],
                    "yaw_deg": 25.0,
                    "spread_dir": [-0.42, 0.91, 0.0],
                },
                {
                    "detection_id": "cube-2",
                    "category": "cube",
                    "xy": [0.38, 0.14],
                },
            ],
        }
        self.failure = {
            "stage": "execution",
            "event_index": 0,
            "failure_type": "pickup_failed",
            "detection_id": "cube-1",
        }

    def test_first_retry_proposes_two_mm_x_nudge_without_mutating_input(self):
        original = copy.deepcopy(self.plan)

        proposal = propose_recovery_action(self.plan, self.failure, retry_number=1)

        self.assertEqual(proposal["action"], "propose_pickup_retry")
        self.assertEqual(proposal["nudge_xy_m"], [0.002, 0.0])
        self.assertEqual(proposal["candidate_plan"]["picks"][0]["xy"], [0.312, 0.12])
        self.assertEqual(proposal["candidate_plan"]["picks"][1], self.plan["picks"][1])
        self.assertEqual(self.plan, original)
        self.assertFalse(proposal["executed"])
        self.assertTrue(proposal["requires_fresh_observation"])
        self.assertTrue(proposal["requires_motion_validation"])

    def test_second_retry_proposes_opposite_nudge(self):
        proposal = propose_recovery_action(self.plan, self.failure, retry_number=2)

        self.assertEqual(proposal["nudge_xy_m"], [-0.002, 0.0])
        self.assertEqual(proposal["candidate_plan"]["picks"][0]["xy"], [0.308, 0.12])

    def test_retry_limit_stops_for_review(self):
        proposal = propose_recovery_action(
            self.plan, self.failure, retry_number=MAX_PICKUP_RETRIES + 1
        )

        self.assertEqual(proposal["action"], "stop_and_review")
        self.assertEqual(proposal["reason"], "pickup_retry_limit_reached")
        self.assertTrue(proposal["requires_operator_review"])

    def test_other_failures_never_propose_motion_retry(self):
        for failure_type in (
            "collision_detected",
            "dropped_object",
            "placement_failed",
            "stack_support_failed",
            "state_uncertain",
        ):
            with self.subTest(failure_type=failure_type):
                proposal = propose_recovery_action(
                    self.plan,
                    {**self.failure, "failure_type": failure_type},
                    retry_number=1,
                )
                self.assertEqual(proposal["action"], "stop_and_review")
                self.assertNotIn("candidate_plan", proposal)

    def test_only_cube_pickups_are_eligible_for_retry(self):
        plan = copy.deepcopy(self.plan)
        plan["picks"][0]["category"] = "vial"

        proposal = propose_recovery_action(plan, self.failure, retry_number=1)

        self.assertEqual(proposal["action"], "stop_and_review")
        self.assertEqual(
            proposal["reason"], "automatic_retry_prototype_supports_cubes_only"
        )

    def test_failure_identity_and_plan_coordinates_are_validated(self):
        mismatch = {**self.failure, "detection_id": "wrong-cube"}
        with self.assertRaisesRegex(ValueError, "does not match"):
            propose_recovery_action(self.plan, mismatch)

        plan = copy.deepcopy(self.plan)
        plan["picks"][0]["xy"] = [float("nan"), 0.12]
        with self.assertRaisesRegex(ValueError, "finite XY"):
            propose_recovery_action(plan, self.failure)

    def test_retry_policy_cannot_be_configured_above_hard_limit(self):
        with self.assertRaisesRegex(ValueError, "between 0 and 2"):
            propose_recovery_action(
                self.plan, self.failure, max_retries=MAX_PICKUP_RETRIES + 1
            )


if __name__ == "__main__":
    unittest.main()
