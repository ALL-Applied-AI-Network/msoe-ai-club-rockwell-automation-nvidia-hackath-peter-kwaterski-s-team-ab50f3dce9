import math
import unittest

from lego_insertion import (
    CandidateMeasurement,
    InsertionConfig,
    SearchOffset,
    bounded_search_offsets,
    contact_wrench,
    rank_candidates,
    score_candidate,
)


class InsertionConfigTests(unittest.TestCase):
    def test_defaults_are_lego_scale_and_bounded(self):
        config = InsertionConfig()
        self.assertEqual(config.pitch_m, 0.008)
        self.assertEqual(config.stud_diameter_m, 0.0048)
        self.assertEqual(config.maximum_corrections, 12)
        self.assertLess(config.xy_radius_m, config.pitch_m / 2)

    def test_invalid_config_is_rejected(self):
        with self.assertRaises(ValueError):
            InsertionConfig(xy_step_m=0.0)
        with self.assertRaises(ValueError):
            InsertionConfig(stud_diameter_m=0.009)
        with self.assertRaises(ValueError):
            InsertionConfig(yaw_steps_deg=(0.5, 0.25))


class SearchTests(unittest.TestCase):
    def test_search_is_deterministic_unique_and_inside_radial_bound(self):
        config = InsertionConfig()
        first = bounded_search_offsets(config)
        second = bounded_search_offsets(config)
        self.assertEqual(first, second)
        self.assertEqual(first[0], SearchOffset(0.0, 0.0, 0.0, "center"))
        self.assertEqual(
            [(item.x_m, item.y_m) for item in first[1:5]],
            [
                (config.xy_step_m, 0.0),
                (-config.xy_step_m, 0.0),
                (0.0, config.xy_step_m),
                (0.0, -config.xy_step_m),
            ],
        )
        keys = {(item.x_m, item.y_m, item.yaw_deg) for item in first}
        self.assertEqual(len(keys), len(first))
        for item in first:
            self.assertLessEqual(item.radial_distance_m, config.xy_radius_m + 1.0e-12)
        self.assertEqual([item.yaw_deg for item in first[-6:]], [0.25, -0.25, 0.5, -0.5, 1.0, -1.0])

    def test_single_site_search_suppresses_yaw_and_supports_runtime_cap(self):
        offsets = bounded_search_offsets(
            yaw_observable=False,
            maximum_candidates=12,
        )
        self.assertEqual(len(offsets), 12)
        self.assertTrue(all(item.yaw_deg == 0.0 for item in offsets))


class ScoringTests(unittest.TestCase):
    def test_better_measured_pose_scores_first(self):
        center = SearchOffset(0.0, 0.0)
        good = CandidateMeasurement(center, 0.0001, 0.2, True, 0.4, sequence=2)
        bad = CandidateMeasurement(center, 0.0005, 2.0, True, 0.4, sequence=1)
        no_contact = CandidateMeasurement(center, 0.0001, 0.2, False, 0.0, sequence=0)
        self.assertLess(score_candidate(good).total, score_candidate(bad).total)
        self.assertLess(score_candidate(good).total, score_candidate(no_contact).total)
        self.assertEqual(rank_candidates((bad, good, no_contact))[0], good)

    def test_force_cap_rejects_candidate_and_ties_use_sequence(self):
        offset = SearchOffset(0.0, 0.0)
        overloaded = CandidateMeasurement(offset, 0.0, 0.0, True, 5.01)
        self.assertTrue(math.isinf(score_candidate(overloaded).total))
        later = CandidateMeasurement(offset, 0.0002, 0.5, True, sequence=2)
        earlier = CandidateMeasurement(offset, 0.0002, 0.5, True, sequence=1)
        self.assertEqual(rank_candidates((later, earlier)), (earlier, later))


class WrenchTests(unittest.TestCase):
    def test_impulses_reduce_to_force_and_torque(self):
        wrench = contact_wrench(
            [
                {
                    "position_world": (0.0, 1.0, 0.0),
                    "impulse_on_active_body_Ns": (2.0, 0.0, 0.0),
                },
                {
                    "position_world": (0.0, 0.0, 0.0),
                    "impulse_on_active_body_Ns": (0.0, 2.0, 0.0),
                },
            ],
            origin_world=(0.0, 0.0, 0.0),
            duration_s=2.0,
        )
        self.assertEqual(wrench.force_world_N, (1.0, 1.0, 0.0))
        self.assertEqual(wrench.torque_world_Nm, (0.0, 0.0, -1.0))
        self.assertEqual(wrench.contact_count, 2)

    def test_wrench_rotates_into_requested_frame_and_applies_sign(self):
        # Frame +X is world +Y, frame +Y is world -X, frame +Z is world +Z.
        rotation = (
            (0.0, -1.0, 0.0),
            (1.0, 0.0, 0.0),
            (0.0, 0.0, 1.0),
        )
        wrench = contact_wrench(
            [
                {
                    "position_world": (0.0, 0.0, 0.0),
                    "impulse_world": (2.0, 0.0, 0.0),
                    "active_body_sign": -1,
                }
            ],
            origin_world=(0.0, 0.0, 0.0),
            duration_s=1.0,
            world_from_frame=rotation,
        )
        self.assertEqual(wrench.force_world_N, (-2.0, 0.0, 0.0))
        self.assertEqual(wrench.force_frame_N, (0.0, 2.0, 0.0))

    def test_bad_contact_records_are_rejected(self):
        with self.assertRaises(ValueError):
            contact_wrench([], origin_world=(0, 0, 0), duration_s=0)
        with self.assertRaises(ValueError):
            contact_wrench(
                [{"position_world": (0, 0, 0)}],
                origin_world=(0, 0, 0),
                duration_s=1,
            )


if __name__ == "__main__":
    unittest.main()
