"""Grasp fault injection helpers."""

import unittest

from grasp_fault import (
    INJECT_ACCIDENTAL_DROP,
    fault_to_dict,
    inject_fault_label,
    maybe_biased_pick,
    parse_inject_missed_grasp,
)


class GraspFaultTests(unittest.TestCase):
    def test_parse_and_apply_offset_once(self):
        fault = parse_inject_missed_grasp("2:2:0:0")
        self.assertEqual(fault, (2, "offset", 2.0, 0.0, 0.0))
        pick = {"xy": [0.30, 0.12], "yaw_deg": 0.0, "spread_dir": [0, 1, 0]}
        unchanged = maybe_biased_pick(pick, 1, fault, retry_count=0)
        self.assertEqual(unchanged["xy"], pick["xy"])
        biased = maybe_biased_pick(pick, 2, fault, retry_count=0)
        self.assertAlmostEqual(biased["xy"][0], 0.302)
        after_retry = maybe_biased_pick(biased, 2, fault, retry_count=1)
        self.assertEqual(after_retry, biased)

    def test_parse_and_apply_drop_once(self):
        fault = parse_inject_missed_grasp("1:drop")
        self.assertEqual(fault[0], 1)
        self.assertEqual(fault[1], "drop")
        pick = {"xy": [0.30, 0.12], "yaw_deg": 0.0}
        biased = maybe_biased_pick(pick, 1, fault, retry_count=0)
        self.assertTrue(biased[INJECT_ACCIDENTAL_DROP])
        self.assertEqual(inject_fault_label(fault), "accidental drop on pick 1")

    def test_invalid_spec(self):
        with self.assertRaises(ValueError):
            parse_inject_missed_grasp("bad")

    def test_fault_to_dict(self):
        payload = fault_to_dict((1, "offset", 2.0, 0.0, -5.0))
        self.assertEqual(payload["pick_index"], 1)
        self.assertEqual(payload["mode"], "offset")


if __name__ == "__main__":
    unittest.main()
