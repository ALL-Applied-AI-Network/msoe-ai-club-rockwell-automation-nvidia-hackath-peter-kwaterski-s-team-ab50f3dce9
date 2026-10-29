"""Host-only checks for the dashboard's real pipeline adapter and polling loop."""

import json
import os
from pathlib import Path
import queue
import tempfile
import unittest
from unittest.mock import Mock

from TkinterView import COUNT_FIELDS, DemoSource, DigitalTwinSource, SortingView


class DigitalTwinSourceTests(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory()
        self.addCleanup(self.temporary.cleanup)
        self.run = Path(self.temporary.name) / "stack_01"
        self.run.mkdir()
        self.source = DigitalTwinSource(self.run)

    def write(self, name, value):
        path = self.run / name
        path.write_text(json.dumps(value), encoding="utf-8")
        return path

    def plan(self):
        cubes = [dict(detection_id=i, category="cube", eligible=True, skip_reason=None)
                 for i in range(6)]
        distractors = [dict(detection_id=i, category=category, eligible=False, skip_reason="not_a_cube")
                       for i, category in enumerate(("cap", "cap", "vial", "vial"), 6)]
        self.write("sort_plan.json", dict(task="cube_stack", stack_ready=True,
                   observations=cubes + distractors, picks=cubes, skipped=distractors))

    @staticmethod
    def event(identifier, *, placed=True, supported=True):
        return dict(detection_id=identifier, category="cube", tracked_object=f"SortCube{identifier}",
                    pickup_verified=True, placement_verified=placed,
                    stack_support_verified=supported, placement_error_mm=0.25)

    def test_empty_or_missing_run_has_unknown_counts(self):
        for source in (self.source, DigitalTwinSource(self.run / "not_created_yet"),
                       DigitalTwinSource(runs_dir=self.run / "no_runs_yet")):
            with self.subTest(source=source):
                status = source.read_status()
                self.assertIsNone(status.generated_at)
                self.assertTrue(all(value is None for row in status.counts.values() for value in row.values()))

    def test_capture_does_not_use_ground_truth_as_inventory(self):
        self.write("scene_state.json", {"objects": {"SortCube0": {}}})
        self.write("ground_truth.json", {"objects": [{"category": "cube"}]})
        status = self.source.read_status()
        self.assertEqual(status.stage, "Perception (Cosmos)")
        self.assertIsNone(status.counts["Cubes"]["detected"])

    def test_detections_are_not_placements_and_vials_are_tubes(self):
        self.write("detections.json", {"objects": [{"category": "cube"}, {"category": "vial"}]})
        status = self.source.read_status()
        self.assertEqual(status.stage, "Planning")
        self.assertEqual(status.counts["Cubes"]["detected"], 1)
        self.assertEqual(status.counts["Tubes"]["detected"], 1)
        self.assertIsNone(status.counts["Cubes"]["sorted"])
        self.assertIsNone(status.counts["Cubes"]["attempted"])

    def test_plan_preserves_skips_and_unknown_execution_metrics(self):
        self.plan()
        status = self.source.read_status()
        self.assertEqual(status.counts["Cubes"]["detected"], 6)
        self.assertEqual(status.counts["Cubes"]["eligible"], 6)
        self.assertEqual(status.counts["Caps"]["skipped"], 2)
        self.assertEqual(status.counts["Tubes"]["eligible"], 0)
        self.assertIsNone(status.counts["Cubes"]["sorted"])
        self.assertIsNone(status.metrics["Cycle time (wall clock)"])
        self.assertEqual({a.reason for a in status.alerts}, {"not_a_cube"})
        self.assertEqual([a.id for a in status.alerts], [a.id for a in self.source.read_status().alerts])

    def test_events_update_outcomes_and_failure_checks(self):
        self.plan()
        self.write("execution_events.json", [self.event(0), self.event(1, placed=False, supported=False)])
        status = self.source.read_status()
        self.assertEqual(tuple(status.counts["Cubes"][key] for key in COUNT_FIELDS), (6, 6, 2, 1, 0, 1))
        self.assertEqual(status.metrics["Verified / reported attempts"], "1 / 2 (50.0%)")
        self.assertEqual(status.metrics["Maximum placement error"], "0.250 mm (n=2 reported picks)")
        failure = next(a for a in status.alerts if a.outcome == "Failed")
        self.assertIn("placement_verified=false", failure.reason)
        self.assertIn("stack_support_verified=false", failure.reason)
        self.assertFalse(status.finished)

    def test_final_report_supersedes_events_even_if_events_are_incomplete_json(self):
        self.plan()
        (self.run / "execution_events.json").write_text("[", encoding="utf-8")
        self.write("execution_report.json", dict(picks=[self.event(i) for i in range(6)],
                   task_complete=True, abort_reason=None, verified_pickups=6, verified_placements=6))
        status = self.source.read_status()
        self.assertTrue(status.finished)
        self.assertTrue(status.simulated)
        self.assertFalse(status.demonstration)
        self.assertEqual(status.status, "Completed")
        self.assertEqual(status.counts["Cubes"]["sorted"], 6)

    def test_abort_reason_is_preserved(self):
        reason = "Cube tower stopped at level 1: pickup, placement or supporting stack failed"
        self.write("execution_report.json", dict(picks=[self.event(0, supported=False)],
                   task_complete=False, abort_reason=reason))
        status = self.source.read_status()
        self.assertEqual(status.status, "Incomplete / aborted")
        self.assertIn(reason, [a.reason for a in status.alerts])
        self.assertEqual(status.counts["Cubes"]["failed"], 1)
        self.assertIsNone(status.counts["Cubes"]["detected"])

    def test_failed_support_is_not_counted_as_sorted(self):
        self.write("execution_events.json", [self.event(0, supported=False)])
        status = self.source.read_status()
        self.assertEqual(status.counts["Cubes"]["sorted"], 0)
        self.assertEqual(status.counts["Cubes"]["failed"], 1)
        self.assertEqual(status.metrics["Verified / reported attempts"], "0 / 1 (0.0%)")

    def test_failed_pose_is_not_counted_as_sorted(self):
        event = dict(self.event(0), placement_pose_verified=False)
        self.write("execution_events.json", [event])
        status = self.source.read_status()
        self.assertEqual(status.counts["Cubes"]["sorted"], 0)
        self.assertIn("placement_pose_verified=false", status.alerts[0].reason)

    def test_success_report_must_cover_all_planned_picks(self):
        self.plan()
        self.write("execution_report.json", dict(picks=[self.event(0)], task_complete=True))
        with self.assertRaisesRegex(ValueError, "task_complete conflicts"):
            self.source.read_status()

    def test_report_totals_must_match_pick_results(self):
        self.write("execution_report.json", dict(picks=[self.event(0)], task_complete=True,
                   verified_pickups=6, verified_placements=6))
        with self.assertRaisesRegex(ValueError, "verified_pickups conflicts"):
            self.source.read_status()

    def test_incomplete_inventory_is_blocked_instead_of_completed(self):
        obj = dict(detection_id=0, category="cube", eligible=False, skip_reason="incomplete_cube_colour_set")
        self.write("sort_plan.json", dict(observations=[obj], picks=[], skipped=[obj]))
        status = self.source.read_status()
        self.assertEqual(status.status, "Blocked: no eligible picks")
        self.assertEqual(status.alerts[0].reason, "incomplete_cube_colour_set")
        self.assertIsNone(status.counts["Cubes"]["attempted"])

    def test_partial_write_is_reported_and_next_poll_recovers(self):
        path = self.run / "detections.json"
        path.write_text('{"objects": [', encoding="utf-8")
        with self.assertRaisesRegex(ValueError, "detections.json: invalid or incomplete JSON"):
            self.source.read_status()
        self.write("detections.json", {"objects": []})
        self.assertEqual(self.source.read_status().counts["Cubes"]["detected"], 0)

    def test_invalid_execution_schema_is_rejected(self):
        invalid_events = [
            [dict(self.event(0), placement_verified="true")],
            [dict(self.event(0), pickup_verified=False)],
            [dict(self.event(0), placement_error_mm=float("nan"))],
            [self.event(0), self.event(0)],
        ]
        for events in invalid_events:
            with self.subTest(events=events):
                self.write("execution_events.json", events)
                with self.assertRaises(ValueError):
                    self.source.read_status()

    def test_inconsistent_success_report_is_rejected(self):
        self.write("execution_report.json", dict(picks=[self.event(0, placed=False)], task_complete=True))
        with self.assertRaisesRegex(ValueError, "task_complete conflicts"):
            self.source.read_status()

    def test_file_timestamp_does_not_refresh_on_poll(self):
        path = self.write("detections.json", {"objects": []})
        os.utime(path, (1000000000, 1000000000))
        first = self.source.read_status()
        self.assertEqual(first.generated_at.timestamp(), 1000000000)
        self.assertEqual(first.generated_at, self.source.read_status().generated_at)

    def test_auto_follow_newest_run_directory(self):
        second = self.run.parent / "stack_02"
        second.mkdir()
        os.utime(self.run, (1000000000, 1000000000))
        os.utime(second, (1000000001, 1000000001))
        source = DigitalTwinSource(runs_dir=self.run.parent)
        self.assertEqual(source.read_status().run_id, "stack_02")
        os.utime(self.run, (1000000002, 1000000002))
        self.assertEqual(source.read_status().run_id, "stack_01")

    def test_demo_works_and_is_distinguished_from_twin_output(self):
        status = DemoSource().read_status()
        self.assertTrue(status.demonstration)
        self.assertTrue(status.finished)
        self.assertEqual(status.counts["Cubes"]["sorted"], 5)


class PollingTests(unittest.TestCase):
    def view(self):
        view = SortingView.__new__(SortingView)
        view.root = Mock()
        view.events = queue.Queue()
        view.snapshot = object()
        view.last_received = None
        view.error = None
        view.render = Mock()
        view.update_freshness = Mock()
        return view

    def test_successful_read_keeps_polling(self):
        view = self.view()
        status = DemoSource().read_status()
        view.events.put((status, None))
        view.drain_events()
        self.assertIs(view.snapshot, status)
        view.render.assert_called_once_with(status)
        view.update_freshness.assert_called_once()
        view.root.after.assert_called_once_with(100, view.drain_events)

    def test_error_retains_last_snapshot_and_keeps_polling(self):
        view = self.view()
        previous = view.snapshot
        view.events.put((None, "incomplete JSON"))
        view.drain_events()
        self.assertIs(view.snapshot, previous)
        self.assertEqual(view.error, "incomplete JSON")
        view.render.assert_not_called()
        view.root.after.assert_called_once_with(100, view.drain_events)

    def test_empty_queue_keeps_polling(self):
        view = self.view()
        view.drain_events()
        view.update_freshness.assert_called_once()
        view.root.after.assert_called_once_with(100, view.drain_events)


if __name__ == "__main__":
    unittest.main()
