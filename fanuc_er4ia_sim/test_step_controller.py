"""Unit tests for incremental step controller state."""

import json
import tempfile
import unittest
from pathlib import Path

import scene_config as C

from step_controller import (
    advance_after_success,
    default_step_state,
    is_terminal,
    load_step_state,
    mark_stopped,
    save_step_state,
    total_picks,
)


def sample_plan():
    picks = []
    for level, colour in enumerate(C.CUBE_STACK_COLOURS):
        picks.append(
            dict(
                detection_id=level,
                category="cube",
                colour=colour,
                stack_level=level,
                xy=[0.3, 0.0],
                place_xy=list(C.CUBE_STACK_XY),
            )
        )
    return {"task": "cube_stack", "picks": picks}


class StepControllerTests(unittest.TestCase):
    def test_advance_through_stack(self):
        plan = sample_plan()
        state = default_step_state()
        state = advance_after_success(state, plan)
        self.assertEqual(state["current_pick_index"], 1)
        self.assertEqual(state["completed_pick_indices"], [0])
        self.assertFalse(is_terminal(state))

    def test_completion_after_final_pick(self):
        plan = sample_plan()
        state = default_step_state()
        for _ in range(total_picks(plan)):
            state = advance_after_success(state, plan)
        self.assertTrue(is_terminal(state))
        self.assertEqual(state["phase"], "completed")

    def test_persist_round_trip(self):
        with tempfile.TemporaryDirectory() as tmp:
            run = Path(tmp)
            plan = sample_plan()
            state = default_step_state()
            save_step_state(run, state)
            loaded = load_step_state(run, plan)
            self.assertEqual(loaded["phase"], "initializing")
            stopped = mark_stopped(state, "test stop")
            save_step_state(run, stopped)
            payload = json.loads((run / "step_state.json").read_text(encoding="utf-8"))
            self.assertEqual(payload["terminal_reason"], "test stop")


if __name__ == "__main__":
    unittest.main()
