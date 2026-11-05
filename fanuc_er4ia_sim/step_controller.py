"""Incremental cube-stack control state for stepwise Isaac execution."""

from __future__ import annotations

from copy import deepcopy
from pathlib import Path
from typing import Any, Dict, List, Optional

import scene_config as C

from run_artifacts import atomic_write_json, read_json, utc_now_iso

STEP_STATE_NAME = "step_state.json"
PHASES = (
    "initializing",
    "ready",
    "executing",
    "verifying",
    "recovering",
    "completed",
    "stopped",
)


def default_step_state(*, task: str = "cube_stack") -> Dict[str, Any]:
    return {
        "version": 1,
        "task": task,
        "phase": "initializing",
        "current_pick_index": 0,
        "completed_pick_indices": [],
        "retry_count": 0,
        "recovery_rungs_used": 0,
        "recovery_retries": 0,
        "terminal_reason": None,
        "updated_at": utc_now_iso(),
    }


def load_step_state(run_dir, plan: Optional[dict] = None) -> Dict[str, Any]:
    run_dir = Path(run_dir)
    existing = read_json(run_dir / STEP_STATE_NAME)
    if isinstance(existing, dict) and existing.get("version") == 1:
        return existing
    task = "cube_stack"
    if plan is not None:
        task = plan.get("task", task)
    return default_step_state(task=task)


def save_step_state(run_dir, state: Dict[str, Any]) -> None:
    run_dir = Path(run_dir)
    payload = deepcopy(state)
    payload["updated_at"] = utc_now_iso()
    atomic_write_json(run_dir / STEP_STATE_NAME, payload)


def total_picks(plan: dict) -> int:
    picks = plan.get("picks")
    if not isinstance(picks, list):
        return 0
    return len(picks)


def current_pick(plan: dict, state: dict) -> Optional[dict]:
    index = state.get("current_pick_index", 0)
    picks = plan.get("picks", [])
    if not isinstance(index, int) or index < 0 or index >= len(picks):
        return None
    return picks[index]


def is_terminal(state: dict) -> bool:
    return state.get("phase") in ("completed", "stopped")


def mark_phase(state: dict, phase: str) -> Dict[str, Any]:
    if phase not in PHASES:
        raise ValueError(f"Unknown phase {phase!r}")
    updated = deepcopy(state)
    updated["phase"] = phase
    return updated


def advance_after_success(state: dict, plan: dict) -> Dict[str, Any]:
    index = int(state["current_pick_index"])
    completed: List[int] = list(state.get("completed_pick_indices", []))
    if index not in completed:
        completed.append(index)
    next_index = index + 1
    updated = deepcopy(state)
    updated["completed_pick_indices"] = completed
    updated["retry_count"] = 0
    updated["recovery_rungs_used"] = 0
    updated["recovery_retries"] = 0
    if next_index >= total_picks(plan):
        updated["phase"] = "completed"
        updated["current_pick_index"] = index
        updated["terminal_reason"] = None
    else:
        updated["phase"] = "ready"
        updated["current_pick_index"] = next_index
    return updated


def begin_recovery(state: dict) -> Dict[str, Any]:
    updated = deepcopy(state)
    updated["phase"] = "recovering"
    updated["retry_count"] = int(state.get("retry_count", 0)) + 1
    return updated


def apply_recovery_plan(state: dict, plan: dict) -> Dict[str, Any]:
    """Keep the same pick index; planner counters come from the updated plan."""
    updated = deepcopy(state)
    updated["phase"] = "ready"
    updated["recovery_rungs_used"] = int(plan.get("recovery_rungs_used", 0))
    updated["recovery_retries"] = int(plan.get("recovery_retries", 0))
    return updated


def mark_stopped(state: dict, reason: str) -> Dict[str, Any]:
    updated = deepcopy(state)
    updated["phase"] = "stopped"
    updated["terminal_reason"] = reason
    return updated


def cube_stack_failure_reason(event: dict) -> Optional[str]:
    if not event.get("pickup_verified"):
        return "pickup verification failed"
    if not event.get("placement_verified"):
        return "placement verification failed"
    if event.get("stack_support_verified") is False:
        return "stack support verification failed"
    return None


def expected_stack_levels() -> int:
    return len(C.CUBE_STACK_COLOURS)
