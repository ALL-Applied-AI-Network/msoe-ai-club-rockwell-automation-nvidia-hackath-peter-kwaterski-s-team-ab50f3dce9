"""Pure recovery classification and retry-plan proposal helpers.

These functions do not read or write files and do not execute robot motion.
Retry plans are candidates that an execution layer must validate before use.
"""

from copy import deepcopy
import math


MAX_PICKUP_RETRIES = 2
PICKUP_RETRY_OFFSETS_M = ((0.002, 0.0), (-0.002, 0.0))


def classify_failure(events):
    """Return the first detectable execution failure, or ``None``.

    The classifier only infers failures from explicit event fields. Collision
    and drop detection are recognized when supplied by an execution adapter;
    the current simulator event schema does not yet produce those fields.
    """
    if not isinstance(events, list):
        raise TypeError("events must be a list")

    for event_index, event in enumerate(events):
        if not isinstance(event, dict):
            raise TypeError(f"events[{event_index}] must be a dictionary")

        failure_type = None
        if event.get("collision_detected") is True:
            failure_type = "collision_detected"
        elif event.get("drop_detected") is True:
            failure_type = "dropped_object"
        elif event.get("state_uncertain") is True:
            failure_type = "state_uncertain"
        elif event.get("pickup_verified") is False:
            failure_type = "pickup_failed"
        elif event.get("placement_verified") is False:
            failure_type = "placement_failed"
        elif event.get("stack_support_verified") is False:
            failure_type = "stack_support_failed"
        elif event.get("trajectory_ik_failures", 0) > 0:
            failure_type = "trajectory_ik_failure"

        if failure_type is not None:
            failure = {
                "stage": "execution",
                "event_index": event_index,
                "failure_type": failure_type,
            }
            if "detection_id" in event:
                failure["detection_id"] = event["detection_id"]
            return failure
    return None


def propose_recovery_action(
    plan, failure, retry_number=1, max_retries=MAX_PICKUP_RETRIES
):
    """Return a stop decision or an unexecuted candidate plan for a pickup retry.

    Retry numbers are one-based. The first and second candidates shift the
    selected cube by +2 mm and -2 mm along world X, respectively. The returned
    candidate is not collision-checked and must not be executed without fresh
    observation and motion validation.
    """
    if not isinstance(plan, dict):
        raise TypeError("plan must be a dictionary")
    if not isinstance(failure, dict):
        raise TypeError("failure must be a dictionary")
    if isinstance(max_retries, bool) or not isinstance(max_retries, int):
        raise TypeError("max_retries must be an integer")
    if not 0 <= max_retries <= MAX_PICKUP_RETRIES:
        raise ValueError(f"max_retries must be between 0 and {MAX_PICKUP_RETRIES}")

    failure_type = failure.get("failure_type")
    if failure_type != "pickup_failed":
        return {
            "action": "stop_and_review",
            "reason": failure_type or "unclassified_failure",
            "requires_operator_review": True,
        }

    if isinstance(retry_number, bool) or not isinstance(retry_number, int):
        raise TypeError("retry_number must be an integer")
    if retry_number < 1:
        raise ValueError("retry_number must be one-based")
    if retry_number > max_retries:
        return {
            "action": "stop_and_review",
            "reason": "pickup_retry_limit_reached",
            "requires_operator_review": True,
            "retry_number": retry_number,
            "max_retries": max_retries,
        }

    event_index = failure.get("event_index")
    if isinstance(event_index, bool) or not isinstance(event_index, int):
        raise ValueError("pickup failure must contain an integer event_index")
    picks = plan.get("picks")
    if not isinstance(picks, list) or not 0 <= event_index < len(picks):
        raise ValueError("failure event_index does not identify a planned pick")

    candidate = deepcopy(plan)
    pick = candidate["picks"][event_index]
    original_pick = plan["picks"][event_index]
    if not isinstance(pick, dict) or not isinstance(original_pick, dict):
        raise ValueError("planned pick must be a dictionary")
    if pick.get("category") != "cube":
        return {
            "action": "stop_and_review",
            "reason": "automatic_retry_prototype_supports_cubes_only",
            "requires_operator_review": True,
        }
    if (
        "detection_id" in failure
        and original_pick.get("detection_id") != failure["detection_id"]
    ):
        raise ValueError("failure detection_id does not match the indexed planned pick")

    xy = pick.get("xy")
    if (
        not isinstance(xy, list)
        or len(xy) != 2
        or any(isinstance(value, bool) or not isinstance(value, (int, float)) for value in xy)
        or not all(math.isfinite(value) for value in xy)
    ):
        raise ValueError("planned cube pick must have two finite XY coordinates")

    offset = PICKUP_RETRY_OFFSETS_M[retry_number - 1]
    pick["xy"] = [xy[0] + offset[0], xy[1] + offset[1]]
    return {
        "action": "propose_pickup_retry",
        "retry_number": retry_number,
        "max_retries": max_retries,
        "reason": "pickup_failed",
        "nudge_xy_m": list(offset),
        "candidate_plan": candidate,
        "requires_fresh_observation": True,
        "requires_motion_validation": True,
        "executed": False,
    }
