# Recovery policy prototype

## Purpose and status

`recovery_policy.py` is an isolated, pure-Python prototype for the recovery
workstream. It classifies failures from execution-event dictionaries and
proposes a bounded retry-plan candidate for a failed cube pickup.

This is **not live recovery**. It does not read run files, access Isaac Sim,
observe the scene, validate trajectories, or command motion. The execution
pipeline and GUI do not call it. A retry proposal must not be sent directly to
the motion executor.

## Project context

The pipeline in `run_sort_pipeline.sh` runs capture, perception, planning,
Isaac execution, and video encoding as separate stages. `sort_scene.py`
performs the planned motions and records per-pick events in
`execution_events.json`. The recovery policy is deliberately separate so its
classification and decision behavior can be tested without Isaac, Cosmos, or
changes to the GUI.

The intended data flow for this prototype is:

1. A caller supplies the event list written by the executor and the associated
   plan.
2. `classify_failure(events)` identifies the first event with a recognized
   failure indicator.
3. `propose_recovery_action(plan, failure, ...)` returns either a stop-for-review
   decision or a copied candidate plan.
4. A future integration layer would need to obtain fresh observations, validate
   the candidate motion, execute it in the current simulation, and verify the
   outcome. Those steps are not implemented here.

## Functions and interfaces

### `classify_failure(events)`

Accepts a list of dictionaries shaped like execution events. Returns `None`
when no failure indicator is present, or a dictionary with:

- `stage`: currently `execution`
- `event_index`: index into both the event list and, by assumption, the plan's
  `picks` list
- `failure_type`: one recognized failure classification
- `detection_id`: included when present in the event

Failure indicators, in precedence order, are:

1. `collision_detected: true`
2. `drop_detected: true`
3. `state_uncertain: true`
4. `pickup_verified: false`
5. `placement_verified: false`
6. `stack_support_verified: false`
7. `trajectory_ik_failures > 0`

The current `sort_scene.py` event schema provides pickup, placement, stack
support, and IK failure information. It does **not** currently produce the
collision, drop, or uncertain-state flags. Those classifications are available
only for callers that provide those explicit fields; this module does not
detect the conditions itself.

The event-index-to-pick-index relationship is an assumption, not a validated
identifier mapping. A future caller must ensure events correspond positionally
to the plan or replace this mapping with stable pick identifiers.

### `propose_recovery_action(plan, failure, retry_number=1, max_retries=2)`

For `pickup_failed` on a planned cube, returns a deep-copied candidate plan with
the selected pick's world-X coordinate offset:

- Retry 1: `+0.002` metres
- Retry 2: `-0.002` metres

The policy hard-caps `max_retries` at two. Retry numbers are one-based. Returned
candidate actions include `executed: false`,
`requires_fresh_observation: true`, and
`requires_motion_validation: true`.

All other failure types return `stop_and_review`; non-cube picks also stop for
review. Invalid input shapes and mismatched failure/pick identities raise
errors rather than silently selecting a different target.

The XY offset is a testable example proposal, **not an approved or safe motion
target**. It is not collision-checked, does not re-estimate object position,
and does not verify reachability. Do not execute it until a live integration
establishes current scene state and validates the whole candidate trajectory.

## Interactions and change impact

Current code interactions:

- Reads no files and writes no files.
- Imports no simulator, perception, or planning modules.
- Accepts plain Python dictionaries shaped like the planner and execution
  output.
- Returns plain Python dictionaries; it does not mutate the supplied plan.
- Is tested by `test_recovery_policy.py` and included in the starter-package
  allowlist in `package_hackathon.py`.
- Does not change `sort_scene.py`, `plan_sort.py`, `run_sort_pipeline.sh`, or
  `TkinterView`.

Changes likely to affect this prototype:

- Renaming/removing event fields such as `pickup_verified`,
  `placement_verified`, `stack_support_verified`, or
  `trajectory_ik_failures`.
- Changing the event or pick ordering, because the prototype currently uses
  `event_index` to select `plan["picks"][event_index]`.
- Changing the plan's `picks` shape, cube category label, detection IDs, or
  coordinate representation.
- Changing units or coordinate frames for plan XY coordinates or retry offsets.
- Raising the retry limit or changing the offset policy without corresponding
  tests and safety review.

If future live recovery is added, keep it in a separate execution integration
until its safety and runtime behavior are validated. It must detect failure
while the simulation is running, obtain fresh sensor/perception evidence, stop
on collisions or uncertain state, validate revised trajectories, enforce a
bounded retry count, and verify recovery before resuming the remaining task.
The current policy prototype provides none of those live control guarantees.

## Tests

Run the policy tests from `fanuc_er4ia_sim/`:

```bash
.venv/bin/python -m unittest test_recovery_policy -v
```

These are host-side unit tests only; passing them does not validate Isaac
physics, camera observations, collision handling, or live recovery.
