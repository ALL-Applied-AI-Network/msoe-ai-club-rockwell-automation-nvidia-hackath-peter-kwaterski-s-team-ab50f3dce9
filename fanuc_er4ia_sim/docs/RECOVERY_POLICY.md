# Recovery policy and simulation replay

## Status and scope

The recovery policy classifies completed execution attempts and can propose
bounded cube-pick plan changes. `run_with_recovery.sh` can execute those plans
in **separate Isaac Sim processes and fresh run directories**.

This is **not live recovery**. A failed attempt is not corrected in the running
simulation. Every replay reconstructs the scene from that run's saved
`scene_state.json`, which is the captured initial state. The workflow does not
command or support a physical robot and must not be presented as hardware or
in-session recovery.

The tool only automatically retries a classified missed cube grasp. Wrong
object, unreachable motion, placement/support failure, collision, detected
drop, uncertain state, and execution crash stop for review.

## Architecture and data flow

The normal pipeline creates a run containing capture/perception/planning
artifacts, executes `sort_scene.py`, and writes `execution_events.json` and
`execution_report.json`. Recovery is started separately:

1. `run_with_recovery.sh RUN_DIR` invokes the host Python policy.
2. If the run has no execution report, it first invokes Isaac with
   `sort_scene.py --physics-only`.
3. `classify_failure(events, plan, report)` identifies the first event failure
   or detects an incomplete/missing report.
4. `decide(...)` chooses a bounded action for a missed grasp. If the action
   changes the plan, the wrapper copies the required run inputs into a new
   sibling run directory (`RUN_DIR_retry1`, then `RUN_DIR_retry2`), writes the
   candidate plan, and invokes a fresh Isaac process.
5. Each attempt receives a `recovery_decision.json`; the base run receives a
   `recovery_log.json` describing the attempt sequence and final outcome.

Retry directories are never overwritten. Start from a clean base run whose
retry directories do not already exist. Review `sort.log` and the JSON reports
for every attempt.

## Failure classification

`classify_event(event)` returns one failure type or `None` for a successful
event. `classify_failure(events, plan=None, report=None)` returns the first
failure with stage, event index, detection ID, and stack level when available.
When plan and report context are supplied, it also returns:

- `execution_crash` if the report is missing or fewer events than planned picks
  were recorded.
- `incomplete` when events do not show an explicit failure but the report does
  not confirm completion.
- `none` only when the report says complete and the expected events exist.

The classifier recognizes:

| Failure | Meaning | Automatic retry |
|---|---|---|
| `untracked_target` | No tracked target was associated with the attempted pick | No |
| `ik_failure` | Recorded trajectory IK failures | No |
| `missed_grasp` | Pickup not verified and no object was lifted | Yes, bounded |
| `wrong_object` | Multiple or unintended objects were lifted | No |
| `placement_failed` | Pickup succeeded but placement did not verify | No |
| `support_failed` | Stack support validation failed | No |
| `collision_detected`, `dropped_object`, `state_uncertain` | Explicit event flags from an execution adapter | No |
| `event_plan_mismatch` | Event detection ID differs from the planned pick at that index | No |
| `execution_crash` | Missing report or fewer events than planned picks | No |
| `incomplete` | No classified event failure, but task completion is not confirmed | No |

The current `sort_scene.py` reports pickup, lift, placement, support, and IK
evidence. It does not emit collision, drop, or uncertain-state flags; the policy
can classify those only if an event supplies them. The current policy does not
detect those conditions itself.

Simulator-tracked object data in execution events is used to classify/report
what happened in the simulation. It must not be used to adjust grasp commands;
retry pose proposals use the plan and, for image fitting, the saved camera image.

## Retry strategy

Only `missed_grasp` is retryable. The default hard limit remains **two retries
per base run**. Actions are attempted in this order:

1. **`remeasure`** — refit cube pose from the run's saved `world_camera.png` and
   `camera.json`, using image geometry and the existing minimum pose-fit gate.
   If the saved image is unavailable or the fit is inadequate, skip this rung.
   This is a re-analysis of a frozen image, not a fresh post-failure
   observation.
2. **`flip_spread`** — rotate the grasp direction by 90 degrees and update the
   corresponding yaw/spread fields.
3. **`nudge`** — move the grasp target 2 mm along the planned jaw direction.

Rungs that cannot produce a plan are skipped without consuming a retry. Each
actual replay consumes one retry. With the two-retry limit, a normal sequence
may use remeasure and flip; if remeasure is skipped, a later replay can use the
nudge rung. The retry count and rung position are stored in the plan to prevent
loops.

All policy helpers return copied plans and preserve unmodified picks. Retry
plans pass the existing `sort_scene.validate_plan` gate before files are
created. Isaac still validates the plan again before motion. No retry is
attempted for a different object or unsafe/uncertain failure.

## Commands

From `fanuc_er4ia_sim/` after creating and executing an ordinary pipeline run:

```bash
ISAAC=/path/to/isaacsim
bash run_with_recovery.sh runs/stack_01
```

The run must contain a `sort_plan.json`; if it already has an execution report,
the wrapper analyzes that result rather than running the baseline again. To
change the default cap for a test while respecting the hard maximum:

```bash
MAX_RECOVERY_RETRIES=1 ./run_with_recovery.sh runs/stack_01
```

Policy and runner tests:

```bash
.venv/bin/python -m unittest test_recovery_policy test_recovery_runner test_fake_isaac -v
```

`test_fake_isaac` executes the real `sort_scene.py` and recovery runner against
a deliberately simplified fake Isaac API (not Isaac physics). To run a manual
CLI test in WSL, create a synthetic run with a bad grasp and replay it:

```bash
.venv/bin/python fake_isaac/make_fake_run.py runs/recovery_fault --fault 2:2:0:0
chmod +x fake_isaac/python.sh
HOST_PYTHON="$PWD/.venv/bin/python" ISAAC="$PWD/fake_isaac" \
  bash run_with_recovery.sh runs/recovery_fault
```

The fake launcher bypasses the real Isaac installation and physics engine. It
is for checking the plan/replay/execution plumbing only; passing it is not
evidence that a trajectory is safe or that Isaac/PhysX behaves the same way.

These are host-side tests. They do not prove Isaac physics or sensor
performance. A completed retry sequence is not evidence of live recovery.

Summarize a run or calculate recovery rate from multiple base-run directories:

```bash
.venv/bin/python recovery_policy.py --summarize runs/stack_01
.venv/bin/python recovery_policy.py --rate runs/stack_01 runs/stack_02
```

The rate counts base runs whose first classified failure was `missed_grasp`,
then reports the fraction that eventually completed after replay. It excludes
other first-failure types; it is not a per-pick success rate.

The synthetic-run `--fault` argument is
`PICK_INDEX:DX_MM:DY_MM:DYAW_DEG`; `PICK_INDEX` is the zero-based position in
the plan's `picks` list. For example, `2:2:0:0` offsets the third planned grasp
by 2 mm in X to exercise image-based remeasurement.

## Interfaces and change impact

The policy consumes plain plan/event/report dictionaries, optional decoded
image/camera objects, and emits candidate plans. The runner reads/writes run
JSON and launches Isaac; it does not change `sort_scene.py`, `plan_sort.py`,
`run_sort_pipeline.sh`, or the GUI.

Changes likely to affect recovery include:

- Event fields or semantics such as `tracked_object`, `objects_lifted`,
  `pickup_verified`, `placement_verified`, `stack_support_verified`, and
  `trajectory_ik_failures`.
- Plan/event ordering or detection identifiers used to associate a failed
  event with `plan["picks"]`.
- Plan fields, coordinate units/frames, cube orientation, or
  `sort_scene.validate_plan` gates.
- Run artifact names or the source-scene reset semantics.
- Retry limits or rung order.

Keep tests and this document aligned whenever those interfaces change. Live
recovery would require a distinct execution-loop design with current-state
sensing, collision/contact detection, validated recovery trajectories, and
stop behavior; this replay runner does not supply those capabilities.
