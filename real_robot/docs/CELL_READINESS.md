# Cell readiness checklist

Confirm with organizers before relying on real grasps:

- Default scene: eight M6 countersunk screws and yellow bin; task wording matches submission.
- Rollout time limit and scoring (success = all eight in bin).
- Screw dimensions, lying height, and acceptable grasp yaw.
- Yellow bin rim height and safe drop pose inside the bin.
- Gripper close timing (binary 0/1) and whether partial close is supported.
- Reachable workspace vs our TCP envelope; collision behavior during Cartesian moves.
- Whether depth streams are always available at 10 Hz during rollouts.

## Tuning order (supervised)

1. Single screw: localize, hover, approach, close, lift only (no bin).
2. Single screw full cycle into bin.
3. Three screws with re-localize after each drop.
4. Full eight-screw rollout.

Capture RGB/depth from the cell when allowed; tune `config/screw_task.yaml` clearances offline.

## Submission

Fill `submission/submission.yaml` (team, contact, task text). Default scene needs no custom `placement.yaml` rollouts. Archive the git revision and `UPSTREAM.md` pin used for `--submit`.
