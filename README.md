# Rockwell Automation + NVIDIA Innovation Lab · Fall 2026

This is your team's repository for the four-week physical AI hackathon run by Rockwell Automation, NVIDIA, the MSOE AI Club and UWM's Connected Systems Institute. Build everything here. Sponsors can follow your work as it progresses, and your final submission links straight to this repository.

**Full event guide** (datasets, compute, rules, judging, prizes): <https://msoe-maic.com/library/?article=Rockwell-NVIDIA-Fall-2026>
**Your team workspace** (team, repository access, submissions): <https://dashboard.all-ai-network.org/j/L36HEH>
**Questions:** [join the hackathon Slack](https://join.slack.com/t/rockwellnvidi-wq82211/shared_invite/zt-4bp05nsn9-UW3afFtM9wjzWtZiQoJBHg) and ask in the channel that matches your question. The event guide's "Getting help" section says who to ping for what.

## Key dates

| When | What |
|---|---|
| Thu Oct 8, 4:00–6:00 PM | Kickoff at Rockwell Automation HQ, 1201 S 2nd St, Milwaukee |
| **Wed Oct 14, 11:59 PM CT** | **Team plan due:** [submit it here](https://dashboard.all-ai-network.org/s/V28NHK) |
| **Sun Nov 1, 11:59 PM CT** | **Final submission due:** [submit it here](https://dashboard.all-ai-network.org/s/V28NHK) |
| Thu Nov 5, 5:00–8:00 PM | Finals at MSOE Diercks Hall: live presentations and prizes |

## First week

- [ ] Everyone on the team has an ALL AI Network account and has joined this team in the [workspace](https://dashboard.all-ai-network.org/j/L36HEH).
- [ ] Everyone has connected GitHub and accepted the repository invitation (check your GitHub email, or open the workspace and press the check button).
- [ ] Pick your dataset. We recommend one strong solution around a single dataset.
- [ ] Download the data and make sure everyone who needs it can open it.
- [ ] Read the compute guide and decide what you'll need.
- [ ] Join Slack, open `#hackathon-announcements`, and find your dataset's channel.
- [ ] Submit the [team plan](https://dashboard.all-ai-network.org/s/V28NHK) by Wednesday, October 14. It's a short resource request, not a graded paper, and any teammate can update it before the deadline.

## The three datasets

1. **[Final Vial Inspection](https://msoe-maic.com/library?article=Rockwell-NVIDIA-Fall-2026-Vial-Inspection):** find missing caps, crooked caps, damaged vials and unexpected empty positions after the robot fills a fixture.
2. **[Fluid Mixing Station 3](https://msoe-maic.com/library?article=Rockwell-NVIDIA-Fall-2026-Fluid-Mixing):** use synchronized camera views to detect fill and color problems caused by recipes and fluid carryover.
3. **[Grasping and Sorting](https://msoe-maic.com/library?article=Rockwell-NVIDIA-Fall-2026-Grasping-Sorting):** use a digital twin and multiple cameras to make a standalone FANUC arm grasp and sort objects more reliably.

## Read before your team plan

- **[Compute: Brev, DGX Spark and build.nvidia.com](https://msoe-maic.com/library?article=Rockwell-NVIDIA-Fall-2026-Compute).** Brev credits are spent whenever an instance is running, so stop instances you aren't using.
- **[Collect and test your own data](https://msoe-maic.com/library?article=Rockwell-NVIDIA-Fall-2026-Data-Collection):** visits, extra data, digital twins, and booking a supervised test on the real equipment.
- **[Tools and resources](https://msoe-maic.com/library?article=Rockwell-NVIDIA-Fall-2026-Resources):** AI coding tools, vision-language models, NVIDIA Cosmos and more. Start with a simple baseline.

## Final submission (Sun Nov 1)

- A video of about five minutes
- A one-page project summary
- A link to this repository
- References and credits for data, models, code and tools

## Mentee tutorials

Step-by-step Jupyter notebooks (Git, Python, `uv`, and extending the FANUC Isaac/Cosmos pipeline) live in [`tutorials/notebooks/`](tutorials/notebooks/README.md). Start there if you are new to the repo or the Brev GPU workflow.

**Team roadmap, metrics, and environment checklist:** [`docs/TEAM_MENTEE_GUIDE.md`](docs/TEAM_MENTEE_GUIDE.md). Copy [`scripts/env.example.sh`](scripts/env.example.sh) to `env.local.sh`, set paths, and `source` it each session.

**New Brev instance (Isaac already installed):** run once from the repo root:

```bash
bash scripts/setup-brev.sh              # use existing model weights on disk
bash scripts/setup-brev.sh --download-model --verify-model   # first time / fresh disk
```

Then each session: `source scripts/env.local.sh`. Manual steps: [`fanuc_er4ia_sim/SETUP.md`](fanuc_er4ia_sim/SETUP.md), `bash scripts/setup-dev.sh`, `bash scripts/preflight.sh`.

**Git push on Brev:** Cursor/VS Code inject `GIT_ASKPASS` and a git IPC socket that fails in SSH terminals (you see `ECONNREFUSED ... vscode-git-....sock`). After `source scripts/env.local.sh`, push with `bash scripts/git-push.sh` — it clears those variables and prompts for GitHub username + PAT (authorize the token for org SSO if required).

## Digital twin desktop dashboard

For the interactive local React demonstration console (start runs, select cube
fault scenarios and play the completed rendered video),
see [`demo/README.md`](demo/README.md). The control API runs on Brev behind an
SSH tunnel while React runs on the presenting laptop.

From WSL, the complete demo can be launched with `bash scripts/start-demo.sh`.

Run the read-only Tkinter dashboard with a desktop Python that includes Tkinter:

```bash
python3 TkinterView.py --run fanuc_er4ia_sim/runs/stack_01
```

Use the same run folder passed to `fanuc_er4ia_sim/run_sort_pipeline.sh`. The
dashboard can start before the run folder exists. It refreshes once per second,
reading `detections.json`, `sort_plan.json`, `execution_events.json`, and finally
`execution_report.json`. Execution events arrive after each pick; a quiet stage
does not mean the simulator has stopped. Only verified placements count as sorted.
Planner skip reasons, failed execution checks, and the final abort reason appear
in the alerts table. Unavailable measurements stay **Not measured**.

With no arguments, the dashboard follows the most recently modified run folder
under `fanuc_er4ia_sim/runs/`. Use `--runs-dir /your/local/runs` to watch another
parent folder, or `--demo` for explicitly fictional example data. For a twin
running on Brev, synchronize its run folder to the desktop and point `--run` at
that local folder. The dashboard reads files; it does not start or control Isaac
Sim and does not establish a remote connection itself.

Run the dashboard's host tests with `python3 -m unittest test_tkinter_view`.

### Stepwise execution and live laptop stream

On the GPU host, run the stepwise pipeline (one verified cube at a time, with
bounded live recovery on missed grasps inside a single Isaac session):

```bash
cd fanuc_er4ia_sim
export HOST_PYTHON="$PWD/.venv/bin/python"
bash run_step_pipeline.sh runs/stack_step_01
```

This starts a localhost HTTP adapter on port `8765` (override with
`TWIN_STREAM_PORT`) that serves `/status`, `/step_state`, and `/frame`
(`live_frame.png` from the simulator). From your laptop, forward the port over
SSH, then open the dashboard against the tunnel:

```bash
ssh -L 8765:127.0.0.1:8765 user@your-brev-host
python3 TkinterView.py --url http://127.0.0.1:8765
```

For direct encrypted camera/status transmission, the server and dashboard support
**mutual TLS 1.3** with certificates for both sender and viewer. See
[`fanuc_er4ia_sim/docs/SECURE_STREAM.md`](fanuc_er4ia_sim/docs/SECURE_STREAM.md)
for certificate generation, HTTPS commands, pipeline configuration and rejection
tests. Direct non-loopback HTTP is refused; loopback HTTP remains available for
the SSH workflow above. This secures simulated data in transit without changing
camera, perception or motion behavior.

Step state is written to `step_state.json` in the run folder. After a failed
grasp, the simulator captures `mid_step_capture.png` and calls
`live_step_recovery.py` on the host (requires `HOST_PYTHON` and the host
`.venv`). Placement, support, and IK failures stop the run without autonomous
motion retries.

## Bounded recovery replay (cube missed grasp)

After a normal sort run, you can classify execution failures and retry **missed cube grasps** in fresh Isaac processes and sibling run directories (`runs/your_run_retry1`, …). This replays from the saved initial scene; it does **not** fix a failure inside the same live simulation session.

```bash
# From fanuc_er4ia_sim/, with ISAAC set (see SETUP.md):
bash run_with_recovery.sh runs/your_run
```

Details, limits, and local tests without Isaac: [`fanuc_er4ia_sim/docs/RECOVERY_POLICY.md`](fanuc_er4ia_sim/docs/RECOVERY_POLICY.md). Host unit tests: `cd fanuc_er4ia_sim && .venv/bin/python -m unittest test_recovery_policy test_recovery_runner test_fake_isaac`.

## Working in this repository

```bash
git clone <this repository's URL from the green "Code" button>
```

- Commit early and push often. Mentors and judges see what's here.
- **Don't commit large datasets or model weights.** Keep them out with `.gitignore` and note in this README where to download them.
- Before the finals, replace this README with your project: the problem, your approach, results, how to run it, and credits.

**Can't push or see this repository?** Open the [team workspace](https://dashboard.all-ai-network.org/j/L36HEH) and press the GitHub check button. If it still doesn't work, ping Brett Storoe in `#hackathon-general` on Slack.
