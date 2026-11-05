# Local React demonstration console

The console mirrors the Tkinter dashboard's run identity, pipeline stage,
controller step, verified inventory/outcomes, metrics, alerts, and finished
review videos. It also starts a validated demonstration through a
small control API on the Brev GPU host.

The public console exposes cube stacking only. The newer LEGO pipeline remains
disabled in both the task catalog and the run endpoint unless the API operator
explicitly enables experimental tasks.

## One-command startup from WSL

With the Brev instance and its Isaac container already running, start the whole
demo from the repository root:

```bash
bash scripts/start-demo.sh
```

The launcher reuses a healthy backend or starts it when absent, establishes the
local port forward when needed, installs the React dependencies, and opens the
Vite application. Press Ctrl+C to stop React and any tunnel created by the
launcher. The backend remains running on Brev for the next session.

The defaults match the team's current instance and checkout. They can be
overridden when necessary, for example:

```bash
BREV_INSTANCE=another-instance DEMO_WEB_PORT=5174 bash scripts/start-demo.sh
```

## 1. Start the API on Brev

Configure Isaac Sim and Cosmos as described in `fanuc_er4ia_sim/SETUP.md`, then:

```bash
source scripts/env.local.sh
fanuc_er4ia_sim/.venv/bin/python fanuc_er4ia_sim/demo_api.py
```

The API binds to `127.0.0.1:8765`. It deliberately refuses a public plaintext
binding. Keep this process running while people use the demo.

## 2. Forward the API to the laptop

From the laptop, use the same Brev SSH destination you normally use:

```bash
ssh -N -L 8765:127.0.0.1:8765 user@your-brev-host
```

This makes the Brev API available to the local Vite proxy without exposing a
public control port.

## 3. Run React locally

In another laptop terminal:

```bash
cd demo/frontend
npm install
npm run dev
```

Open <http://127.0.0.1:5173>. Vite proxies `/api` to the forwarded Brev port.
To use another local forwarded port or API host, set `DEMO_API_URL` before
starting Vite.

## Demo behavior

- Only one demonstration can run at a time.
- Cube runs can be nominal, inject one accidental drop, or inject one 4 mm
  grasp offset on a selected pick. Both injected cases use the simulator's
  existing bounded missed-grasp recovery path.
- While a run is active, the media panel shows progress without live imagery.
  MP4 playback is exposed only after the pipeline exits successfully, the
  execution report declares the task complete, and both expected review videos
  have finished rendering. It then offers table/side views.
- Run files stay under `fanuc_er4ia_sim/runs/demo/`. API process metadata and
  logs are kept in that directory's `.demo_control/` folder so each simulator
  run directory remains empty at pipeline start.
- The API accepts only checked-in task IDs and fault presets. It never executes
  a request value as a command or accepts a caller-supplied filesystem path.

Build verification:

```bash
cd demo/frontend && npm run build
cd ../../fanuc_er4ia_sim && python3 -m unittest test_demo_api
```
