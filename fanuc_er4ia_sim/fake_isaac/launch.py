"""Run an Isaac script (e.g. sort_scene.py) against the fakes. Called by fake_isaac/python.sh."""

import json
import runpy
import sys
from pathlib import Path

SIM = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(SIM))

import fakes  # noqa: E402  (fake_isaac/ is on sys.path as the script dir)

script, args = sys.argv[1], sys.argv[2:]
if Path(script).name != "sort_scene.py":
    sys.exit(f"fake_isaac only supports sort_scene.py, not {script}")
run = Path(args[args.index("--run") + 1])
state = fakes.FakeWorldState(json.loads((run / "scene_state.json").read_text()),
                             json.loads((run / "ground_truth.json").read_text()))
fakes.install(state)
sys.argv = [script, *args]
runpy.run_path(script, run_name="__main__")
