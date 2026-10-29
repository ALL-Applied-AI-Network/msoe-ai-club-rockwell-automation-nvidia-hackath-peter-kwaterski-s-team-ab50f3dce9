"""Run sort_scene.py against the local fake Isaac API adapter."""

import json
import runpy
import sys
from pathlib import Path

SIM = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(SIM))

import fakes  # noqa: E402


def main(argv=None):
    args = list(sys.argv[1:] if argv is None else argv)
    if not args or Path(args[0]).name != "sort_scene.py":
        raise SystemExit("fake Isaac supports sort_scene.py only")
    script, script_args = args[0], args[1:]
    try:
        run_dir = Path(script_args[script_args.index("--run") + 1])
    except (ValueError, IndexError) as exc:
        raise SystemExit("sort_scene.py invocation must include --run RUN_DIR") from exc

    scene_state = json.loads((run_dir / "scene_state.json").read_text(encoding="utf-8"))
    ground_truth = json.loads(
        (run_dir / "ground_truth.json").read_text(encoding="utf-8")
    )
    fakes.install(fakes.FakeWorldState(scene_state, ground_truth))
    sys.argv = [script, *script_args]
    runpy.run_path(script, run_name="__main__")


if __name__ == "__main__":
    main()
