"""Local demo control API for the React operations console.

Run this on the GPU host and reach it through an SSH port forward.  It exposes
only the two checked-in pipeline launchers and validated fault presets; request
data is never interpolated into a shell command.
"""

from __future__ import annotations

import argparse
import json
import mimetypes
import os
import re
import subprocess
import sys
import threading
from datetime import datetime, timezone
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import Any
from urllib.parse import unquote, urlparse

REPO_ROOT = Path(__file__).resolve().parent.parent
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from dashboard_status import read_dashboard_status, status_to_dict  # noqa: E402
from run_artifacts import atomic_write_json, read_json  # noqa: E402
from twin_tls import is_loopback  # noqa: E402


TASKS = {
    "cube_stack": {
        "label": "Cube stacking",
        "description": "Perceive and stack six rotated cubes yellow to blue to red.",
        "script": "run_step_pipeline.sh",
        "videos": ("sort_table.mp4", "sort_side.mp4"),
        "planned_items": 6,
        "experimental": False,
        "faults": (
            {
                "id": "none",
                "label": "Nominal run",
                "description": "Run without an injected simulator fault.",
            },
            {
                "id": "drop",
                "label": "Accidental drop",
                "description": "Open the gripper once during lift/carry, then exercise bounded recovery.",
            },
            {
                "id": "grasp_offset",
                "label": "Grasp offset",
                "description": "Offset one grasp by 4 mm to exercise missed-grasp recovery.",
            },
        ),
    },
    "lego_assembly": {
        "label": "LEGO assembly",
        "description": "Dual-arm assembly of the source-indexed LEGO 77243 step 74–79 slice.",
        "script": "run_lego_pipeline.sh",
        "videos": ("lego_overview.mp4", "lego_side.mp4"),
        "planned_items": 10,
        "experimental": True,
        "faults": (
            {
                "id": "none",
                "label": "Nominal run",
                "description": "This pipeline does not expose fault injection.",
            },
        ),
    },
}

TERMINAL_STATES = frozenset({"succeeded", "failed"})
SAFE_VIDEO = re.compile(r"^[A-Za-z0-9_.-]+\.mp4$")


def utc_now() -> str:
    return datetime.now(timezone.utc).isoformat()


def _tail(path: Path, size: int = 12000) -> str:
    if not path.is_file():
        return ""
    with path.open("rb") as stream:
        stream.seek(0, os.SEEK_END)
        length = stream.tell()
        stream.seek(max(0, length - size))
        return stream.read().decode("utf-8", errors="replace")


class RunManager:
    """Own one pipeline process and translate its run files for the browser."""

    def __init__(self, sim_dir: Path, runs_dir: Path, *, enable_experimental: bool = False):
        self.sim_dir = Path(sim_dir).resolve()
        self.runs_dir = Path(runs_dir).resolve()
        self.enable_experimental = enable_experimental
        self.runs_dir.mkdir(parents=True, exist_ok=True)
        self.control_dir = self.runs_dir / ".demo_control"
        self.control_dir.mkdir(exist_ok=True)
        self.lock = threading.RLock()
        self.process: subprocess.Popen | None = None
        self.run_dir: Path | None = None
        self.metadata: dict[str, Any] | None = None
        self._restore_latest()

    def _restore_latest(self) -> None:
        candidates = sorted(
            self.control_dir.glob("*.json"),
            key=lambda path: path.stat().st_mtime_ns,
            reverse=True,
        )
        for path in candidates:
            payload = read_json(path)
            if isinstance(payload, dict) and payload.get("task_type") in TASKS:
                if TASKS[payload["task_type"]]["experimental"] and not self.enable_experimental:
                    continue
                candidate_run = self.runs_dir / str(payload.get("run_id", ""))
                if not candidate_run.is_dir() or candidate_run.parent != self.runs_dir:
                    continue
                self.run_dir = candidate_run
                self.metadata = payload
                if payload.get("state") == "running":
                    pid = payload.get("pid")
                    if type(pid) is int and self._pid_alive(pid):
                        threading.Thread(
                            target=self._watch_external,
                            args=(pid, candidate_run),
                            daemon=True,
                        ).start()
                    else:
                        self._finish_external(candidate_run)
                return

    def _metadata_path(self, run_dir: Path) -> Path:
        return self.control_dir / f"{run_dir.name}.json"

    def _log_path(self, run_dir: Path) -> Path:
        return self.control_dir / f"{run_dir.name}.log"

    @staticmethod
    def _pid_alive(pid: int) -> bool:
        try:
            os.kill(pid, 0)
        except ProcessLookupError:
            return False
        except PermissionError:
            return True
        return True

    def _finish_external(self, run_dir: Path) -> None:
        metadata_path = self._metadata_path(run_dir)
        payload = read_json(metadata_path, {})
        task_type = payload.get("task_type")
        report = read_json(run_dir / "execution_report.json", {})
        videos_ready = task_type in TASKS and all(
            (run_dir / name).is_file() and (run_dir / name).stat().st_size
            for name in TASKS[task_type]["videos"]
        )
        succeeded = bool(report.get("task_complete")) and videos_ready
        payload.update(
            state="succeeded" if succeeded else "failed",
            returncode=0 if succeeded else None,
            finished_at=utc_now(),
        )
        if not succeeded:
            payload["message"] = "The API restarted and the prior pipeline ended without complete review artifacts."
        atomic_write_json(metadata_path, payload)
        with self.lock:
            if self.run_dir == run_dir:
                self.metadata = payload

    def _watch_external(self, pid: int, run_dir: Path) -> None:
        while self._pid_alive(pid):
            threading.Event().wait(1.0)
        self._finish_external(run_dir)

    def capabilities(self) -> dict[str, Any]:
        with self.lock:
            busy = self._process_state() == "running"
        tasks = []
        for task_id, task in TASKS.items():
            if task["experimental"] and not self.enable_experimental:
                continue
            tasks.append(
                {
                    "id": task_id,
                    "label": task["label"],
                    "description": task["description"],
                    "experimental": task["experimental"],
                    "planned_items": task["planned_items"],
                    "faults": list(task["faults"]),
                }
            )
        return {"tasks": tasks, "busy": busy}

    def _validate_request(self, payload: Any) -> tuple[str, str, int]:
        if not isinstance(payload, dict):
            raise ValueError("Request body must be a JSON object")
        task_type = payload.get("task_type")
        if task_type not in TASKS:
            raise ValueError("Unknown task_type")
        if TASKS[task_type]["experimental"] and not self.enable_experimental:
            raise ValueError("Unknown task_type")
        fault = payload.get("fault", "none")
        allowed = {item["id"] for item in TASKS[task_type]["faults"]}
        if fault not in allowed:
            raise ValueError(f"Fault {fault!r} is not available for {task_type}")
        pick_index = payload.get("pick_index", 0)
        if type(pick_index) is not int or not 0 <= pick_index < TASKS[task_type]["planned_items"]:
            raise ValueError("pick_index is outside the selected task")
        return task_type, fault, pick_index

    def start(self, payload: Any) -> dict[str, Any]:
        task_type, fault, pick_index = self._validate_request(payload)
        with self.lock:
            if self._process_state() == "running":
                raise RuntimeError("A demonstration is already running")
            stamp = datetime.now(timezone.utc).strftime("%Y%m%d_%H%M%S_%f")
            run_dir = self.runs_dir / f"demo_{task_type}_{stamp}"
            run_dir.mkdir(parents=False, exist_ok=False)
            task = TASKS[task_type]
            metadata = {
                "run_id": run_dir.name,
                "task_type": task_type,
                "task_label": task["label"],
                "fault": fault,
                "pick_index": pick_index if fault != "none" else None,
                "state": "starting",
                "started_at": utc_now(),
                "finished_at": None,
                "returncode": None,
            }
            atomic_write_json(self._metadata_path(run_dir), metadata)
            command = ["bash", str(self.sim_dir / task["script"]), str(run_dir)]
            env = os.environ.copy()
            env["TWIN_STREAM_DISABLED"] = "1"
            env["TWIN_STREAM_POST_RUN_SEC"] = "0"
            # A nominal request must stay nominal even if the operator's shell
            # previously exported a manual test fault.
            env.pop("INJECT_MISSED_GRASP", None)
            if fault == "drop":
                env["INJECT_MISSED_GRASP"] = f"{pick_index}:drop"
            elif fault == "grasp_offset":
                env["INJECT_MISSED_GRASP"] = f"{pick_index}:4:0:0"
            log_path = self._log_path(run_dir)
            log_handle = log_path.open("ab", buffering=0)
            try:
                process = subprocess.Popen(
                    command,
                    cwd=self.sim_dir,
                    env=env,
                    stdout=log_handle,
                    stderr=subprocess.STDOUT,
                    start_new_session=True,
                )
            except Exception:
                log_handle.close()
                raise
            metadata.update(state="running", pid=process.pid)
            atomic_write_json(self._metadata_path(run_dir), metadata)
            self.process = process
            self.run_dir = run_dir
            self.metadata = metadata
            threading.Thread(
                target=self._watch,
                args=(process, run_dir, log_handle),
                daemon=True,
            ).start()
            return dict(metadata)

    def _watch(self, process: subprocess.Popen, run_dir: Path, log_handle) -> None:
        returncode = process.wait()
        log_handle.close()
        with self.lock:
            metadata_path = self._metadata_path(run_dir)
            payload = read_json(metadata_path, {})
            payload.update(
                state="succeeded" if returncode == 0 else "failed",
                returncode=returncode,
                finished_at=utc_now(),
            )
            atomic_write_json(metadata_path, payload)
            if self.process is process:
                self.metadata = payload

    def _process_state(self) -> str:
        if self.process is not None and self.process.poll() is None:
            return "running"
        if self.metadata:
            return self.metadata.get("state", "failed")
        return "idle"

    def _cube_status(self, run_dir: Path) -> dict[str, Any]:
        payload = status_to_dict(read_dashboard_status(run_dir))
        payload.pop("frame_image", None)
        return payload

    def _lego_status(self, run_dir: Path) -> dict[str, Any]:
        report = read_json(run_dir / "execution_report.json", {})
        manifest = read_json(run_dir / "run_manifest.json", {})
        events = report.get("events", []) if isinstance(report, dict) else []
        if not isinstance(events, list):
            events = []
        planned = manifest.get("requested_task_count", TASKS["lego_assembly"]["planned_items"])
        completed = sum(bool(event.get("placement_verified")) for event in events if isinstance(event, dict))
        alerts = []
        failure = report.get("failure") if isinstance(report, dict) else None
        if failure:
            alerts.append({"id": "pipeline-failure", "object_id": "LEGO assembly", "category": "Pipeline", "outcome": "Failed", "reason": str(failure)})
        misses = report.get("tracking_gate_misses", []) if isinstance(report, dict) else []
        for index, miss in enumerate(misses if isinstance(misses, list) else []):
            alerts.append({"id": f"tracking-{index}", "object_id": "Tracking gate", "category": "Assembly", "outcome": "Failed", "reason": str(miss)})
        last = events[-1] if events and isinstance(events[-1], dict) else {}
        generated = None
        report_path = run_dir / "execution_report.json"
        if report_path.is_file():
            generated = datetime.fromtimestamp(report_path.stat().st_mtime, timezone.utc).isoformat()
        return {
            "run_id": run_dir.name,
            "status": "Completed" if report.get("task_complete") else ("Assembly in progress" if report else "Initializing simulation"),
            "stage": "Final execution report" if report.get("task_complete") else "Dual-arm physical assembly",
            "source": str(run_dir),
            "simulated": True,
            "demonstration": False,
            "finished": bool(report.get("task_complete")),
            "generated_at": generated,
            "counts": {
                "LEGO pieces": {
                    "planned": planned,
                    "attempted": len(events),
                    "verified pickups": report.get("verified_pickups") if report else None,
                    "verified placements": report.get("verified_placements") if report else None,
                    "failed": max(0, len(events) - completed),
                }
            },
            "metrics": {
                "Connector locks": report.get("connector_locks") if report else None,
                "IK failures": report.get("ik_failures") if report else None,
                "Teleports during execution": report.get("teleports_during_execution") if report else None,
            },
            "alerts": alerts,
            "controller_phase": "complete" if report.get("task_complete") else ("executing" if report else "starting"),
            "current_step": f"{last.get('task', 'Task')} · {last.get('part', 'piece')}" if last else None,
        }

    def status(self) -> dict[str, Any]:
        with self.lock:
            run_dir = self.run_dir
            metadata = dict(self.metadata) if self.metadata else None
            state = self._process_state()
        if run_dir is None or metadata is None:
            return {
                "run_id": None,
                "status": "Ready for a demonstration",
                "stage": "Idle",
                "source": str(self.runs_dir),
                "simulated": True,
                "demonstration": False,
                "finished": False,
                "generated_at": None,
                "counts": {},
                "metrics": {},
                "alerts": [],
                "controller_phase": "idle",
                "current_step": None,
                "task_type": None,
                "task_label": None,
                "process": {"state": "idle", "returncode": None, "started_at": None, "finished_at": None},
                "media": {"render_state": "waiting", "videos": [], "bundle_url": None},
                "log_tail": "",
            }
        try:
            if metadata["task_type"] == "cube_stack":
                payload = self._cube_status(run_dir)
            else:
                payload = self._lego_status(run_dir)
        except Exception as exc:
            payload = {
                "run_id": run_dir.name,
                "status": "Waiting for valid run output",
                "stage": "Pipeline startup",
                "source": str(run_dir),
                "simulated": True,
                "demonstration": False,
                "finished": False,
                "generated_at": None,
                "counts": {},
                "metrics": {},
                "alerts": [{"id": "status-read", "object_id": "Run files", "category": "Pipeline", "outcome": "Waiting", "reason": str(exc)}],
                "controller_phase": "starting",
                "current_step": None,
            }
        if state == "failed":
            payload["status"] = "Pipeline failed"
            payload["finished"] = True
            payload.setdefault("alerts", []).append(
                {"id": "process-exit", "object_id": "Demo process", "category": "Pipeline", "outcome": "Failed", "reason": metadata.get("message") or f"Pipeline exited with code {metadata.get('returncode')}"}
            )
        elif state == "succeeded":
            payload["finished"] = True
        videos = self.ready_video_names()
        if videos:
            render_state = "ready"
        elif state == "running":
            render_state = "rendering"
        elif state in TERMINAL_STATES:
            render_state = "failed"
        else:
            render_state = "waiting"
        bundle = run_dir / "run_bundle.zip"
        payload.update(
            task_type=metadata["task_type"],
            task_label=metadata["task_label"],
            fault=metadata.get("fault", "none"),
            pick_index=metadata.get("pick_index"),
            process={
                "state": state,
                "returncode": metadata.get("returncode"),
                "started_at": metadata.get("started_at"),
                "finished_at": metadata.get("finished_at"),
            },
            media={
                "render_state": render_state,
                "videos": [{"name": name, "url": f"/api/video/{name}"} for name in videos],
                "bundle_url": "/api/bundle" if bundle.is_file() else None,
            },
            log_tail=_tail(self._log_path(run_dir)),
        )
        return payload

    def ready_video_names(self) -> list[str]:
        """Expose review media only after the complete pipeline success gate."""
        if self.run_dir is None or not self.metadata or self._process_state() != "succeeded":
            return []
        report = read_json(self.run_dir / "execution_report.json", {})
        if report.get("task_complete") is not True:
            return []
        expected = TASKS[self.metadata["task_type"]]["videos"]
        if not all((self.run_dir / name).is_file() and (self.run_dir / name).stat().st_size for name in expected):
            return []
        return list(expected)

    def video_path(self, name: str) -> Path | None:
        if not SAFE_VIDEO.fullmatch(name) or self.run_dir is None or not self.metadata:
            return None
        if name not in self.ready_video_names():
            return None
        path = self.run_dir / name
        return path if path.is_file() and path.stat().st_size else None

    def bundle_path(self) -> Path | None:
        if self.run_dir is None:
            return None
        path = self.run_dir / "run_bundle.zip"
        return path if path.is_file() and path.stat().st_size else None


class DemoAPIHandler(BaseHTTPRequestHandler):
    manager: RunManager
    server_version = "FANUCDemoAPI/1.0"

    def log_message(self, format, *args):  # noqa: A003
        return

    def _cors(self) -> None:
        origin = self.headers.get("Origin", "")
        if origin.startswith("http://localhost:") or origin.startswith("http://127.0.0.1:"):
            self.send_header("Access-Control-Allow-Origin", origin)
            self.send_header("Vary", "Origin")
        self.send_header("Access-Control-Allow-Methods", "GET, POST, OPTIONS")
        self.send_header("Access-Control-Allow-Headers", "Content-Type")

    def _json(self, payload: Any, status: int = 200) -> None:
        body = json.dumps(payload, indent=2).encode("utf-8")
        self.send_response(status)
        self._cors()
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Cache-Control", "no-store")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def _file(self, path: Path, *, attachment: bool = False) -> None:
        size = path.stat().st_size
        start, end = 0, size - 1
        range_header = self.headers.get("Range")
        if range_header:
            match = re.fullmatch(r"bytes=(\d*)-(\d*)", range_header.strip())
            if not match:
                self.send_response(416)
                self.end_headers()
                return
            if not match.group(1) and not match.group(2):
                self.send_response(416)
                self.end_headers()
                return
            if not match.group(1):
                suffix_length = int(match.group(2))
                if suffix_length <= 0:
                    self.send_response(416)
                    self.end_headers()
                    return
                start = max(0, size - suffix_length)
                end = size - 1
            else:
                start = int(match.group(1))
            if match.group(1) and match.group(2):
                end = min(int(match.group(2)), size - 1)
            if start > end or start >= size:
                self.send_response(416)
                self.send_header("Content-Range", f"bytes */{size}")
                self.end_headers()
                return
        length = end - start + 1
        self.send_response(206 if range_header else 200)
        self._cors()
        self.send_header("Content-Type", mimetypes.guess_type(path.name)[0] or "application/octet-stream")
        self.send_header("Content-Length", str(length))
        self.send_header("Accept-Ranges", "bytes")
        self.send_header("Cache-Control", "no-store" if path.suffix == ".png" else "private, max-age=60")
        if range_header:
            self.send_header("Content-Range", f"bytes {start}-{end}/{size}")
        if attachment:
            self.send_header("Content-Disposition", f'attachment; filename="{path.name}"')
        self.end_headers()
        with path.open("rb") as stream:
            stream.seek(start)
            remaining = length
            try:
                while remaining:
                    block = stream.read(min(65536, remaining))
                    if not block:
                        break
                    self.wfile.write(block)
                    remaining -= len(block)
            except (BrokenPipeError, ConnectionResetError):
                # Browsers routinely abandon speculative/range video reads
                # after they have enough metadata or when the user seeks.
                # The next range request resumes independently.
                return

    def do_OPTIONS(self):  # noqa: N802
        self.send_response(204)
        self._cors()
        self.end_headers()

    def do_GET(self):  # noqa: N802
        path = urlparse(self.path).path
        if path == "/api/health":
            self._json({"ok": True})
        elif path == "/api/capabilities":
            self._json(self.manager.capabilities())
        elif path == "/api/status":
            self._json(self.manager.status())
        elif path.startswith("/api/video/"):
            video = self.manager.video_path(unquote(path.removeprefix("/api/video/")))
            if video is None:
                self._json({"error": "Video is not available"}, 404)
            else:
                self._file(video)
        elif path == "/api/bundle":
            bundle = self.manager.bundle_path()
            if bundle is None:
                self._json({"error": "Run bundle is not available"}, 404)
            else:
                self._file(bundle, attachment=True)
        else:
            self._json({"error": "Not found"}, 404)

    def do_POST(self):  # noqa: N802
        if urlparse(self.path).path != "/api/runs":
            self._json({"error": "Not found"}, 404)
            return
        try:
            length = int(self.headers.get("Content-Length", "0"))
            if length <= 0 or length > 65536:
                raise ValueError("Request body must be between 1 and 65536 bytes")
            payload = json.loads(self.rfile.read(length).decode("utf-8"))
            result = self.manager.start(payload)
        except RuntimeError as exc:
            self._json({"error": str(exc)}, 409)
        except (ValueError, UnicodeError, json.JSONDecodeError) as exc:
            self._json({"error": str(exc)}, 400)
        except OSError as exc:
            self._json({"error": f"Could not start pipeline: {exc}"}, 500)
        else:
            self._json(result, 202)


def create_server(
    sim_dir: Path,
    runs_dir: Path,
    host: str,
    port: int,
    *,
    enable_experimental: bool = False,
) -> ThreadingHTTPServer:
    if not is_loopback(host):
        raise ValueError("The demo API only binds to loopback; use an SSH port forward from the React host")
    manager = RunManager(sim_dir, runs_dir, enable_experimental=enable_experimental)
    handler = type("ConfiguredDemoAPIHandler", (DemoAPIHandler,), {"manager": manager})
    return ThreadingHTTPServer((host, port), handler)


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=8765)
    parser.add_argument("--runs-dir", type=Path, default=Path(__file__).resolve().parent / "runs" / "demo")
    parser.add_argument(
        "--enable-experimental",
        action="store_true",
        help="privately expose experimental tasks such as LEGO assembly",
    )
    args = parser.parse_args(argv)
    try:
        server = create_server(
            Path(__file__).resolve().parent,
            args.runs_dir,
            args.host,
            args.port,
            enable_experimental=args.enable_experimental,
        )
    except (ValueError, OSError) as exc:
        parser.error(str(exc))
    print(f"[demo-api] listening at http://{args.host}:{args.port}; runs: {args.runs_dir.resolve()}", flush=True)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        server.server_close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
