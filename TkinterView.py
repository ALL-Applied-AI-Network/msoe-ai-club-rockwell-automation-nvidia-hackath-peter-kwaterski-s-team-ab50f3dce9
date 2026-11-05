"""Read-only desktop dashboard for the FANUC digital twin's run artifacts.

Run with ``python3 TkinterView.py --run fanuc_er4ia_sim/runs/stack_01``.
Without --run, follow the most recently modified run directory. No Isaac Sim
or Cosmos imports are needed on the machine displaying the dashboard.
"""

import argparse
from dataclasses import dataclass, field
from datetime import datetime, timezone
import json
import math
from pathlib import Path
import queue
import threading
import time
from typing import Optional, Protocol


NOT_MEASURED = "Not measured"
COUNT_FIELDS = ("detected", "eligible", "attempted", "sorted", "skipped", "failed")
CATEGORIES = {"cube": "Cubes", "cap": "Caps", "vial": "Tubes"}
EXECUTION_CHECKS = ("pickup_verified", "placement_verified", "placement_pose_verified", "stack_support_verified")
DEFAULT_RUNS_DIR = Path(__file__).resolve().parent / "fanuc_er4ia_sim" / "runs"
BUNDLE_DOWNLOAD_TIMEOUT_S = 600.0


def downloads_directory() -> Path:
    """Best-effort location for saved run bundles on the viewing machine."""
    home = Path.home()
    for name in ("Downloads", "downloads"):
        candidate = home / name
        if candidate.is_dir():
            return candidate
    return home


@dataclass
class Alert:
    id: str
    object_id: str
    category: str
    outcome: str
    reason: str


@dataclass
class ViewStatus:
    """Presentation data, independent of Tk widgets and simulator imports."""

    run_id: str
    status: str
    stage: str
    source: str
    simulated: bool
    generated_at: Optional[datetime] = None
    counts: dict = field(default_factory=dict)
    metrics: dict = field(default_factory=dict)
    alerts: list = field(default_factory=list)
    demonstration: bool = False
    finished: bool = False
    controller_phase: Optional[str] = None
    current_step: Optional[str] = None
    frame_url: Optional[str] = None
    frame_image: Optional[bytes] = field(default=None, repr=False)
    bundle_url: Optional[str] = None



class StatusSource(Protocol):
    """Replaceable, read-only adapter, called only on the worker thread."""

    def read_status(self) -> ViewStatus: ...



class DigitalTwinSource:
    """Read the existing pipeline contract; never read evaluation ground truth.

    execution_events.json is published after each pick; execution_report.json
    is authoritative once present. Missing artifacts indicate a pending stage.
    Malformed/partially written artifacts raise an error, so the view retains
    its last good snapshot and retries on the next poll.
    """

    def __init__(self, run=None, runs_dir=DEFAULT_RUNS_DIR):
        self.run = Path(run).expanduser().resolve() if run is not None else None
        self.runs_dir = Path(runs_dir).expanduser().resolve()

    def _run_directory(self):
        if self.run is not None:
            if self.run.exists() and not self.run.is_dir():
                raise ValueError(f"Expected a run directory: {self.run}")
            return self.run
        if not self.runs_dir.exists():
            return None
        directories = [p for p in self.runs_dir.iterdir() if p.is_dir()]
        return max(directories, key=lambda p: (p.stat().st_mtime_ns, p.name), default=None)

    @staticmethod
    def _read(path, expected_type, timestamps):
        try:
            with path.open(encoding="utf-8") as stream:
                stamp = path.stat().st_mtime
                value = json.load(stream)
        except FileNotFoundError:
            return None
        except (ValueError, UnicodeError) as exc:
            raise ValueError(f"{path.name}: invalid or incomplete JSON; retrying: {exc}") from exc
        if not isinstance(value, expected_type):
            raise ValueError(f"{path.name}: expected a JSON {expected_type.__name__}")
        timestamps.append(stamp)
        return value

    @staticmethod
    def _placement_verified(event):
        return all(event.get(flag, True) is True for flag in EXECUTION_CHECKS)

    @staticmethod
    def _objects(document, key, filename, *, planned=False, executed=False):
        rows = document.get(key)
        if not isinstance(rows, list):
            raise ValueError(f"{filename}: '{key}' must be a list")
        seen = set()
        for index, row in enumerate(rows):
            if not isinstance(row, dict) or row.get("category") not in CATEGORIES:
                raise ValueError(f"{filename}: invalid object/category at {key}[{index}]")
            if planned or executed:
                identifier = row.get("detection_id")
                if type(identifier) is not int or identifier < 0 or identifier in seen:
                    raise ValueError(f"{filename}: invalid/duplicate detection_id at {key}[{index}]")
                seen.add(identifier)
            if planned and type(row.get("eligible")) is not bool:
                raise ValueError(f"{filename}: eligible must be a boolean at {key}[{index}]")
            if executed:
                for flag in ("pickup_verified", "placement_verified"):
                    if type(row.get(flag)) is not bool:
                        raise ValueError(f"{filename}: {flag} must be a boolean at {key}[{index}]")
                if row["placement_verified"] and not row["pickup_verified"]:
                    raise ValueError(f"{filename}: placement verified without a verified pickup")
                for flag in ("placement_pose_verified", "stack_support_verified"):
                    if flag in row and type(row[flag]) is not bool:
                        raise ValueError(f"{filename}: {flag} must be a boolean")
        return rows

    def read_status(self):
        run = self._run_directory()
        counts = {category: dict.fromkeys(COUNT_FIELDS) for category in CATEGORIES.values()}
        status = ViewStatus(
            run_id=run.name if run else "Waiting for a run",
            status="Waiting for digital twin output",
            stage="Capture / initialization",
            source=str(run if run else self.runs_dir),
            simulated=True,
            counts=counts,
            metrics={"Verified / reported attempts": None, "Cycle time (wall clock)": None,
                     "Maximum placement error": None},
        )
        if run is None:
            return status
        timestamps = []
        # These mark capture progress only; object counts never come from scene state.
        for name in ("world_camera.png", "camera.json", "scene_state.json"):
            try:
                timestamps.append((run / name).stat().st_mtime)
            except FileNotFoundError:
                continue
            status.status = "Capture output available; waiting for perception"
            status.stage = "Perception (Cosmos)"

        plan = self._read(run / "sort_plan.json", dict, timestamps)
        if plan is not None:
            observations = self._objects(plan, "observations", "sort_plan.json", planned=True)
            picks = self._objects(plan, "picks", "sort_plan.json", planned=True)
            skipped = self._objects(plan, "skipped", "sort_plan.json", planned=True)
            status.stage = "Physical execution (Isaac Sim)" if picks else "Planning / eligibility checks"
            status.status = "Plan ready; waiting for execution results" if picks else "Blocked: no eligible picks"
            status.metrics["Planned picks"] = len(picks)
            for category, label in CATEGORIES.items():
                counts[label]["eligible"] = sum(o["category"] == category and o["eligible"] for o in observations)
                counts[label]["skipped"] = sum(o["category"] == category for o in skipped)
            for obj in skipped:
                reason = obj.get("skip_reason") or "No skip reason supplied by the planner"
                if not isinstance(reason, str):
                    raise ValueError("sort_plan.json: skip_reason must be text")
                status.alerts.append(Alert(
                    f"skip-{obj['detection_id']}", f"detection-{obj['detection_id']}",
                    CATEGORIES[obj["category"]], "Skipped", reason,
                ))
        else:
            detections = self._read(run / "detections.json", dict, timestamps)
            observations = None if detections is None else self._objects(detections, "objects", "detections.json")
            if observations is not None:
                status.status = "Detections available; waiting for a sort plan"
                status.stage = "Planning"
        if observations is not None:
            for category, label in CATEGORIES.items():
                counts[label]["detected"] = sum(o["category"] == category for o in observations)

        report = self._read(run / "execution_report.json", dict, timestamps)
        if report is not None:
            events = self._objects(report, "picks", "execution_report.json", executed=True)
            if type(report.get("task_complete")) is not bool:
                raise ValueError("execution_report.json: task_complete must be a boolean")
            planned_ids = ({p["detection_id"] for p in picks} if plan is not None else None)
            if report["task_complete"] and (
                    not events or any(not self._placement_verified(e) for e in events)
                    or report.get("abort_reason")
                    or (planned_ids is not None and {e["detection_id"] for e in events} != planned_ids)):
                raise ValueError("execution_report.json: task_complete conflicts with execution results")
            for total, flag in (("verified_pickups", "pickup_verified"),
                                ("verified_placements", "placement_verified")):
                if total in report and (type(report[total]) is not int or
                                        report[total] != sum(e[flag] for e in events)):
                    raise ValueError(f"execution_report.json: {total} conflicts with pick results")
            status.finished = True
            status.status = "Completed" if report["task_complete"] else "Incomplete / aborted"
            status.stage = "Final execution report"
            reason = report.get("abort_reason")
            if reason:
                if not isinstance(reason, str):
                    raise ValueError("execution_report.json: abort_reason must be text")
                status.alerts.append(Alert("run-abort", "Run", "Pipeline", "Aborted", reason))
        else:
            events = self._read(run / "execution_events.json", list, timestamps)
            if events is not None:
                events = self._objects({"picks": events}, "picks", "execution_events.json", executed=True)
                status.status = "Execution results available; waiting for the final report"
                status.stage = "Physical execution / placement verification"
        if events is not None:
            for category, label in CATEGORIES.items():
                attempts = [e for e in events if e["category"] == category]
                counts[label]["attempted"] = len(attempts)
                counts[label]["sorted"] = sum(self._placement_verified(e) for e in attempts)
                counts[label]["failed"] = len(attempts) - counts[label]["sorted"]
            verified = sum(self._placement_verified(e) for e in events)
            status.metrics["Verified / reported attempts"] = (
                f"{verified} / {len(events)} ({verified / len(events):.1%})" if events else "0 / 0 (no reported attempts)"
            )
            errors = []
            for event in events:
                failed_checks = [f"{flag}=false" for flag in EXECUTION_CHECKS
                                 if event.get(flag) is False]
                if failed_checks:
                    status.alerts.append(Alert(
                        f"execution-{event['detection_id']}",
                        event.get("tracked_object") or f"detection-{event['detection_id']}",
                        CATEGORIES[event["category"]], "Failed", "; ".join(failed_checks),
                    ))
                error = event.get("placement_error_mm")
                if error is not None:
                    if type(error) not in (int, float) or not math.isfinite(error) or error < 0:
                        raise ValueError("Execution results: placement_error_mm must be finite and nonnegative")
                    errors.append(error)
            if errors:
                status.metrics["Maximum placement error"] = f"{max(errors):.3f} mm (n={len(errors)} reported picks)"

        step_state = self._read(run / "step_state.json", dict, timestamps)
        if isinstance(step_state, dict):
            status.controller_phase = step_state.get("phase")
            index = step_state.get("current_pick_index")
            if plan is not None and isinstance(index, int):
                picks = plan.get("picks", [])
                if 0 <= index < len(picks):
                    pick = picks[index]
                    level = pick.get("stack_level")
                    colour = pick.get("colour")
                    if level is not None and colour:
                        status.current_step = f"Cube {colour} (level {int(level) + 1})"
                    else:
                        status.current_step = f"Pick index {index + 1} of {len(picks)}"

        if timestamps:
            status.generated_at = datetime.fromtimestamp(max(timestamps), timezone.utc)
        return status


class RemoteStatusSource:
    """Mutual TLS viewer, or loopback HTTP through an existing SSH tunnel."""

    def __init__(
        self,
        base_url: str,
        *,
        timeout: float = 2.0,
        download_timeout: float = BUNDLE_DOWNLOAD_TIMEOUT_S,
        ca=None,
        cert=None,
        key=None,
    ):
        from urllib.parse import urlsplit
        from twin_tls import is_loopback, secure_opener, tls_context

        url = urlsplit(base_url)
        if (url.scheme not in ("http", "https") or not url.hostname
                or url.username is not None or url.password is not None
                or url.query or url.fragment or url.path not in ("", "/")):
            raise ValueError("Use an http(s)://host:port base URL without credentials, path, query or fragment")
        self.secure = url.scheme == "https"
        self.opener = None
        if self.secure:
            self.opener = secure_opener(tls_context(server=False, ca=ca, cert=cert, key=key))
        elif any((ca, cert, key)):
            raise ValueError("TLS credentials require an https:// URL")
        elif not is_loopback(url.hostname):
            raise ValueError("Remote HTTP is unencrypted; use mutual TLS or a loopback SSH tunnel")
        self.base_url = base_url.rstrip("/")
        self.timeout = timeout
        self.download_timeout = download_timeout

    def _open(self, path, *, timeout=None):
        import urllib.request

        wait = self.timeout if timeout is None else timeout
        if self.opener is not None:
            return self.opener.open(f"{self.base_url}{path}", timeout=wait)
        return urllib.request.urlopen(f"{self.base_url}{path}", timeout=wait)

    def read_status(self) -> ViewStatus:
        import urllib.error
        import urllib.request

        from dashboard_status import status_from_dict

        try:
            with self._open("/status") as response:
                payload = json.loads(response.read().decode("utf-8"))
        except urllib.error.URLError as exc:
            raise ConnectionError(f"Remote status unavailable: {exc}") from exc
        if "error" in payload and "run_id" not in payload:
            raise ValueError(payload["error"])
        status = status_from_dict(payload)
        status.source = self.base_url
        try:
            with self._open("/frame") as response:
                status.frame_image = response.read()
        except urllib.error.HTTPError as exc:
            if exc.code != 404:
                raise ConnectionError(f"Remote frame unavailable: {exc}") from exc
        except urllib.error.URLError as exc:
            if self.secure:
                raise ConnectionError(f"Secure frame connection failed: {exc}") from exc
        return status

    def download_bundle(self, destination: Path) -> Path:
        """Fetch ``/bundle.zip`` from the sender into ``destination``."""
        import shutil
        import tempfile
        import urllib.error

        destination = Path(destination)
        destination.parent.mkdir(parents=True, exist_ok=True)
        try:
            with self._open("/bundle.zip", timeout=self.download_timeout) as response:
                with tempfile.NamedTemporaryFile(
                    dir=destination.parent, prefix=f".{destination.name}.", suffix=".part", delete=False
                ) as handle:
                    shutil.copyfileobj(response, handle)
                    partial = Path(handle.name)
        except urllib.error.HTTPError as exc:
            raise ConnectionError(f"Run bundle unavailable: HTTP {exc.code}") from exc
        except urllib.error.URLError as exc:
            raise ConnectionError(f"Run bundle download failed: {exc}") from exc
        partial.replace(destination)
        return destination


class DemoSource:
    """Explicitly selected fictional snapshot, separate from twin results."""

    def __init__(self):
        self.created = datetime.now(timezone.utc)

    def read_status(self):
        return ViewStatus(
            run_id="SIMULATED-RUN-001",
            status="Example run finished with skips",
            stage="Placement verification (example)",
            source="Built-in simulated snapshot — no Brev connection",
            simulated=True,
            demonstration=True,
            finished=True,
            generated_at=self.created,
            counts={
                "Cubes": dict(zip(COUNT_FIELDS, (6, 6, 6, 5, 0, 1))),
                "Caps": dict(zip(COUNT_FIELDS, (2, 0, 0, 0, 2, 0))),
                "Tubes": dict(zip(COUNT_FIELDS, (2, 0, 0, 0, 2, 0))),
            },
            metrics={
                "Verified / attempted": "5 / 6 (83.3%) — simulated",
                "Cycle time": None,
                "Placement error": None,
            },
            alerts=[
                Alert("demo-cube-06", "cube-06", "Cubes", "Failed",
                      "Simulated example: placement verification failed; actual reason comes from the controller."),
                *[Alert(f"demo-{kind}-{i}", f"{kind}-{i}", category, "Skipped",
                        "Simulated example: excluded from the cube-stack task (not_a_cube).")
                  for kind, category in (("cap", "Caps"), ("tube", "Tubes"))
                  for i in (1, 2)],
            ],
        )


def display(value):
    return NOT_MEASURED if value is None else str(value)


class SortingView:
    def __init__(self, root, source: StatusSource):
        import tkinter as tk
        from tkinter import ttk

        self.root = root
        self.source = source
        self.events = queue.Queue(maxsize=1)
        self.stop = threading.Event()
        self.last_received = None
        self.snapshot = None
        self.error = None
        self.alert_rows = {}  # Scoped to the run; repeated IDs update existing rows.

        self.run_id = None
        self._bundle_saved_runs = set()
        self._bundle_download_in_progress = False
        root.title("Robotic sorting | FANUC digital twin")
        root.geometry("1120x760")
        root.minsize(900, 650)
        root.configure(bg="#edf2f7")
        style = ttk.Style(root)
        style.theme_use("clam")
        style.configure("TFrame", background="#edf2f7")
        style.configure("TLabel", background="#edf2f7", font=("Segoe UI", 10))
        style.configure("Title.TLabel", font=("Segoe UI", 22, "bold"))
        style.configure("Section.TLabel", font=("Segoe UI", 12, "bold"))
        style.configure("Banner.TLabel", background="#fff0c2", foreground="#604000", padding=12)

        style.configure("Treeview", rowheight=32, font=("Segoe UI", 10))
        style.configure("Treeview.Heading", font=("Segoe UI", 10, "bold"))

        frame = ttk.Frame(root, padding=22)
        frame.pack(fill="both", expand=True)
        ttk.Label(frame, text="Robotic sorting", style="Title.TLabel").pack(anchor="w")
        ttk.Label(frame, text="Read-only digital twin dashboard • pipeline run files").pack(anchor="w", pady=(0, 12))

        self.banner = ttk.Label(frame, style="Banner.TLabel")
        self.banner.pack(fill="x")
        self.run_text = tk.StringVar(value="Waiting for a snapshot…")
        ttk.Label(frame, textvariable=self.run_text, justify="left", wraplength=1000).pack(anchor="w", pady=12)
        self.fresh_text = tk.StringVar(value="No source read yet")
        ttk.Label(frame, textvariable=self.fresh_text, wraplength=1000).pack(anchor="w", pady=(0, 12))
        self.connection_text = tk.StringVar(value="")
        ttk.Label(frame, textvariable=self.connection_text, wraplength=1000).pack(anchor="w", pady=(0, 8))
        self.bundle_text = tk.StringVar(value="")
        ttk.Label(frame, textvariable=self.bundle_text, wraplength=1000).pack(anchor="w", pady=(0, 8))

        ttk.Label(frame, text="Inventory and verified outcomes", style="Section.TLabel").pack(anchor="w")
        ttk.Label(frame, text="Sorted counts require all reported placement checks to pass. Attempts appear after each pick.").pack(
            anchor="w", pady=(2, 8))
        self.counts = ttk.Treeview(frame, columns=("category",) + COUNT_FIELDS, show="headings", height=3,
                                   selectmode="none")
        headings = ("Category", "Detected", "Eligible", "Attempted", "Verified sorted", "Skipped", "Failed")

        for key, title in zip(self.counts["columns"], headings):
            self.counts.heading(key, text=title)
            self.counts.column(key, width=135, anchor="center")
        for category in ("Cubes", "Caps", "Tubes"):
            self.counts.insert("", "end", iid=category, values=(category,) + (NOT_MEASURED,) * 6)
        self.counts.pack(fill="x")

        ttk.Label(frame, text="Available metrics", style="Section.TLabel").pack(anchor="w", pady=(16, 4))
        self.metric_text = tk.StringVar()
        ttk.Label(frame, textvariable=self.metric_text).pack(anchor="w")
        ttk.Label(frame, text="Object alerts", style="Section.TLabel").pack(anchor="w", pady=(16, 6))
        alert_frame = ttk.Frame(frame)
        alert_frame.pack(fill="both", expand=True)
        self.alerts = ttk.Treeview(alert_frame, columns=("object", "category", "outcome", "reason"), show="headings",
                                   height=5)
        for key, title, width in (("object", "Object", 100), ("category", "Category", 80),
                                  ("outcome", "Outcome", 85), ("reason", "Reason (select for full text)", 620)):
            self.alerts.heading(key, text=title)
            self.alerts.column(key, width=width, minwidth=60)
        scroll = ttk.Scrollbar(alert_frame, orient="vertical", command=self.alerts.yview)
        self.alerts.configure(yscrollcommand=scroll.set)
        self.alerts.pack(side="left", fill="both", expand=True)
        scroll.pack(side="right", fill="y")
        self.detail = tk.StringVar(value="Select an alert to read its full reason.")
        self.step_text = tk.StringVar(value="")
        ttk.Label(frame, textvariable=self.step_text, wraplength=1000).pack(anchor="w", pady=(0, 8))
        self.live_frame = ttk.Label(frame)
        self.live_frame.pack(anchor="w", pady=(0, 8))
        self._photo = None

        ttk.Label(frame, textvariable=self.detail, wraplength=1000).pack(anchor="w", pady=8)
        self.alerts.bind("<<TreeviewSelect>>", self.show_alert)
        ttk.Label(frame, text="The cube-stack task excludes caps and tubes as distractors.\n"
                  "Reads local or synchronized run files. File updates reflect pipeline stages, not a simulator heartbeat.",
                  foreground="#555555").pack(anchor="w", pady=(6, 0))

        # The worker never accesses Tk, including StringVar or root.after.
        threading.Thread(target=self.poll_source, daemon=True).start()
        self.timer = root.after(100, self.drain_events)
        root.protocol("WM_DELETE_WINDOW", self.close)


    def poll_source(self):
        while not self.stop.is_set():
            try:
                event = (self.source.read_status(), None)
            except Exception as exc:
                event = (None, f"{type(exc).__name__}: {exc}")
            try:
                self.events.put_nowait(event)
            except queue.Full:
                # Keep the newest snapshot if the UI is temporarily busy.
                try:
                    self.events.get_nowait()
                except queue.Empty:
                    pass
                self.events.put_nowait(event)
            self.stop.wait(1.0)

    def drain_events(self):
        try:
            status, self.error = self.events.get_nowait()
            if status is not None:
                self.last_received = time.monotonic()
                self.snapshot = status
                self.render(status)
                self.maybe_download_bundle(status)
        except queue.Empty:
            pass

        self.update_freshness()
        self.timer = self.root.after(100, self.drain_events)

    def render(self, status):
        if status.demonstration:
            banner = "SIMULATED DEMONSTRATION — fictional counts and alerts"
        elif status.simulated:
            banner = "DIGITAL TWIN — Isaac Sim pipeline results from run files"
        else:
            banner = "STATUS SNAPSHOT — " + status.source
        self.banner.configure(text=banner)
        self.run_text.set(
            f"Run: {status.run_id}\nStatus: {status.status}\nPipeline stage: {status.stage}\nSource: {status.source}")
        step_parts = []
        if status.controller_phase:
            step_parts.append(f"Controller phase: {status.controller_phase}")
        if status.current_step:
            step_parts.append(f"Current step: {status.current_step}")
        self.step_text.set(" | ".join(step_parts))
        if status.frame_image:
            import os
            import tempfile
            import tkinter as tk

            with tempfile.NamedTemporaryFile(suffix=".png", delete=False) as handle:
                handle.write(status.frame_image)
                path = handle.name
            try:
                photo = tk.PhotoImage(file=path)
            finally:
                os.unlink(path)
            self._photo = photo
            self.live_frame.configure(image=photo)
        else:
            self.live_frame.configure(image="")
            self._photo = None
        for category in CATEGORIES.values():
            counts = status.counts.get(category, {})
            self.counts.item(category, values=(category,) + tuple(display(counts.get(k)) for k in COUNT_FIELDS))
        self.metric_text.set(
            "\n".join(f"{name}: {display(value)}" for name, value in status.metrics.items()) or NOT_MEASURED)

        run_key = (status.run_id, status.source)
        if run_key != self.run_id:
            for row in self.alerts.get_children():
                self.alerts.delete(row)
            self.alert_rows.clear()
            self.detail.set("Select an alert to read its full reason.")
            self.run_id = run_key
            if not isinstance(self.source, RemoteStatusSource):
                self.bundle_text.set("")
            elif status.run_id not in self._bundle_saved_runs:
                self.bundle_text.set("")
        current_ids = {alert.id for alert in status.alerts}
        for identifier in list(self.alert_rows):
            if identifier not in current_ids:
                self.alerts.delete(self.alert_rows.pop(identifier))
        for alert in status.alerts:
            values = (alert.object_id, alert.category, alert.outcome, alert.reason)
            if alert.id in self.alert_rows:
                self.alerts.item(self.alert_rows[alert.id], values=values)
            else:
                self.alert_rows[alert.id] = self.alerts.insert("", "end", values=values)
        if self.alerts.selection():
            self.show_alert()
        else:
            self.detail.set("Select an alert to read its full reason.")

    def update_freshness(self):
        self.update_connection_status()
        if self.error:
            message = "Source error — retaining last snapshot: " if self.snapshot else "Source error — waiting for valid output: "
            self.fresh_text.set(message + self.error)
            return
        if self.last_received is None:
            return
        read_age = time.monotonic() - self.last_received
        stamp = self.snapshot.generated_at
        age = None if stamp is None else (datetime.now(timezone.utc) - stamp).total_seconds()
        age_text = "no output files yet" if age is None else f"last output update {max(0, age):.0f}s ago"

        # Re-reading a file does not refresh its producer time. A completed run
        # is an archived result, and long inference/physics stages can be quiet.
        state = "fixed fictional demo" if self.snapshot.demonstration else (
            "finished run" if self.snapshot.finished else "pipeline file snapshots")
        if read_age > 3:
            state += " • source read delayed"
        self.fresh_text.set(f"Freshness: {state} | {age_text} | last successful read {read_age:.0f}s ago")

    def update_connection_status(self):
        if not isinstance(self.source, RemoteStatusSource):
            self.connection_text.set("Local source — no network transmission")
            return
        if self.source.secure:
            if self.error:
                text = "Secure connection unavailable or rejected — any retained image is from an earlier read"
            elif self.last_received is None:
                text = "Mutual TLS — waiting for an authenticated connection"
            elif time.monotonic() - self.last_received > 3:
                text = "Mutual TLS — authenticated read delayed; connection not currently confirmed"
            else:
                text = "Mutual TLS 1.3 — server verified; client certificate accepted on last read"
        else:
            text = "Loopback HTTP — encryption depends on your SSH tunnel; the dashboard cannot verify it"
        self.connection_text.set(text)

    def maybe_download_bundle(self, status: ViewStatus):
        if not isinstance(self.source, RemoteStatusSource):
            return
        if not status.bundle_url:
            return
        if status.run_id in self._bundle_saved_runs or self._bundle_download_in_progress:
            return
        self._bundle_download_in_progress = True
        self.bundle_text.set(f"Downloading run bundle for {status.run_id}…")
        threading.Thread(
            target=self._download_bundle_worker,
            args=(status.run_id,),
            daemon=True,
        ).start()

    def _download_bundle_worker(self, run_id: str):
        destination = downloads_directory() / f"{run_id}.zip"
        try:
            self.source.download_bundle(destination)
        except Exception as exc:
            message = f"Run bundle download failed: {exc}"
        else:
            self._bundle_saved_runs.add(run_id)
            message = f"Run bundle saved to {destination}"
        finally:
            self._bundle_download_in_progress = False
        self.root.after(0, lambda: self.bundle_text.set(message))


    def show_alert(self, _event=None):
        selected = self.alerts.selection()
        if selected:
            values = self.alerts.item(selected[0], "values")
            self.detail.set(f"{values[0]} — {values[2]}: {values[3]}")

    def close(self):
        self.stop.set()
        self.root.after_cancel(self.timer)
        self.root.destroy()


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    selection = parser.add_mutually_exclusive_group()
    selection.add_argument("--run", type=Path, help="Follow one pipeline run directory, even before it exists")
    selection.add_argument(
        "--url",
        help="Poll https://sender:8765 with mutual TLS, or http://127.0.0.1:8765 via SSH tunnel",
    )
    selection.add_argument("--demo", action="store_true", help="Show explicitly fictional example data")
    parser.add_argument("--runs-dir", type=Path, default=DEFAULT_RUNS_DIR,
                        help="Follow the most recently modified run directory here when --run is omitted")
    parser.add_argument("--tls-ca", type=Path, help="CA certificate trusted to issue the sender certificate")
    parser.add_argument("--tls-cert", type=Path, help="Viewer client certificate")
    parser.add_argument("--tls-key", type=Path, help="Viewer private key")
    args = parser.parse_args(argv)
    if any((args.tls_ca, args.tls_cert, args.tls_key)) and not args.url:
        parser.error("TLS credentials require --url https://...")
    if args.demo:
        source = DemoSource()
    elif args.url:
        try:
            source = RemoteStatusSource(args.url, ca=args.tls_ca, cert=args.tls_cert, key=args.tls_key)
        except (ValueError, OSError) as exc:
            parser.error(str(exc))
    else:
        source = DigitalTwinSource(args.run, args.runs_dir)
    try:
        import tkinter as tk
    except ImportError:
        parser.error("This Python lacks Tk support. Run the dashboard with a desktop Python that includes Tkinter; "
                     "Isaac's Python can still run the adapter tests.")
    try:
        root = tk.Tk()
    except tk.TclError as exc:
        parser.error(f"Cannot open the Tk display: {exc}. Use a desktop display or configure DISPLAY on Brev.")
    SortingView(root, source)
    root.mainloop()


if __name__ == "__main__":
    main()
