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
import tkinter as tk
from tkinter import ttk
from typing import Optional, Protocol


NOT_MEASURED = "Not measured"
COUNT_FIELDS = ("detected", "eligible", "attempted", "sorted", "skipped", "failed")
CATEGORIES = {"cube": "Cubes", "cap": "Caps", "vial": "Tubes"}
DEFAULT_RUNS_DIR = Path(__file__).resolve().parent / "fanuc_er4ia_sim" / "runs"


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
                if "stack_support_verified" in row and type(row["stack_support_verified"]) is not bool:
                    raise ValueError(f"{filename}: stack_support_verified must be a boolean")
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
            if report["task_complete"] and (not events or any(not e["placement_verified"] or
                    e.get("stack_support_verified") is False for e in events) or report.get("abort_reason")):
                raise ValueError("execution_report.json: task_complete conflicts with execution results")
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
                counts[label]["sorted"] = sum(e["placement_verified"] for e in attempts)
                counts[label]["failed"] = sum(not e["placement_verified"] or
                                              e.get("stack_support_verified") is False for e in attempts)
            verified = sum(e["placement_verified"] for e in events)
            status.metrics["Verified / reported attempts"] = (
                f"{verified} / {len(events)} ({verified / len(events):.1%})" if events else "0 / 0 (no reported attempts)"
            )
            errors = []
            for event in events:
                failed_checks = [f"{flag}=false" for flag in
                                 ("pickup_verified", "placement_verified", "stack_support_verified")
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

        if timestamps:
            status.generated_at = datetime.fromtimestamp(max(timestamps), timezone.utc)
        return status


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
        self.root = root
        self.source = source
        self.events = queue.Queue(maxsize=1)
        self.stop = threading.Event()
        self.last_received = None
        self.snapshot = None
        self.error = None
        self.alert_rows = {}  # Scoped to the run; repeated IDs update existing rows.

        self.run_id = None
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

        ttk.Label(frame, text="Inventory and verified outcomes", style="Section.TLabel").pack(anchor="w")
        ttk.Label(frame, text="Sorted counts require placement verification. Attempts appear after each pick is reported.").pack(
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
    selection.add_argument("--demo", action="store_true", help="Show explicitly fictional example data")
    parser.add_argument("--runs-dir", type=Path, default=DEFAULT_RUNS_DIR,
                        help="Follow the most recently modified run directory here when --run is omitted")
    args = parser.parse_args(argv)
    source = DemoSource() if args.demo else DigitalTwinSource(args.run, args.runs_dir)
    root = tk.Tk()
    SortingView(root, source)
    root.mainloop()


if __name__ == "__main__":
    main()
