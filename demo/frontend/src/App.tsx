import { FormEvent, useCallback, useEffect, useMemo, useState } from "react";

type Fault = { id: string; label: string; description: string };
type Task = {
  id: string;
  label: string;
  description: string;
  experimental: boolean;
  planned_items: number;
  faults: Fault[];
};
type Alert = { id: string; object_id: string; category: string; outcome: string; reason: string };
type Video = { name: string; url: string };
type Status = {
  run_id: string | null;
  status: string;
  stage: string;
  source: string;
  generated_at: string | null;
  counts: Record<string, Record<string, string | number | null>>;
  metrics: Record<string, string | number | null>;
  alerts: Alert[];
  controller_phase: string | null;
  current_step: string | null;
  task_type: string | null;
  task_label: string | null;
  fault?: string;
  pick_index?: number | null;
  process: { state: "idle" | "running" | "succeeded" | "failed"; returncode: number | null; started_at: string | null; finished_at: string | null };
  media: { render_state: "waiting" | "rendering" | "ready" | "failed"; videos: Video[]; bundle_url: string | null };
  log_tail: string;
};

const countOrder = ["detected", "eligible", "planned", "attempted", "verified pickups", "sorted", "verified placements", "skipped", "failed"];

function display(value: unknown) {
  return value === null || value === undefined || value === "" ? "—" : String(value);
}

function formatTime(value: string | null | undefined) {
  if (!value) return "—";
  return new Intl.DateTimeFormat(undefined, { hour: "numeric", minute: "2-digit", second: "2-digit" }).format(new Date(value));
}

function App() {
  const [tasks, setTasks] = useState<Task[]>([]);
  const [status, setStatus] = useState<Status | null>(null);
  const [taskId, setTaskId] = useState("cube_stack");
  const [fault, setFault] = useState("none");
  const [pickIndex, setPickIndex] = useState(0);
  const [selectedVideo, setSelectedVideo] = useState(0);
  const [requestError, setRequestError] = useState("");
  const [starting, setStarting] = useState(false);
  const [connected, setConnected] = useState(false);

  const refresh = useCallback(async () => {
    try {
      const response = await fetch("/api/status", { cache: "no-store" });
      if (!response.ok) throw new Error(`Status request failed (${response.status})`);
      setStatus(await response.json());
      setConnected(true);
    } catch (error) {
      setConnected(false);
      setRequestError(error instanceof Error ? error.message : String(error));
    }
  }, []);

  useEffect(() => {
    fetch("/api/capabilities", { cache: "no-store" })
      .then((response) => {
        if (!response.ok) throw new Error(`Capabilities request failed (${response.status})`);
        return response.json();
      })
      .then((data: { tasks: Task[] }) => {
        setTasks(data.tasks);
        if (data.tasks.length && !data.tasks.some((task) => task.id === taskId)) setTaskId(data.tasks[0].id);
      })
      .catch((error) => setRequestError(String(error)));
    refresh();
    const timer = window.setInterval(refresh, 1000);
    return () => window.clearInterval(timer);
  }, [refresh]);

  const selectedTask = tasks.find((task) => task.id === taskId) ?? tasks[0];
  useEffect(() => {
    if (selectedTask && !selectedTask.faults.some((item) => item.id === fault)) setFault("none");
  }, [selectedTask, fault]);

  useEffect(() => setSelectedVideo(0), [status?.run_id]);

  const countColumns = useMemo(() => {
    const found = new Set<string>();
    Object.values(status?.counts ?? {}).forEach((row) => Object.keys(row).forEach((key) => found.add(key)));
    return [...found].sort((a, b) => {
      const ai = countOrder.indexOf(a);
      const bi = countOrder.indexOf(b);
      return (ai < 0 ? 999 : ai) - (bi < 0 ? 999 : bi) || a.localeCompare(b);
    });
  }, [status?.counts]);

  const isBusy = status?.process.state === "running" || starting;
  const showVideo = Boolean(status?.media.render_state === "ready" && status.media.videos.length);
  const activeVideo = status?.media.videos[selectedVideo] ?? status?.media.videos[0];

  async function startRun(event: FormEvent) {
    event.preventDefault();
    setStarting(true);
    setRequestError("");
    try {
      const response = await fetch("/api/runs", {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify({ task_type: taskId, fault, pick_index: pickIndex }),
      });
      const body = await response.json();
      if (!response.ok) throw new Error(body.error || `Start request failed (${response.status})`);
      await refresh();
    } catch (error) {
      setRequestError(error instanceof Error ? error.message : String(error));
    } finally {
      setStarting(false);
    }
  }

  const state = status?.process.state ?? "idle";

  return (
    <div className="app-shell">
      <header className="topbar">
        <div className="brand-mark" aria-hidden="true"><span /><span /><span /></div>
        <div>
          <p className="eyebrow">ROCKWELL × NVIDIA</p>
          <h1>FANUC Twin Operations</h1>
        </div>
        <div className={`connection ${connected ? "online" : "offline"}`}>
          <span className="connection-dot" />
          {connected ? "Brev link online" : "Brev link offline"}
        </div>
      </header>

      <main>
        <section className="hero-grid">
          <div className="media-panel panel">
            <div className="panel-heading">
              <div>
                <p className="section-kicker">CELL 01 / RENDERED REVIEW</p>
                <h2>{showVideo ? "Completed run playback" : "Completed run video"}</h2>
              </div>
              <span className={`state-pill ${state}`}><span />{state}</span>
            </div>
            <div className="media-stage">
              {showVideo && activeVideo ? (
                <video key={activeVideo.url} controls autoPlay muted playsInline>
                  <source src={activeVideo.url} type="video/mp4" />
                </video>
              ) : (
                <div className="media-empty">
                  <div className="scan-icon"><span /><span /></div>
                  <strong>
                    {state === "running"
                      ? "Demonstration in progress"
                      : status?.media.render_state === "failed"
                        ? "No completed video available"
                        : "Completed video will appear here"}
                  </strong>
                  <p>
                    {state === "running"
                      ? "Playback unlocks after execution finishes and every review video is rendered."
                      : status?.media.render_state === "failed"
                        ? "The pipeline did not pass the completion and render gate."
                        : "Start a demonstration to produce a rendered review."}
                  </p>
                </div>
              )}
              <div className="media-overlay">
                <span>{status?.run_id ?? "NO ACTIVE RUN"}</span>
                <span>{status?.generated_at ? `UPDATED ${formatTime(status.generated_at)}` : "AWAITING TELEMETRY"}</span>
              </div>
            </div>
            {showVideo && status && status.media.videos.length > 1 && (
              <div className="camera-tabs">
                {status.media.videos.map((video, index) => (
                  <button className={index === selectedVideo ? "active" : ""} onClick={() => setSelectedVideo(index)} key={video.name}>
                    {index === 0 ? "Table view" : "Side view"}
                  </button>
                ))}
              </div>
            )}
          </div>

          <aside className="control-panel panel">
            <div className="panel-heading">
              <div>
                <p className="section-kicker">DEMONSTRATION CONTROL</p>
                <h2>Configure run</h2>
              </div>
              <span className="lock-label">LOCAL CONTROL</span>
            </div>
            <form onSubmit={startRun}>
              <fieldset disabled={isBusy || !connected}>
                <legend>Task type</legend>
                <div className="task-options">
                  {tasks.map((task) => (
                    <label className={`task-option ${taskId === task.id ? "selected" : ""}`} key={task.id}>
                      <input type="radio" name="task" value={task.id} checked={taskId === task.id} onChange={() => setTaskId(task.id)} />
                      <span className="radio-dot" />
                      <span><strong>{task.label}</strong><small>{task.description}</small></span>
                      {task.experimental && <em>Experimental</em>}
                    </label>
                  ))}
                </div>
              </fieldset>
              <fieldset disabled={isBusy || !connected}>
                <legend>Fault scenario</legend>
                <select value={fault} onChange={(event) => setFault(event.target.value)}>
                  {selectedTask?.faults.map((item) => <option value={item.id} key={item.id}>{item.label}</option>)}
                </select>
                <p className="field-help">{selectedTask?.faults.find((item) => item.id === fault)?.description}</p>
              </fieldset>
              {fault !== "none" && (
                <fieldset disabled={isBusy || !connected}>
                  <legend>Inject on pick</legend>
                  <div className="pick-grid">
                    {Array.from({ length: selectedTask?.planned_items ?? 0 }, (_, index) => (
                      <button type="button" className={pickIndex === index ? "active" : ""} onClick={() => setPickIndex(index)} key={index}>{index + 1}</button>
                    ))}
                  </div>
                </fieldset>
              )}
              <button className="start-button" type="submit" disabled={isBusy || !connected || !selectedTask}>
                <span>{isBusy ? "Demonstration running" : "Start demonstration"}</span>
                <b aria-hidden="true">→</b>
              </button>
            </form>
            {requestError && <div className="request-error">{requestError}</div>}
            <p className="safety-note"><span>i</span> One run at a time. Faults are simulator-only and applied once.</p>
          </aside>
        </section>

        <section className="status-strip panel">
          <div><span>RUN STATUS</span><strong>{status?.status ?? "Connecting…"}</strong></div>
          <div><span>PIPELINE STAGE</span><strong>{status?.stage ?? "—"}</strong></div>
          <div><span>CONTROLLER</span><strong>{display(status?.controller_phase)}</strong></div>
          <div><span>CURRENT STEP</span><strong>{display(status?.current_step)}</strong></div>
          <div><span>STARTED</span><strong>{formatTime(status?.process.started_at)}</strong></div>
          <div><span>SOURCE</span><strong title={status?.source}>{display(status?.source)}</strong></div>
        </section>

        <section className="data-grid">
          <article className="panel data-panel inventory-panel">
            <div className="panel-heading compact"><div><p className="section-kicker">VERIFIED OUTPUT</p><h2>Inventory & outcomes</h2></div></div>
            {Object.keys(status?.counts ?? {}).length ? (
              <div className="table-wrap"><table>
                <thead><tr><th>Category</th>{countColumns.map((column) => <th key={column}>{column}</th>)}</tr></thead>
                <tbody>{Object.entries(status?.counts ?? {}).map(([category, row]) => (
                  <tr key={category}><td>{category}</td>{countColumns.map((column) => <td key={column}>{display(row[column])}</td>)}</tr>
                ))}</tbody>
              </table></div>
            ) : <div className="empty-data">Outcome counts will appear as the pipeline publishes run artifacts.</div>}
            <p className="table-note">Sorted counts require every reported placement check to pass.</p>
          </article>

          <article className="panel data-panel metrics-panel">
            <div className="panel-heading compact"><div><p className="section-kicker">RUN TELEMETRY</p><h2>Available metrics</h2></div></div>
            <div className="metric-list">
              {Object.entries(status?.metrics ?? {}).map(([name, value]) => <div key={name}><span>{name}</span><strong>{display(value)}</strong></div>)}
              {!Object.keys(status?.metrics ?? {}).length && <div className="empty-data">No measurements reported.</div>}
            </div>
          </article>
        </section>

        <section className="panel alerts-panel">
          <div className="panel-heading compact">
            <div><p className="section-kicker">EXCEPTIONS & EXCLUSIONS</p><h2>Object alerts</h2></div>
            <span className="alert-count">{status?.alerts.length ?? 0} alerts</span>
          </div>
          {status?.alerts.length ? (
            <div className="alert-list">{status.alerts.map((alert) => (
              <div className="alert-row" key={alert.id}>
                <span className={`alert-symbol ${alert.outcome.toLowerCase()}`}>!</span>
                <div><strong>{alert.object_id}</strong><small>{alert.category} · {alert.outcome}</small></div>
                <p>{alert.reason}</p>
              </div>
            ))}</div>
          ) : <div className="clear-state"><span>✓</span><div><strong>No active alerts</strong><p>Failures, skips, and abort reasons will appear here.</p></div></div>}
        </section>

        {(status?.log_tail || status?.media.bundle_url) && (
          <section className="run-tools">
            {status?.media.bundle_url && <a href={status.media.bundle_url}>Download run bundle ↓</a>}
            {status?.log_tail && <details><summary>Pipeline log</summary><pre>{status.log_tail}</pre></details>}
          </section>
        )}
      </main>
      <footer><span>FANUC ER-4iA DIGITAL TWIN</span><span>Isaac Sim + Cosmos · simulated output</span></footer>
    </div>
  );
}

export default App;
