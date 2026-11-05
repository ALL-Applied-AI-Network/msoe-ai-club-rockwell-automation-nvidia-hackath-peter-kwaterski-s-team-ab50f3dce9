"""Serialize dashboard status for HTTP transport (no Tk imports)."""

from __future__ import annotations

from dataclasses import asdict
from datetime import datetime
from typing import Any, Dict, Optional

from TkinterView import Alert, DigitalTwinSource, ViewStatus


def status_to_dict(status: ViewStatus) -> Dict[str, Any]:
    payload = asdict(status)
    stamp = status.generated_at
    payload["generated_at"] = stamp.isoformat() if stamp is not None else None
    payload["alerts"] = [asdict(alert) for alert in status.alerts]
    return payload


def status_from_dict(payload: Dict[str, Any]) -> ViewStatus:
    generated = payload.get("generated_at")
    stamp: Optional[datetime] = None
    if generated:
        stamp = datetime.fromisoformat(generated)
    alerts = [
        Alert(**item) for item in payload.get("alerts", []) if isinstance(item, dict)
    ]
    counts = payload.get("counts", {})
    metrics = payload.get("metrics", {})
    return ViewStatus(
        run_id=payload.get("run_id", ""),
        status=payload.get("status", ""),
        stage=payload.get("stage", ""),
        source=payload.get("source", ""),
        simulated=bool(payload.get("simulated", True)),
        generated_at=stamp,
        counts=counts,
        metrics=metrics,
        alerts=alerts,
        demonstration=bool(payload.get("demonstration", False)),
        finished=bool(payload.get("finished", False)),
        controller_phase=payload.get("controller_phase"),
        current_step=payload.get("current_step"),
        frame_url=payload.get("frame_url"),
        frame_image=None,
        bundle_url=payload.get("bundle_url"),
    )


def read_dashboard_status(run_dir) -> ViewStatus:
    return DigitalTwinSource(run_dir).read_status()
