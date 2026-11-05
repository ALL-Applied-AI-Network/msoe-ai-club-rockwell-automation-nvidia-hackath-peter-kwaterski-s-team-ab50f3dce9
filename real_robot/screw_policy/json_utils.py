"""Minimal JSON extraction for Cosmos screw/bin replies."""

from __future__ import annotations

import json
import re


def parse_json(raw: str):
    raw = raw.strip()
    if raw.startswith("```"):
        raw = re.sub(r"^```(?:json)?\s*", "", raw)
        raw = re.sub(r"\s*```$", "", raw)
    try:
        return json.loads(raw)
    except json.JSONDecodeError:
        pass
    start = raw.find("{")
    end = raw.rfind("}")
    if start >= 0 and end > start:
        try:
            return json.loads(raw[start : end + 1])
        except json.JSONDecodeError:
            return None
    return None


def canonicalise_detection(parsed: dict) -> dict:
    if not isinstance(parsed, dict):
        return {"screws": [], "bin": None}
    screws = parsed.get("screws") or parsed.get("objects") or []
    out_screws = []
    for i, s in enumerate(screws):
        if not isinstance(s, dict):
            continue
        centre = s.get("center") or s.get("centre")
        if not centre or len(centre) < 2:
            continue
        out_screws.append(
            {
                "id": int(s.get("id", i)),
                "center": [float(centre[0]), float(centre[1])],
                "bbox": s.get("bbox"),
                "yaw_deg": float(s.get("yaw_deg", 0.0)),
                "confidence": float(s.get("confidence", 0.5)),
            }
        )
    bin_obj = parsed.get("bin") or parsed.get("bins")
    if isinstance(bin_obj, list) and bin_obj:
        bin_obj = bin_obj[0]
    bin_out = None
    if isinstance(bin_obj, dict):
        c = bin_obj.get("center") or bin_obj.get("centre")
        if c and len(c) >= 2:
            bin_out = {
                "center": [float(c[0]), float(c[1])],
                "confidence": float(bin_obj.get("confidence", 0.5)),
            }
    return {"screws": out_screws, "bin": bin_out}
