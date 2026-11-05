"""Strict, source-indexed AssemblyIR loader for the LEGO 77243 demo slice."""

from __future__ import annotations

import hashlib
import json
import math
from pathlib import Path

import lego_demo_config as D
from lego_operation_profiles import GRASP_PROFILES, OPERATION_PROFILES


class LegoPlanError(ValueError):
    pass


def _read_json(path: Path) -> dict:
    value = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(value, dict):
        raise LegoPlanError(f"{path} must contain one JSON object")
    return value


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def load_assets() -> dict:
    manifest = _read_json(D.MANIFEST_PATH)
    if manifest.get("set_number") != D.SET_NUMBER:
        raise LegoPlanError("asset manifest set number does not match the locked demo")
    parts = manifest.get("parts")
    if not isinstance(parts, dict) or not parts:
        raise LegoPlanError("asset manifest has no parts")
    for design_id, part in parts.items():
        if not isinstance(part, dict) or not part.get("name"):
            raise LegoPlanError(f"invalid asset record for {design_id}")
        if "ldraw" in part and not (D.LDRAW_ROOT / part["ldraw"]).is_file():
            raise LegoPlanError(f"missing LDraw asset for {design_id}: {part['ldraw']}")
        alias = part.get("ldraw_design_id")
        if alias is not None and Path(part["ldraw"]).stem != str(alias):
            raise LegoPlanError(f"part {design_id} has an inconsistent LDraw alias")
        if alias is not None:
            package = D.ASSET_ROOT / part.get("ldraw_package", "")
            if not package.is_file() or sha256_file(package) != part.get(
                "ldraw_package_sha256"
            ):
                raise LegoPlanError(f"part {design_id} has an unpinned LDraw package")
            dat_path = D.LDRAW_ROOT / part["ldraw"]
            if sha256_file(dat_path) != part.get("ldraw_dat_sha256"):
                raise LegoPlanError(f"part {design_id} LDraw geometry hash differs")
        if "ldraw" not in part and part.get("geometry") != "procedural_nominal":
            raise LegoPlanError(f"part {design_id} has no grounded geometry")
        if not (0 < float(part.get("mass_kg", 0)) < 0.1):
            raise LegoPlanError(f"part {design_id} has invalid mass")
    return manifest


def validate_booklet_index(index: dict) -> None:
    steps = index.get("steps")
    if index.get("manual_sha256") != D.MANUAL_SHA256:
        raise LegoPlanError("booklet index is not pinned to the locked official manual")
    if not isinstance(steps, list) or len(steps) != 106:
        raise LegoPlanError("booklet index must contain exactly 106 main steps")
    if [row.get("step") for row in steps] != list(range(1, 107)):
        raise LegoPlanError("booklet step numbers must be complete and ordered")
    for row in steps:
        if not isinstance(row.get("pdf_page"), int) or not 1 <= row["pdf_page"] <= 112:
            raise LegoPlanError(f"invalid source page for step {row.get('step')}")
        if not row.get("action_class") or not row.get("geometry_status"):
            raise LegoPlanError(f"step {row.get('step')} is missing status")


def load_plan() -> dict:
    manifest = load_assets()
    plan = _read_json(D.SLICE_PATH)
    index = _read_json(D.BOOKLET_INDEX_PATH)
    validate_booklet_index(index)
    if plan.get("set_number") != D.SET_NUMBER:
        raise LegoPlanError("slice set number does not match the locked demo")
    if tuple(plan.get("source_steps", ())) != D.SLICE_STEPS:
        raise LegoPlanError("slice source steps differ from the locked steps 74-79")

    instances = plan.get("parts")
    tasks = plan.get("tasks")
    if not isinstance(instances, list) or len(instances) != 10:
        raise LegoPlanError("the locked slice must contain ten rigid part instances")
    if not isinstance(tasks, list) or len(tasks) != len(instances):
        raise LegoPlanError("every rigid part must have one task")
    part_ids = [part.get("id") for part in instances]
    if len(set(part_ids)) != len(part_ids) or any(not x for x in part_ids):
        raise LegoPlanError("part instance IDs must be unique non-empty strings")
    by_part = {part["id"]: part for part in instances}
    for part in instances:
        if part.get("design_id") not in manifest["parts"]:
            raise LegoPlanError(f"unresolved design ID for {part.get('id')}")
        if part.get("grasp_profile") not in GRASP_PROFILES:
            raise LegoPlanError(f"part {part.get('id')} has an unknown grasp profile")
        feeder = part.get("feeder")
        if not isinstance(feeder, dict):
            raise LegoPlanError(f"part {part.get('id')} has no feeder frame")
        feeder_values = [*feeder.get("rpy_deg", []), feeder.get("clearance_mm")]
        if len(feeder_values) != 4 or not all(
            value is not None and math.isfinite(float(value)) for value in feeder_values
        ):
            raise LegoPlanError(f"part {part.get('id')} has an invalid feeder frame")
        if feeder.get("support") not in {
            "standard",
            "raised_spread_x",
            "raised_spread_y",
            "inverted_spread_x",
            "inverted_spread_y",
        }:
            raise LegoPlanError(f"part {part.get('id')} has an invalid feeder support")
        if feeder.get("support", "").startswith("inverted_spread_") and (
            "rail_height_mm" not in feeder
            or not math.isfinite(float(feeder["rail_height_mm"]))
            or float(feeder["rail_height_mm"]) <= 0
        ):
            raise LegoPlanError(
                f"part {part.get('id')} has an invalid inverted feeder rail height"
            )
        if "rail_width_mm" in feeder and (
            not math.isfinite(float(feeder["rail_width_mm"]))
            or float(feeder["rail_width_mm"]) <= 0
        ):
            raise LegoPlanError(f"part {part.get('id')} has an invalid feeder rail width")
        if "pick_tip_from_part_root_z_mm" in feeder and not math.isfinite(
            float(feeder["pick_tip_from_part_root_z_mm"])
        ):
            raise LegoPlanError(
                f"part {part.get('id')} has an invalid feeder grasp height"
            )
        connector_offset = part.get("connector_to_body_offset_mm", (0, 0, 0))
        if len(connector_offset) != 3 or not all(math.isfinite(float(x)) for x in connector_offset):
            raise LegoPlanError(f"part {part.get('id')} has an invalid connector frame")
        if "press_point_part_mm" in part:
            press_point = part["press_point_part_mm"]
            if len(press_point) != 3 or not all(
                math.isfinite(float(x)) for x in press_point
            ):
                raise LegoPlanError(f"part {part.get('id')} has an invalid press point")
            press_normal = part.get("press_normal_part", [])
            length = math.sqrt(sum(float(x) ** 2 for x in press_normal)) if len(press_normal) == 3 else 0.0
            if not math.isfinite(length) or abs(length - 1.0) > 1e-6:
                raise LegoPlanError(f"part {part.get('id')} has an invalid press normal")

    seen = set()
    placed_parts = set()
    task_ids = {task.get("id") for task in tasks}
    if len(task_ids) != len(tasks) or None in task_ids:
        raise LegoPlanError("task IDs must be unique")
    for task in tasks:
        if task.get("part") not in by_part:
            raise LegoPlanError(f"task {task.get('id')} references an unknown part")
        if task.get("arm") not in {a["name"] for a in D.ARM_SPECS}:
            raise LegoPlanError(f"task {task.get('id')} references an unknown arm")
        if task.get("operation") not in OPERATION_PROFILES:
            raise LegoPlanError(f"task {task.get('id')} has an unknown operation")
        actions = task.get("before_actions", [])
        allowed_before_actions = {
            "present_underside",
            "restore_upright_hold",
            "restore_upright_and_dock",
            "retract_fixture_base",
        }
        if not isinstance(actions, list) or any(
            action not in allowed_before_actions for action in actions
        ):
            raise LegoPlanError(f"task {task.get('id')} has invalid before-actions")
        parent = task.get("connector_parent")
        if parent is not None and parent not in by_part:
            raise LegoPlanError(f"task {task.get('id')} has an unknown connector parent")
        filters = task.get("collision_filter_parts")
        if not isinstance(filters, list) or any(item not in by_part for item in filters):
            raise LegoPlanError(f"task {task.get('id')} has invalid collision filters")
        if filters:
            raise LegoPlanError(
                f"task {task.get('id')} disables rigid-part collisions; "
                "model the stud/socket clearance instead"
            )
        mates = task.get("mates")
        if not isinstance(mates, list) or any(item not in by_part for item in mates):
            raise LegoPlanError(f"task {task.get('id')} has invalid mate graph")
        if len(set(mates)) != len(mates) or task.get("part") in mates:
            raise LegoPlanError(f"task {task.get('id')} has duplicate/self mates")
        if any(mate not in placed_parts for mate in mates):
            raise LegoPlanError(
                f"task {task.get('id')} mates a part that has not been placed"
            )
        if parent is not None and parent not in mates:
            raise LegoPlanError(
                f"task {task.get('id')} fixed-joint owner is missing from its mates"
            )
        allowed_supports = task.get("allowed_supports", [])
        if not isinstance(allowed_supports, list) or any(
            support != "fixture" for support in allowed_supports
        ):
            raise LegoPlanError(f"task {task.get('id')} has invalid allowed supports")
        allowed_contacts = task.get("allowed_contacts", [])
        if (
            not isinstance(allowed_contacts, list)
            or any(item not in placed_parts for item in allowed_contacts)
            or set(allowed_contacts) & set(mates)
        ):
            raise LegoPlanError(f"task {task.get('id')} has invalid allowed contacts")
        seating = task.get("seating")
        if seating is not None:
            if seating.get("mode") != "plunger" or seating.get("shoe") != "flat":
                raise LegoPlanError(f"task {task.get('id')} has invalid seating mode")
            if "press_point_part_mm" not in by_part[task["part"]]:
                raise LegoPlanError(f"task {task.get('id')} has no press interface")
            numeric = [
                *seating.get("seat_patch_mm", []),
                *seating.get("guide_bounds_part_mm", []),
                seating.get("guide_height_mm"),
                seating.get("guide_wall_mm"),
                seating.get("jaw_clearance_mm"),
                seating.get("press_travel_mm"),
                seating.get("alignment_tolerance_deg"),
                seating.get("position_tolerance_mm"),
                seating.get("angle_tolerance_deg"),
                seating.get("hold_steps"),
            ]
            bounds = seating.get("guide_bounds_part_mm", [])
            positive = [
                *seating.get("seat_patch_mm", []),
                seating.get("guide_height_mm"),
                seating.get("guide_wall_mm"),
                seating.get("jaw_clearance_mm"),
                seating.get("press_travel_mm"),
                seating.get("alignment_tolerance_deg"),
                seating.get("position_tolerance_mm"),
                seating.get("angle_tolerance_deg"),
                seating.get("hold_steps"),
            ]
            if (
                len(numeric) != 14
                or len(bounds) != 4
                or not all(value is not None and math.isfinite(float(value)) for value in numeric)
                or not all(float(value) > 0 for value in positive)
                or not float(bounds[0]) < float(bounds[1])
                or not float(bounds[2]) < float(bounds[3])
            ):
                raise LegoPlanError(f"task {task.get('id')} has invalid seating parameters")
        prereqs = task.get("prerequisites")
        if not isinstance(prereqs, list) or any(p not in seen for p in prereqs):
            raise LegoPlanError(f"task {task.get('id')} is not topologically ordered")
        target = task.get("target", {})
        values = [*target.get("xyz_mm", []), *target.get("rpy_deg", [])]
        if len(values) != 6 or not all(math.isfinite(float(x)) for x in values):
            raise LegoPlanError(f"task {task.get('id')} has an invalid target transform")
        seen.add(task["id"])
        placed_parts.add(task["part"])

    plan["asset_manifest"] = manifest
    plan["booklet_index"] = index
    return plan


def run_manifest(plan: dict, *, git_hash: str, seed: int = D.DETERMINISTIC_SEED) -> dict:
    return {
        "schema_version": "lego-run-manifest/1.0",
        "set_number": D.SET_NUMBER,
        "slice": plan["name"],
        "manual_id": D.MANUAL_ID,
        "manual_sha256": D.MANUAL_SHA256,
        "asset_manifest_sha256": sha256_file(D.MANIFEST_PATH),
        "slice_sha256": sha256_file(D.SLICE_PATH),
        "booklet_index_sha256": sha256_file(D.BOOKLET_INDEX_PATH),
        "git_hash": git_hash,
        "seed": int(seed),
        "arm_count": len(D.ARM_SPECS),
        "connector_lock_gate": {
            "position_tolerance_m": D.CONNECTOR_POSITION_TOLERANCE_M,
            "angle_tolerance_deg": D.CONNECTOR_ANGLE_TOLERANCE_DEG,
        },
        "worker_fingertip": {
            "asset_id": D.WORKER_FINGER_ASSET_ID,
            "status": "simulation_concept_unverified_physical_mount",
            "source": D.WORKER_FINGER_SOURCE,
            "mount_face_spacing_m": D.WORKER_FINGER_MOUNT_FACE_SPACING_M,
            "real_mount_verified": D.WORKER_FINGER_REAL_MOUNT_VERIFIED,
            "files_sha256": {
                name: sha256_file(D.WORKER_FINGER_ROOT / name)
                for name in D.WORKER_FINGER_HASH_FILES
            },
        },
    }


if __name__ == "__main__":
    loaded = load_plan()
    print(
        json.dumps(
            {
                "set": loaded["set_number"],
                "parts": len(loaded["parts"]),
                "tasks": len(loaded["tasks"]),
                "indexed_steps": len(loaded["booklet_index"]["steps"]),
            },
            indent=2,
        )
    )

