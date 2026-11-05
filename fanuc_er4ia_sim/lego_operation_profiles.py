"""Reusable physical handling profiles for LEGO-scale rigid parts.

The executor selects these by part geometry and assembly operation. New tasks
should be expressible by choosing a profile and connector frame in AssemblyIR;
task IDs must not acquire control-law special cases.
"""

from __future__ import annotations

import numpy as np

import lego_demo_config as D


GRASP_PROFILES = {
    "pad_x_16_bracket_wall": {
        "name": "pad_x_16_bracket_wall",
        "spread_part": (1.0, 0.0, 0.0),
        "nominal_width_m": 2 * D.LEGO_PITCH,
        "interference_m": 0.0007,
        "pick_tip_from_part_root_z_m": 0.011,
        # 99207's real LDraw wall occupies local +Y=4.0..5.6 mm.
        "grasp_center_part_m": (0.0, 0.0048, 0.0),
        "carry_drive_mode": "payload_compensated",
        "carry_tracking_position_tolerance_m": 0.010,
        "carry_tracking_angle_tolerance_deg": 2.5,
        "preinsert_position_tolerance_m": 0.007,
        "contact_tracking_position_tolerance_m": 0.004,
        "closed_loop_approach_stages_m": (0.040, 0.028, 0.020, 0.012),
    },
    "pad_y_8_tall_clearance": {
        "name": "pad_y_8_tall_clearance",
        "spread_part": (0.0, 1.0, 0.0),
        "nominal_width_m": D.LEGO_PITCH,
        "interference_m": 0.0003,
        # Keeps Rev B's face (8 mm proximal to its tip) above the 18 mm wall.
        "pick_tip_from_part_root_z_m": 0.011,
    },
    "pad_x_16_tall_clearance": {
        "name": "pad_x_16_tall_clearance",
        "spread_part": (1.0, 0.0, 0.0),
        "nominal_width_m": 2 * D.LEGO_PITCH,
        "interference_m": 0.0010,
        "pick_tip_from_part_root_z_m": 0.011,
        "grasp_center_part_m": (0.0, -0.0024, 0.0),
        # The tall offset flange creates the largest moment on the long Rev B
        # tool during horizontal free-space carries.
        "carry_drive_mode": "payload_compensated",
        # This gate applies only at the 55 mm free-space staging pose. The
        # measured 12 mm pre-insert and connector/final gates remain strict.
        "carry_tracking_position_tolerance_m": 0.010,
        "carry_tracking_angle_tolerance_deg": 2.5,
        "preinsert_position_tolerance_m": 0.007,
        "contact_tracking_position_tolerance_m": 0.004,
        "closed_loop_approach_stages_m": (0.040, 0.028, 0.020, 0.012),
    },
    "pad_y_8_standard": {
        "name": "pad_y_8_standard",
        "spread_part": (0.0, 1.0, 0.0),
        "nominal_width_m": D.LEGO_PITCH,
        "interference_m": 0.0008,
        "pick_tip_from_part_root_z_m": 0.0015,
        # These low 8 mm-wide parts need only a modest free-space lift.  The
        # previous 55 mm default spent reach along oblique assembly normals.
        # 25 mm remains >7 part-thicknesses and twice the final 12 mm staged
        # approach while keeping top and side carries well inside the arm's
        # stable workspace.
        "carry_clearance_m": 0.025,
    },
    "pad_y_8_bracket_base": {
        "name": "pad_y_8_bracket_base",
        "spread_part": (0.0, 1.0, 0.0),
        "nominal_width_m": D.LEGO_PITCH,
        "interference_m": 0.0003,
        "pick_tip_from_part_root_z_m": 0.0015,
        "carry_drive_mode": "payload_compensated",
        "carry_tracking_position_tolerance_m": 0.010,
        "carry_tracking_angle_tolerance_deg": 2.0,
        "preinsert_position_tolerance_m": 0.007,
        "contact_tracking_position_tolerance_m": 0.004,
    },
    "pad_x_16_standard": {
        "name": "pad_x_16_standard",
        "spread_part": (1.0, 0.0, 0.0),
        "nominal_width_m": 2 * D.LEGO_PITCH - 0.0003,
        "interference_m": 0.0003,
        "pick_tip_from_part_root_z_m": 0.0015,
    },
    "pad_x_16_low": {
        "name": "pad_x_16_low",
        "spread_part": (1.0, 0.0, 0.0),
        "nominal_width_m": 2 * D.LEGO_PITCH - 0.0003,
        "interference_m": 0.0003,
        "pick_tip_from_part_root_z_m": -0.0002,
    },
    "toe_y_16_raised": {
        "name": "toe_y_16_raised",
        "spread_part": (0.0, 1.0, 0.0),
        "nominal_width_m": 2 * D.LEGO_PITCH,
        "interference_m": 0.0002,
        # Rev A's ledge is 0.8 mm proximal to the distal tip. Commanding the
        # tip 1.0 mm below the part root leaves 0.2 mm positive under-hook.
        "pick_tip_from_part_root_z_m": -0.0010,
    },
    "pad_y_16_standard": {
        "name": "pad_y_16_standard",
        "spread_part": (0.0, 1.0, 0.0),
        "nominal_width_m": 2 * D.LEGO_PITCH,
        "interference_m": 0.0003,
        "pick_tip_from_part_root_z_m": 0.0015,
    },
    "pad_y_16_low": {
        "name": "pad_y_16_low",
        "spread_part": (0.0, 1.0, 0.0),
        "nominal_width_m": 2 * D.LEGO_PITCH,
        "interference_m": 0.0003,
        "pick_tip_from_part_root_z_m": -0.0002,
    },
    "pad_y_16_press_clearance": {
        "name": "pad_y_16_press_clearance",
        "spread_part": (0.0, 1.0, 0.0),
        "nominal_width_m": 2 * D.LEGO_PITCH,
        "interference_m": 0.0010,
        "closure_gap_tolerance_m": 0.00075,
        # The measured in-jaw tip/root relation leaves the retracted shoe above
        # the 9.6 mm collision body while preserving enough wall overlap for
        # the pads to stabilize the beam during the final descent.
        "pick_tip_from_part_root_z_m": 0.003,
    },
    "pad_x_8_stud_press_clearance": {
        "name": "pad_x_8_stud_press_clearance",
        "spread_part": (1.0, 0.0, 0.0),
        "nominal_width_m": D.LEGO_PITCH,
        "interference_m": 0.0003,
        # Centering at the exposed +Y stud keeps the flat shoe wholly on the
        # half-beam side of the neighboring red plate at the 90-degree target.
        "grasp_center_part_m": (0.0, 0.004, 0.0),
        "pick_tip_from_part_root_z_m": 0.0045,
    },
    "toe_x_8_raised": {
        "name": "toe_x_8_raised",
        "spread_part": (1.0, 0.0, 0.0),
        "nominal_width_m": D.LEGO_PITCH,
        "interference_m": 0.0002,
        "pick_tip_from_part_root_z_m": -0.0010,
    },
    "pad_x_8_low": {
        "name": "pad_x_8_low",
        "spread_part": (1.0, 0.0, 0.0),
        "nominal_width_m": D.LEGO_PITCH,
        "interference_m": 0.0003,
        "pick_tip_from_part_root_z_m": -0.0002,
    },
    "pad_x_8_standard": {
        "name": "pad_x_8_standard",
        "spread_part": (1.0, 0.0, 0.0),
        "nominal_width_m": D.LEGO_PITCH,
        "interference_m": 0.0003,
        "pick_tip_from_part_root_z_m": 0.0015,
    },
}


OPERATION_PROFILES = {
    "fixture_seed": {
        "placement_bias_m": (0.0, 0.0, 0.0),
    },
    # The raised capture point and slower contact are interface-level behavior:
    # every top mate uses them, independent of design or task identity.
    "top_mate": {
        "placement_bias_m": (0.0, 0.0, 0.0015),
        "descent_samples": 30,
        "contact_samples": 30,
    },
    # Arm B presents the assembly upside-down before these operations. Part
    # local -Z then maps to world +Z, so Arm A can use a top-down guarded
    # insertion without passing through the existing assembly.
    "underside_mate": {
        "placement_bias_m": (0.0, 0.0, 0.0),
        "approach_axis_part": (0.0, 0.0, -1.0),
        "carry_clearance_m": 0.025,
        "preinsert_clearance_m": 0.008,
        "closed_loop_approach_stages_m": (0.018, 0.012, 0.008),
    },
    "side_mate": {
        "placement_bias_m": (0.0, 0.0, 0.0),
        # Part-local +Z is the outward approach/withdrawal direction. For the
        # current 90-degree side mates it maps to world -Y.
        "approach_axis_part": (0.0, 0.0, 1.0),
    },
}


def handling_profile(part: dict, task: dict) -> dict:
    """Return one merged, copy-safe profile for a part/operation pair."""
    grasp = dict(GRASP_PROFILES[part["grasp_profile"]])
    operation = dict(OPERATION_PROFILES[task["operation"]])
    # Operation geometry owns approach/staging behavior; the grasp profile
    # owns jaw/contact geometry. This avoids task-ID-specific insertion paths.
    return {**grasp, **operation}


def body_target_offset(part: dict) -> np.ndarray:
    return np.asarray(part.get("connector_to_body_offset_mm", (0.0, 0.0, 0.0)), float) / 1000


def connector_position_tolerance() -> float:
    return D.CONNECTOR_POSITION_TOLERANCE_M
