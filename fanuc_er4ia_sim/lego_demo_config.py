"""Locked configuration for the LEGO 77243 dual-arm simulation demo.

The values in this module are deliberately separate from ``scene_config`` so
the validated cube-stack path remains a regression baseline.  Coordinates are
metres in the shared robot-base convention; angles are degrees.
"""

from pathlib import Path

HERE = Path(__file__).resolve().parent

SET_NUMBER = "77243"
MANUAL_ID = "6566089"
MANUAL_URL = (
    "https://www.lego.com/cdn/product-assets/product.bi.core.pdf/6566089.pdf"
)
MANUAL_SHA256 = "95bae180d6059d98a97bd28c5a11b380409bcf971eb29c3215d88a2294c30059"
INVENTORY_URL = "https://brickset.com/export/inventory/77243-1"
SLICE_STEPS = (74, 75, 76, 77, 78, 79)
EXECUTABLE_STEPS = (74, 76, 77, 78, 79)
DETERMINISTIC_SEED = 77243
WORKER_FINGER_STYLE = "rev_c"
WORKER_FINGER_ASSET_ID = "tweezer_plunger_revC"
WORKER_FINGER_SOURCE = "assets/gripper/tweezer_plunger_revC"
WORKER_FINGER_ROOT = HERE / WORKER_FINGER_SOURCE
WORKER_FINGER_MOUNT_FACE_SPACING_M = 0.0082
WORKER_FINGER_SIM_ADAPTER_PER_SIDE_M = 0.0011
WORKER_FINGER_REAL_MOUNT_VERIFIED = False
WORKER_FINGER_CLOSED_PAD_GAP_M = 0.0070
CONSTRAINT_ASSISTED_GRASP = True
WORKER_FINGER_HASH_FILES = tuple(
    str(path.relative_to(WORKER_FINGER_ROOT)).replace("\\", "/")
    for path in sorted(WORKER_FINGER_ROOT.rglob("*"))
    if path.is_file()
)

ASSET_ROOT = HERE / "assets" / "lego" / SET_NUMBER
MANIFEST_PATH = ASSET_ROOT / "manifest.json"
SLICE_PATH = ASSET_ROOT / "slice.json"
BOOKLET_INDEX_PATH = ASSET_ROOT / "booklet_index.json"
LDRAW_ROOT = HERE / "assets" / "lego" / "ldraw"

# The two bases share the original calibrated table and face +X.  Sequential
# shared-zone ownership keeps their swept volumes disjoint during the slice.
ARM_SPECS = (
    {
        "name": "arm_a",
        "prim_path": "/World/ER4iA_A",
        "base_position": (0.0, -0.28, 0.90),
        "base_yaw_deg": 0.0,
        "role": "fetch_orient_assemble",
        "safe_home_tip": (0.24, -0.17, 0.30),
    },
    {
        "name": "arm_b",
        "prim_path": "/World/ER4iA_B",
        "base_position": (0.0, 0.34, 0.90),
        "base_yaw_deg": 0.0,
        "role": "fixture_support",
        "safe_home_tip": (0.24, 0.17, 0.30),
    },
)

SURFACE_Z = 0.90 - 0.0051
FIXTURE_DATUM = (0.30, 0.0, SURFACE_Z + 0.018)
FIXTURE_SIZE = (0.090, 0.075, 0.018)
FIXTURE_HANDLE_XY = (0.265, 0.140)
FIXTURE_HANDLE_SIZE = (0.030, 0.010, 0.030)

LEGO_PITCH = 0.008
PLATE_HEIGHT = 0.0032
BRICK_HEIGHT = 0.0096
STUD_DIAMETER = 0.0048
STUD_HEIGHT = 0.0018
ABS_DENSITY_KG_M3 = 1040.0

TRAY_ORIGIN = (0.245, -0.235, SURFACE_Z)
TRAY_COLUMNS = 5
TRAY_PITCH = (0.043, 0.040)
TRAY_POCKET = (0.032, 0.030, 0.010)
EDGE_FEEDER_RISER_HEIGHT = 0.010

# Connector lock is never a hidden correction: the executor must first pass
# these measured gates and write the activation event to the evidence stream.
CONNECTOR_POSITION_TOLERANCE_M = 0.0003
CONNECTOR_ANGLE_TOLERANCE_DEG = 1.5
PREINSERT_POSITION_TOLERANCE_M = 0.006
PREINSERT_ANGLE_TOLERANCE_DEG = 12.0
FINAL_POSITION_TOLERANCE_M = 0.0005
FINAL_ANGLE_TOLERANCE_DEG = 2.0
PRESS_TRAVEL_M = 0.0010
PRESS_DWELL_S = 0.25

SHARED_ZONE = {
    "x": (0.18, 0.43),
    "y": (-0.12, 0.12),
    "z": (SURFACE_Z, SURFACE_Z + 0.40),
}
MAX_IK_FAILURES = 0
MAX_COLLISIONS = 0
REQUIRED_CLEAN_RUNS = 2

