"""Simplified Isaac API and grasp physics for local wiring tests only.

This adapter lets the real sort_scene.py, IK, FK, and verification code execute
without Isaac. It is not a physics simulation and cannot validate real motion.
Set FAKE_ALWAYS_MISS to comma-separated object names to force missed grasps.
"""

import math
import os
import sys
import types

import numpy as np

import fk
import scene_config as C
from cube_pose import silhouette, square_yaw
from test_support import camera_model

TABLE_MODEL = camera_model(C.TABLE_CAM)
BGR = {"red": (45, 35, 165), "blue": (165, 65, 35), "yellow": (40, 190, 220)}
TIP = (
    C.FLANGE_OFFSET
    + C.ADAPTER_T
    + C.ZCYL_LEN
    + C.ZFLANGE_T
    + C.BODY_LEN
    + C.PLATE_T
    + C.CARRIAGE_LEN
    - C.CARRIAGE_OVERLAP
    + C.FINGER_LENGTH
)


def yaw_quat(degrees):
    half_angle = math.radians(degrees) / 2
    return np.array([math.cos(half_angle), 0.0, 0.0, math.sin(half_angle)])


def quat_yaw(quaternion):
    return math.degrees(2 * math.atan2(quaternion[3], quaternion[0]))


def render_table(objects, skip=None):
    """Render cube silhouettes in the calibrated table-camera image."""
    image = np.full((TABLE_MODEL.H, TABLE_MODEL.W, 3), 200, np.uint8)
    for name, obj in objects.items():
        if name == skip or name == "_static":
            continue
        mask = silhouette(
            TABLE_MODEL,
            obj["pos"][:2],
            obj["yaw"],
            (TABLE_MODEL.H, TABLE_MODEL.W),
        ) > 0
        image[mask] = BGR[obj["colour"]]
    return image


class FakeWorldState:
    def __init__(self, scene_state, ground_truth):
        colours = {
            entry["name"]: entry.get("colour")
            for entry in ground_truth.get("objects", [])
        }
        self.objects = {
            name: {
                "pos": np.array(saved["position"], float),
                "yaw": quat_yaw(saved["orientation"]),
                "colour": colours.get(name, "red"),
            }
            for name, saved in scene_state["objects"].items()
            if name.startswith("SortCube")
        }
        self.q = np.zeros(8)
        self.held = None
        self.hold_offset = None
        self.closed = False
        self.always_miss = set(
            filter(None, os.environ.get("FAKE_ALWAYS_MISS", "").split(","))
        )

    def tool(self):
        rotation, tip = fk.tool_pose(
            self.q[:6], tuple(np.array(C.TOOL_AXIS) * TIP)
        )
        spread = rotation @ np.array(C.FINGER_SPREAD_AXIS)
        return tip + [0, 0, C.TABLE_TOP_Z], math.degrees(
            math.atan2(-spread[0], spread[1])
        )

    def step(self):
        jaw = C.JAW_OPEN - abs(self.q[6])
        tip, grasp_yaw = self.tool()
        closed = jaw <= C.CUBE_JAW_HALF_TARGET + 1e-4
        if closed and not self.closed:
            self._close(tip, grasp_yaw)
        if not closed and self.closed and self.held:
            self._release(tip, grasp_yaw)
        self.closed = closed
        if self.held:
            self.objects[self.held]["pos"] = tip + self.hold_offset

    def _close(self, tip, grasp_yaw):
        if not self.objects:
            return
        name = min(
            self.objects,
            key=lambda key: np.linalg.norm(self.objects[key]["pos"][:2] - tip[:2]),
        )
        obj = self.objects[name]
        error_xy = float(np.linalg.norm(obj["pos"][:2] - tip[:2]))
        if error_xy > 0.01:
            return
        error_yaw = abs(square_yaw(obj["yaw"] - grasp_yaw))
        if error_xy <= 0.001 and error_yaw <= 8 and name not in self.always_miss:
            self.held = name
            self.hold_offset = obj["pos"] - tip
            obj["grip_yaw"] = square_yaw(obj["yaw"] - grasp_yaw)
            return

        direction = obj["pos"][:2] - tip[:2]
        magnitude = np.linalg.norm(direction)
        obj["pos"][:2] += 0.0015 * (
            direction / magnitude if magnitude > 1e-6 else np.array([1.0, 0.0])
        )
        obj["yaw"] += 5.0

    def _release(self, tip, grasp_yaw):
        obj = self.objects[self.held]
        below = [
            other
            for name, other in self.objects.items()
            if name != self.held
            and np.linalg.norm(other["pos"][:2] - tip[:2]) < C.CUBE_EDGE * 0.6
        ]
        height = max(
            (other["pos"][2] + C.CUBE_EDGE for other in below),
            default=C.TABLE_TOP_Z + C.SURFACE_DZ,
        )
        obj["pos"] = np.array([tip[0], tip[1], height])
        obj["yaw"] = grasp_yaw + obj.pop("grip_yaw", 0.0)
        self.held = None


def install(state):
    """Register only the Isaac modules consumed by sort_scene.py."""

    class SimulationApp:
        def __init__(self, *_args, **_kwargs):
            pass

        def close(self):
            pass

    class World:
        def __init__(self, **_kwargs):
            pass

        def reset(self):
            pass

        def step(self, render=True):
            state.step()

    class SingleArticulation:
        def __init__(self, prim_path, name):
            self.dof_names = [f"J{i}" for i in range(1, 7)] + ["jaw_l", "jaw_r"]

        def initialize(self):
            pass

        def get_joint_positions(self):
            return state.q.copy()

        def set_joint_positions(self, positions):
            state.q = np.array(positions, float)

        def apply_action(self, action):
            state.q = np.array(action.joint_positions, float)

    class SingleXFormPrim:
        def __init__(self, path):
            self.name = path.split("/")[-1]
            if self.name not in state.objects:
                state.objects.setdefault("_static", {})

        def get_world_pose(self):
            obj = state.objects.get(self.name)
            if obj is None:
                return np.zeros(3), yaw_quat(0)
            return obj["pos"].copy(), yaw_quat(obj["yaw"])

        def set_world_pose(self, position, quaternion):
            if self.name in state.objects:
                state.objects[self.name]["pos"] = np.array(position, float)
                state.objects[self.name]["yaw"] = quat_yaw(quaternion)

    class Camera:
        def __init__(self, prim_path, resolution):
            self.path = prim_path
            self.resolution = resolution

        def initialize(self):
            pass

        def get_rgba(self):
            if self.path == "/World/TableCam":
                bgr = render_table(state.objects, skip=state.held)
                return np.dstack(
                    [bgr[:, :, ::-1], np.full(bgr.shape[:2], 255, np.uint8)]
                )
            width, height = self.resolution
            return np.full((height, width, 4), 128, np.uint8)

    class Attribute:
        def Set(self, _value):
            pass

    class RigidBodyAPI:
        def __init__(self, _prim):
            pass

        def CreateKinematicEnabledAttr(self):
            return Attribute()

    class Prim:
        def IsValid(self):
            return True

    class Stage:
        def GetPrimAtPath(self, _path):
            return Prim()

    def module(name, **attributes):
        result = types.ModuleType(name)
        result.__dict__.update(attributes)
        sys.modules[name] = result
        return result

    state.objects.pop("_static", None)
    module("isaacsim", SimulationApp=SimulationApp)
    for name in ("isaacsim.core", "isaacsim.core.utils", "isaacsim.sensors"):
        module(name)
    module("isaacsim.core.api", World=World)
    module(
        "isaacsim.core.prims",
        SingleArticulation=SingleArticulation,
        SingleXFormPrim=SingleXFormPrim,
    )
    module(
        "isaacsim.core.utils.types",
        ArticulationAction=lambda joint_positions: types.SimpleNamespace(
            joint_positions=joint_positions
        ),
    )
    module("isaacsim.sensors.camera", Camera=Camera)
    usd = module(
        "omni.usd",
        get_context=lambda: types.SimpleNamespace(get_stage=lambda: Stage()),
    )
    module("omni", usd=usd)
    module("pxr", UsdPhysics=types.SimpleNamespace(RigidBodyAPI=RigidBodyAPI))
    module(
        "cell",
        build=lambda _stage: None,
        enable_ccd=lambda _world: None,
        tip_offset=lambda: TIP,
        TABLE_CAM_PATH="/World/TableCam",
        SIDE_CAM_PATH="/World/SideCam",
        WRIST_CAM_PATH="/World/WristCam",
        jaw_targets=lambda jaw: {
            "jaw_l": -(C.JAW_OPEN - float(jaw)),
            "jaw_r": C.JAW_OPEN - float(jaw),
        },
    )
