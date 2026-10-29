"""A tiny fake of the Isaac Sim APIs that sort_scene.py uses. LOCAL TESTING ONLY.

It lets the real sort_scene.py, ik.py and fk.py run on a laptop:
  * joint commands are reached instantly (exact kinematics, no dynamics);
  * a grasp holds only if the fingertips are within 1 mm and 8 degrees of the
    cube (the real gripper has about 1 mm clearance per side);
  * a failed grasp pushes the cube 1.5 mm and turns it 5 degrees;
  * released cubes land on whatever is below them.
It checks plumbing and logic. It is NOT physics: real results come from Isaac.
Set FAKE_ALWAYS_MISS=SortCube4 (comma list) to make a cube impossible to grip.
"""

import math
import os
import sys
import types

import numpy as np

import scene_config as C
import fk
from cube_pose import silhouette, square_yaw
from test_support import camera_model

TABLE_MODEL = camera_model(C.TABLE_CAM)
BGR = {"red": (45, 35, 165), "blue": (165, 65, 35), "yellow": (40, 190, 220)}
TIP = (C.FLANGE_OFFSET + C.ADAPTER_T + C.ZCYL_LEN + C.ZFLANGE_T + C.BODY_LEN + C.PLATE_T
       + C.CARRIAGE_LEN - C.CARRIAGE_OVERLAP + C.FINGER_LENGTH)


def yaw_quat(deg):
    t = math.radians(deg) / 2
    return np.array([math.cos(t), 0.0, 0.0, math.sin(t)])


def quat_yaw(q):
    return math.degrees(2 * math.atan2(q[3], q[0]))


def render_table(objects, skip=None):
    """Flat cube silhouettes as the calibrated table camera sees them (BGR)."""
    img = np.full((TABLE_MODEL.H, TABLE_MODEL.W, 3), 200, np.uint8)
    for name, o in objects.items():
        if name != skip:
            m = silhouette(TABLE_MODEL, o["pos"][:2], o["yaw"], (TABLE_MODEL.H, TABLE_MODEL.W)) > 0
            img[m] = BGR[o["colour"]]
    return img


class FakeWorldState:
    def __init__(self, scene_state, ground_truth):
        colours = {g["name"]: g.get("colour") for g in ground_truth.get("objects", [])}
        self.objects = {
            name: dict(pos=np.array(s["position"], float), yaw=quat_yaw(s["orientation"]),
                       colour=colours.get(name, "red"))
            for name, s in scene_state["objects"].items() if name.startswith("SortCube")
        }
        # Caps, vials, etc.: kept where the scene put them (the fake only moves cubes).
        self.static = {name: (np.array(s["position"], float), np.array(s["orientation"], float))
                       for name, s in scene_state["objects"].items() if not name.startswith("SortCube")}
        self.q = np.zeros(8)
        self.held = None
        self.hold_offset = None
        self.closed = False
        self.always_miss = set(filter(None, os.environ.get("FAKE_ALWAYS_MISS", "").split(",")))

    def tool(self):
        rot, tip = fk.tool_pose(self.q[:6], tuple(np.array(C.TOOL_AXIS) * TIP))
        spread = rot @ np.array(C.FINGER_SPREAD_AXIS)
        return tip + [0, 0, C.TABLE_TOP_Z], math.degrees(math.atan2(-spread[0], spread[1]))

    def step(self):
        jaw = C.JAW_OPEN - abs(self.q[6])
        tip, g_yaw = self.tool()
        now_closed = jaw <= C.CUBE_JAW_HALF_TARGET + 1e-4
        if now_closed and not self.closed:
            self._close(tip, g_yaw)
        if not now_closed and self.closed and self.held:
            self._release(tip, g_yaw)
        self.closed = now_closed
        if self.held:
            self.objects[self.held]["pos"] = tip + self.hold_offset

    def _close(self, tip, g_yaw):
        if not self.objects:
            return
        name = min(self.objects, key=lambda n: np.linalg.norm(self.objects[n]["pos"][:2] - tip[:2]))
        o = self.objects[name]
        err_xy = float(np.linalg.norm(o["pos"][:2] - tip[:2]))
        if err_xy > 0.01:
            return
        err_yaw = abs(square_yaw(o["yaw"] - g_yaw))
        if err_xy <= 0.001 and err_yaw <= 8 and name not in self.always_miss:
            self.held = name
            self.hold_offset = o["pos"] - tip
            o["grip_yaw"] = square_yaw(o["yaw"] - g_yaw)
        else:  # fingers land on the cube and push it
            d = o["pos"][:2] - tip[:2]
            n = np.linalg.norm(d)
            o["pos"][:2] += 0.0015 * (d / n if n > 1e-6 else np.array([1.0, 0.0]))
            o["yaw"] += 5.0

    def _release(self, tip, g_yaw):
        o = self.objects[self.held]
        below = [p for n, p in self.objects.items()
                 if n != self.held and np.linalg.norm(p["pos"][:2] - tip[:2]) < C.CUBE_EDGE * 0.6]
        z = max([p["pos"][2] + C.CUBE_EDGE for p in below], default=C.TABLE_TOP_Z + C.SURFACE_DZ)
        o["pos"] = np.array([tip[0], tip[1], z])
        o["yaw"] = g_yaw + o.pop("grip_yaw", 0.0)
        self.held = None


def install(state):
    """Register fake isaacsim / omni / pxr / cell modules in sys.modules."""

    class SimulationApp:
        def __init__(self, *_): pass
        def close(self): pass

    class World:
        def __init__(self, **_): pass
        def reset(self): pass
        def step(self, render=True): state.step()

    class SingleArticulation:
        def __init__(self, prim_path, name):
            self.dof_names = [f"J{i}" for i in range(1, 7)] + ["jaw_l", "jaw_r"]
        def initialize(self): pass
        def get_joint_positions(self): return state.q.copy()
        def set_joint_positions(self, q): state.q = np.array(q, float)
        def apply_action(self, action): state.q = np.array(action.joint_positions, float)

    class SingleXFormPrim:
        def __init__(self, path):
            self.name = path.split("/")[-1]
        def get_world_pose(self):
            o = state.objects.get(self.name)
            if o is None:  # cap, vial or other non-cube object: stays put
                pos, quat = state.static.get(self.name, (np.zeros(3), yaw_quat(0)))
                return pos.copy(), quat.copy()
            return o["pos"].copy(), yaw_quat(o["yaw"])
        def set_world_pose(self, pos, quat):
            if self.name in state.objects:
                state.objects[self.name]["pos"] = np.array(pos, float)
                state.objects[self.name]["yaw"] = quat_yaw(quat)

    class Camera:
        def __init__(self, prim_path, resolution): self.path, self.res = prim_path, resolution
        def initialize(self): pass
        def get_rgba(self):
            if self.path == "/World/TableCam":
                bgr = render_table(state.objects, skip=state.held)
                return np.dstack([bgr[:, :, ::-1], np.full(bgr.shape[:2], 255, np.uint8)])
            return np.full((self.res[1], self.res[0], 4), 128, np.uint8)

    class _Attr:
        def Set(self, _): pass

    class RigidBodyAPI:
        def __init__(self, _): pass
        def CreateKinematicEnabledAttr(self): return _Attr()

    class _Prim:
        def IsValid(self): return True

    class _Stage:
        def GetPrimAtPath(self, _): return _Prim()

    def mod(name, **attrs):
        m = types.ModuleType(name)
        m.__dict__.update(attrs)
        sys.modules[name] = m
        return m

    mod("isaacsim", SimulationApp=SimulationApp)
    for name in ("isaacsim.core", "isaacsim.core.utils", "isaacsim.sensors"):
        mod(name)
    mod("isaacsim.core.api", World=World)
    mod("isaacsim.core.prims", SingleArticulation=SingleArticulation, SingleXFormPrim=SingleXFormPrim)
    mod("isaacsim.core.utils.types",
        ArticulationAction=lambda joint_positions: types.SimpleNamespace(joint_positions=joint_positions))
    mod("isaacsim.sensors.camera", Camera=Camera)
    usd = mod("omni.usd", get_context=lambda: types.SimpleNamespace(get_stage=lambda: _Stage()))
    mod("omni", usd=usd)
    mod("pxr", UsdPhysics=types.SimpleNamespace(RigidBodyAPI=RigidBodyAPI))
    mod("cell", build=lambda stage: None, enable_ccd=lambda w: None, tip_offset=lambda: TIP,
        TABLE_CAM_PATH="/World/TableCam", SIDE_CAM_PATH="/World/SideCam",
        WRIST_CAM_PATH="/World/WristCam",
        jaw_targets=lambda jaw: {"jaw_l": -(C.JAW_OPEN - float(jaw)), "jaw_r": C.JAW_OPEN - float(jaw)})
