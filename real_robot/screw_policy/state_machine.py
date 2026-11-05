"""Closed-loop TCP state machine for screw pick-and-place."""

from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum, auto

import numpy as np

from screw_policy.config import ScrewTaskConfig
from screw_policy.localization import ScrewTarget, localize_from_obs
from screw_policy.tcp_envelope import validate_chunk


class Phase(Enum):
    LOCALIZE = auto()
    HOVER = auto()
    APPROACH = auto()
    CLOSE = auto()
    LIFT = auto()
    TO_BIN = auto()
    RELEASE = auto()
    RETREAT = auto()
    VERIFY = auto()
    HOLD = auto()


@dataclass
class ControllerState:
    phase: Phase = Phase.LOCALIZE
    targets: list[ScrewTarget] = field(default_factory=list)
    bin_xy: tuple[float, float] = (510.0, 110.0)
    active: ScrewTarget | None = None
    drops: int = 0
    retries: int = 0
    picked_ids: set[tuple[float, float]] = field(default_factory=set)
    last_event: str = "reset"


class ScrewController:
    def __init__(self, cfg: ScrewTaskConfig):
        self.cfg = cfg
        self.state = ControllerState(bin_xy=cfg.bin_xy_mm)

    def reset(self) -> None:
        self.state = ControllerState(bin_xy=self.cfg.bin_xy_mm)
        self.state.last_event = "reset"

    def _tcp(self, x, y, z, r, gripper) -> np.ndarray:
        return np.array([x, y, z, -180.0, 0.0, r, gripper], dtype=float)

    def _hold_chunk(self) -> np.ndarray:
        s = self.cfg.safe_hover
        row = self._tcp(s[0], s[1], s[2], s[5], 0.0)
        return validate_chunk(row, self.cfg)

    def _next_target(self) -> ScrewTarget | None:
        for t in sorted(self.state.targets, key=lambda s: (-s.confidence, s.x_mm)):
            key = (round(t.x_mm, 1), round(t.y_mm, 1))
            if key not in self.state.picked_ids:
                return t
        return None

    def step(
        self,
        obs: dict,
        cosmos_parsed: dict | None,
        image_wh: tuple[int, int],
    ) -> tuple[np.ndarray, dict]:
        st = self.state
        info: dict = {"phase": st.phase.name, "drops": st.drops}

        if st.drops >= self.cfg.expected_screws:
            st.phase = Phase.HOLD
            st.last_event = "complete"
            info["event"] = st.last_event
            return self._hold_chunk(), info

        if st.phase == Phase.HOLD:
            info["event"] = "hold"
            return self._hold_chunk(), info

        if st.phase in (Phase.LOCALIZE, Phase.VERIFY):
            targets, bin_xy, perception = localize_from_obs(
                obs, self.cfg, cosmos_parsed, image_wh=image_wh
            )
            st.targets = targets
            if bin_xy:
                st.bin_xy = bin_xy
            info["perception"] = perception
            if st.phase == Phase.VERIFY:
                dropped = st.active
                st.picked_ids.add(
                    (round(dropped.x_mm, 1), round(dropped.y_mm, 1))
                )
                st.drops += 1
                st.retries = 0
                st.last_event = "dropped"
                st.phase = Phase.LOCALIZE
                info["retries"] = st.retries
                info["drops"] = st.drops
                info["event"] = st.last_event
                info["target"] = {
                    "x": dropped.x_mm,
                    "y": dropped.y_mm,
                    "yaw": dropped.yaw_deg,
                    "confidence": dropped.confidence,
                    "source": dropped.source,
                }
                st.active = None
                return self._hold_chunk(), info
            st.active = self._next_target()
            if st.active is None:
                st.phase = Phase.HOLD
                st.last_event = "no_targets"
                info["event"] = st.last_event
                return self._hold_chunk(), info
            st.phase = Phase.HOVER
            st.last_event = "localized"
            info["localized"] = True
            info["event"] = st.last_event
            info["target"] = {
                "x": st.active.x_mm,
                "y": st.active.y_mm,
                "yaw": st.active.yaw_deg,
                "confidence": st.active.confidence,
                "source": st.active.source,
            }

        assert st.active is not None
        t = st.active
        r = float(t.yaw_deg)
        chunk = None
        if st.phase == Phase.HOVER:
            chunk = self._tcp(t.x_mm, t.y_mm, self.cfg.hover_z_mm, r, 0.0)
            st.phase = Phase.APPROACH
            st.last_event = "hover"
        elif st.phase == Phase.APPROACH:
            chunk = self._tcp(t.x_mm, t.y_mm, self.cfg.grasp_z_mm, r, 0.0)
            st.phase = Phase.CLOSE
            st.last_event = "approach"
        elif st.phase == Phase.CLOSE:
            chunk = self._tcp(t.x_mm, t.y_mm, self.cfg.grasp_z_mm, r, 1.0)
            st.phase = Phase.LIFT
            st.last_event = "close"
        elif st.phase == Phase.LIFT:
            chunk = self._tcp(t.x_mm, t.y_mm, self.cfg.lift_z_mm, r, 1.0)
            st.phase = Phase.TO_BIN
            st.last_event = "lift"
        elif st.phase == Phase.TO_BIN:
            bx, by = st.bin_xy
            chunk = self._tcp(bx, by, self.cfg.bin_hover_z_mm, r, 1.0)
            st.phase = Phase.RELEASE
            st.last_event = "to_bin"
        elif st.phase == Phase.RELEASE:
            bx, by = st.bin_xy
            chunk = self._tcp(bx, by, self.cfg.bin_drop_z_mm, r, 0.0)
            st.phase = Phase.RETREAT
            st.last_event = "release"
        elif st.phase == Phase.RETREAT:
            bx, by = st.bin_xy
            chunk = self._tcp(bx, by, self.cfg.bin_hover_z_mm, r, 0.0)
            st.phase = Phase.VERIFY
            st.last_event = "retreat"
        else:
            chunk = self._hold_chunk()[0]

        info["phase"] = st.phase.name
        info["event"] = st.last_event
        out = validate_chunk(chunk, self.cfg)
        return out, info
