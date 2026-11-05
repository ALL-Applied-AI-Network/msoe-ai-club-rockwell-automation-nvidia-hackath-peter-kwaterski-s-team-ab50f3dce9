"""Pure, deterministic helpers for contact-guided LEGO insertion.

This module deliberately has no Isaac Sim or NumPy dependency.  Scene code is
responsible for executing robot motions and orienting contact impulses onto the
active child body; these helpers only generate bounded search candidates,
score measured outcomes, and reduce contact impulses to a virtual wrench.
"""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Iterable, Mapping, Sequence


Vector3 = tuple[float, float, float]
Matrix3 = tuple[Vector3, Vector3, Vector3]


@dataclass(frozen=True)
class InsertionConfig:
    """Reusable System-brick search and acceptance defaults in SI units."""

    pitch_m: float = 0.008
    stud_diameter_m: float = 0.0048
    guide_clearance_m: float = 0.00012
    axial_step_m: float = 0.00015
    maximum_axial_travel_m: float = 0.0024
    retract_on_bind_m: float = 0.00025
    xy_step_m: float = 0.00015
    xy_radius_m: float = 0.00060
    yaw_steps_deg: tuple[float, ...] = (0.25, 0.50, 1.00)
    maximum_corrections: int = 12
    settle_steps: int = 3
    hold_steps: int = 12
    contact_force_N: float = 0.15
    hard_axial_force_N: float = 5.0
    lateral_force_N: float = 0.25
    torque_about_axis_Nm: float = 0.0006
    accepted_position_error_m: float = 0.00030
    accepted_angle_error_deg: float = 1.50
    missing_contact_penalty: float = 4.0

    def __post_init__(self) -> None:
        positive = {
            "pitch_m": self.pitch_m,
            "stud_diameter_m": self.stud_diameter_m,
            "guide_clearance_m": self.guide_clearance_m,
            "axial_step_m": self.axial_step_m,
            "maximum_axial_travel_m": self.maximum_axial_travel_m,
            "retract_on_bind_m": self.retract_on_bind_m,
            "xy_step_m": self.xy_step_m,
            "xy_radius_m": self.xy_radius_m,
            "contact_force_N": self.contact_force_N,
            "hard_axial_force_N": self.hard_axial_force_N,
            "lateral_force_N": self.lateral_force_N,
            "torque_about_axis_Nm": self.torque_about_axis_Nm,
            "accepted_position_error_m": self.accepted_position_error_m,
            "accepted_angle_error_deg": self.accepted_angle_error_deg,
        }
        invalid = [
            name
            for name, value in positive.items()
            if not math.isfinite(float(value)) or float(value) <= 0
        ]
        if invalid:
            raise ValueError("positive finite insertion values required: " + ", ".join(invalid))
        if self.stud_diameter_m >= self.pitch_m:
            raise ValueError("stud diameter must be smaller than pitch")
        if self.maximum_corrections <= 0 or self.settle_steps <= 0 or self.hold_steps <= 0:
            raise ValueError("step counts must be positive")
        if not self.yaw_steps_deg or any(
            not math.isfinite(float(value)) or float(value) <= 0
            for value in self.yaw_steps_deg
        ):
            raise ValueError("yaw steps must be non-empty positive finite values")
        if tuple(sorted(set(self.yaw_steps_deg))) != self.yaw_steps_deg:
            raise ValueError("yaw steps must be unique and increasing")


DEFAULT_INSERTION_CONFIG = InsertionConfig()


@dataclass(frozen=True)
class SearchOffset:
    """Candidate offset in the connector frame, ordered for deterministic use."""

    x_m: float
    y_m: float
    yaw_deg: float = 0.0
    source: str = "center"

    @property
    def radial_distance_m(self) -> float:
        return math.hypot(self.x_m, self.y_m)


def bounded_search_offsets(
    config: InsertionConfig = DEFAULT_INSERTION_CONFIG,
    *,
    yaw_observable: bool = True,
    maximum_candidates: int | None = None,
) -> tuple[SearchOffset, ...]:
    """Return a deterministic center-out lattice inside the declared bounds.

    XY candidates are center, cardinal directions, then diagonals at each
    radius.  Diagonal components are scaled by ``sqrt(2)`` so their Euclidean
    radius never exceeds ``xy_radius_m``.  Yaw-only candidates are emitted last
    and can be suppressed for single-site connectors whose yaw is unobservable.
    Runtime code may take only the first ``maximum_corrections`` candidates.
    """

    candidates = [SearchOffset(0.0, 0.0, 0.0, "center")]
    ring_count = int(math.floor(config.xy_radius_m / config.xy_step_m + 1.0e-12))
    for ring in range(1, ring_count + 1):
        radius = ring * config.xy_step_m
        for x_sign, y_sign, label in (
            (1, 0, "+x"),
            (-1, 0, "-x"),
            (0, 1, "+y"),
            (0, -1, "-y"),
        ):
            candidates.append(
                SearchOffset(x_sign * radius, y_sign * radius, 0.0, f"ring{ring}_{label}")
            )
        diagonal = radius / math.sqrt(2.0)
        for x_sign, y_sign, label in (
            (1, 1, "+x+y"),
            (1, -1, "+x-y"),
            (-1, 1, "-x+y"),
            (-1, -1, "-x-y"),
        ):
            candidates.append(
                SearchOffset(
                    x_sign * diagonal,
                    y_sign * diagonal,
                    0.0,
                    f"ring{ring}_{label}",
                )
            )

    if yaw_observable:
        for magnitude in config.yaw_steps_deg:
            candidates.append(SearchOffset(0.0, 0.0, magnitude, f"yaw+{magnitude:g}"))
            candidates.append(SearchOffset(0.0, 0.0, -magnitude, f"yaw-{magnitude:g}"))

    if maximum_candidates is not None:
        if maximum_candidates <= 0:
            raise ValueError("maximum_candidates must be positive")
        candidates = candidates[:maximum_candidates]
    return tuple(candidates)


@dataclass(frozen=True)
class CandidateMeasurement:
    """Measured result for one robot-commanded insertion candidate."""

    offset: SearchOffset
    position_error_m: float
    angle_error_deg: float
    physical_contact: bool
    axial_force_N: float | None = None
    sequence: int = 0


@dataclass(frozen=True)
class CandidateScore:
    total: float
    position_term: float
    angle_term: float
    contact_term: float
    hard_force_rejected: bool


def score_candidate(
    measurement: CandidateMeasurement,
    config: InsertionConfig = DEFAULT_INSERTION_CONFIG,
) -> CandidateScore:
    """Score measured errors; lower is better and hard-force violations lose."""

    position = float(measurement.position_error_m)
    angle = float(measurement.angle_error_deg)
    if not math.isfinite(position) or position < 0:
        raise ValueError("position_error_m must be finite and non-negative")
    if not math.isfinite(angle) or angle < 0:
        raise ValueError("angle_error_deg must be finite and non-negative")
    if measurement.axial_force_N is not None and not math.isfinite(
        float(measurement.axial_force_N)
    ):
        raise ValueError("axial_force_N must be finite when provided")

    hard_force = bool(
        measurement.axial_force_N is not None
        and abs(float(measurement.axial_force_N)) > config.hard_axial_force_N
    )
    position_term = (position / config.accepted_position_error_m) ** 2
    angle_term = (angle / config.accepted_angle_error_deg) ** 2
    contact_term = 0.0 if measurement.physical_contact else config.missing_contact_penalty
    total = math.inf if hard_force else position_term + angle_term + contact_term
    return CandidateScore(total, position_term, angle_term, contact_term, hard_force)


def rank_candidates(
    measurements: Iterable[CandidateMeasurement],
    config: InsertionConfig = DEFAULT_INSERTION_CONFIG,
) -> tuple[CandidateMeasurement, ...]:
    """Rank candidates reproducibly, preserving sequence order for exact ties."""

    values = tuple(measurements)
    return tuple(
        sorted(
            values,
            key=lambda measurement: (
                score_candidate(measurement, config).total,
                int(measurement.sequence),
                measurement.offset.source,
            ),
        )
    )


@dataclass(frozen=True)
class ContactWrench:
    """Average simulated wrench over a caller-declared contact time window."""

    force_world_N: Vector3
    torque_world_Nm: Vector3
    force_frame_N: Vector3
    torque_frame_Nm: Vector3
    impulse_world_Ns: Vector3
    contact_count: int
    duration_s: float


def _vector3(value: Sequence[float], name: str) -> Vector3:
    if len(value) != 3:
        raise ValueError(f"{name} must contain exactly three values")
    result = tuple(float(component) for component in value)
    if not all(math.isfinite(component) for component in result):
        raise ValueError(f"{name} must contain finite values")
    return result  # type: ignore[return-value]


def _cross(left: Vector3, right: Vector3) -> Vector3:
    return (
        left[1] * right[2] - left[2] * right[1],
        left[2] * right[0] - left[0] * right[2],
        left[0] * right[1] - left[1] * right[0],
    )


def _add(left: Vector3, right: Vector3) -> Vector3:
    return tuple(left[index] + right[index] for index in range(3))  # type: ignore[return-value]


def _world_to_frame(vector: Vector3, world_from_frame: Matrix3) -> Vector3:
    # Columns of world_from_frame are the frame axes expressed in world.
    return tuple(
        sum(world_from_frame[row][column] * vector[row] for row in range(3))
        for column in range(3)
    )  # type: ignore[return-value]


def contact_wrench(
    records: Iterable[Mapping[str, object]],
    *,
    origin_world: Sequence[float],
    duration_s: float,
    world_from_frame: Sequence[Sequence[float]] | None = None,
) -> ContactWrench:
    """Reduce oriented per-contact impulses into an average virtual wrench.

    Each record must provide ``position_world`` and either
    ``impulse_on_active_body_Ns`` or ``impulse_world``.  The preferred key is
    already oriented onto the active LEGO child by the scene callback.  When
    ``impulse_world`` is used, optional ``active_body_sign`` (``+1``/``-1``)
    is applied.  Scene code must calibrate PhysX's actor impulse convention;
    this pure helper intentionally does not guess collider ordering.
    """

    duration = float(duration_s)
    if not math.isfinite(duration) or duration <= 0:
        raise ValueError("duration_s must be positive and finite")
    origin = _vector3(origin_world, "origin_world")
    if world_from_frame is None:
        rotation: Matrix3 = (
            (1.0, 0.0, 0.0),
            (0.0, 1.0, 0.0),
            (0.0, 0.0, 1.0),
        )
    else:
        if len(world_from_frame) != 3:
            raise ValueError("world_from_frame must be 3x3")
        rotation = tuple(
            _vector3(row, "world_from_frame row") for row in world_from_frame
        )  # type: ignore[assignment]

    impulse_total: Vector3 = (0.0, 0.0, 0.0)
    angular_impulse_total: Vector3 = (0.0, 0.0, 0.0)
    count = 0
    for record in records:
        position_value = record.get("position_world")
        if position_value is None:
            raise ValueError("contact record is missing position_world")
        position = _vector3(position_value, "position_world")  # type: ignore[arg-type]
        impulse_value = record.get("impulse_on_active_body_Ns")
        sign = 1.0
        if impulse_value is None:
            impulse_value = record.get("impulse_world")
            if impulse_value is None:
                raise ValueError("contact record is missing an impulse vector")
            sign = float(record.get("active_body_sign", 1.0))
            if sign not in (-1.0, 1.0):
                raise ValueError("active_body_sign must be +1 or -1")
        raw_impulse = _vector3(impulse_value, "contact impulse")  # type: ignore[arg-type]
        impulse = tuple(sign * component for component in raw_impulse)  # type: ignore[assignment]
        lever = tuple(position[index] - origin[index] for index in range(3))
        impulse_total = _add(impulse_total, impulse)
        angular_impulse_total = _add(
            angular_impulse_total,
            _cross(lever, impulse),  # type: ignore[arg-type]
        )
        count += 1

    force_world = tuple(component / duration for component in impulse_total)
    torque_world = tuple(component / duration for component in angular_impulse_total)
    return ContactWrench(
        force_world_N=force_world,  # type: ignore[arg-type]
        torque_world_Nm=torque_world,  # type: ignore[arg-type]
        force_frame_N=_world_to_frame(force_world, rotation),  # type: ignore[arg-type]
        torque_frame_Nm=_world_to_frame(torque_world, rotation),  # type: ignore[arg-type]
        impulse_world_Ns=impulse_total,
        contact_count=count,
        duration_s=duration,
    )

