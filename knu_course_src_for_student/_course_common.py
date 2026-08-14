"""Shared, ROS-independent functions for the KNU camera-radar labs."""
from __future__ import annotations

import csv
import math
import os
from collections import defaultdict
from pathlib import Path
from typing import Iterable, Sequence

import numpy as np

RadarTarget = tuple[float, float, float]
FusionCandidate = tuple[float, float, float, float, float]


def course_data_root() -> Path:
    """Return USB/course data root, overridable with KNU_COURSE_DATA."""
    return Path(os.environ.get("KNU_COURSE_DATA", Path.home() / "260818_0820_knu_course" / "knu_course_data")).expanduser()


def radial_root() -> Path:
    return course_data_root() / "RADIal_course"


def sample_id_from_path(path: Path) -> int:
    return int(path.stem.rsplit("_", 1)[-1])


def load_labels(path: Path) -> dict[int, list[dict[str, str]]]:
    if not path.is_file():
        raise FileNotFoundError(path)
    grouped: dict[int, list[dict[str, str]]] = defaultdict(list)
    with path.open(newline="", encoding="utf-8") as stream:
        for row in csv.DictReader(stream):
            grouped[int(row["numSample"])].append(row)
    return grouped


def load_radial_pcl(path: Path) -> np.ndarray:
    """Return RADIal PCL as rows: x, y, z, relative_speed in vehicle frame.

    Official RADIal PCL rows 5/6/7 are forward/right/up and row 8 is
    Doppler velocity.  ROS vehicle convention used here is x-forward,
    y-left, z-up, therefore y_left = -right.
    """
    raw = np.asarray(np.load(path, allow_pickle=False), dtype=np.float32)
    rows = raw if raw.ndim == 2 and raw.shape[0] == 9 else raw.T
    if rows.ndim != 2 or rows.shape[0] != 9:
        raise ValueError(f"expected a 9-field RADIal PCL, got {raw.shape}: {path}")
    return np.column_stack((rows[5], -rows[6], rows[7], rows[8])).astype(np.float32)


def nearest_radar_point(
    forward_m: float,
    y_left_m: float,
    targets: Sequence[RadarTarget],
    gate_m: float = 1.0,
) -> RadarTarget | None:
    if gate_m <= 0.0:
        raise ValueError("gate_m must be positive")
    valid = [p for p in targets if len(p) == 3 and all(map(math.isfinite, p))]
    if not valid:
        return None
    nearest = min(valid, key=lambda p: math.hypot(p[0] - forward_m, p[1] - y_left_m))
    distance = math.hypot(nearest[0] - forward_m, nearest[1] - y_left_m)
    return nearest if distance <= gate_m else None


def select_mio(targets: Iterable[RadarTarget], lane_half_width_m: float = 1.8) -> RadarTarget | None:
    """Select the nearest valid in-lane object (MIO)."""
    candidates = [
        p for p in targets
        if len(p) == 3 and all(map(math.isfinite, p))
        and p[0] > 0.0 and abs(p[1]) <= lane_half_width_m
    ]
    return min(candidates, key=lambda p: p[0]) if candidates else None


def select_fused_mio(
    candidates: Sequence[FusionCandidate],
    image_width_px: int = 1920,
    lane_half_width_m: float = 1.8,
    center_fraction: float = 0.60,
) -> tuple[float, float, float, float] | None:
    roi_width = image_width_px * center_fraction
    roi_left = 0.5 * (image_width_px - roi_width)
    roi_right = roi_left + roi_width
    supported = [
        c for c in candidates
        if len(c) == 5 and all(map(math.isfinite, c))
        and roi_left <= c[0] <= roi_right
        and c[1] > 0.0 and abs(c[2]) <= lane_half_width_m and c[4] > 0.0
    ]
    if not supported:
        return None
    c = min(supported, key=lambda item: item[1])
    return c[1], c[2], c[3], c[4]


def choose_fusion_mode(
    camera_confidence: float,
    sync_delta_s: float,
    target_valid: bool,
    min_camera_confidence: float = 0.10,
    max_sync_delta_s: float = 0.06,
) -> str:
    if not target_valid:
        return "invalid_no_target"
    if not math.isfinite(camera_confidence) or camera_confidence < min_camera_confidence:
        return "radar_only_camera_unavailable"
    if not math.isfinite(sync_delta_s) or abs(sync_delta_s) > max_sync_delta_s:
        return "radar_only_unsynchronized"
    return "fused"


def image_health_confidence(gray: np.ndarray) -> float:
    """Bounded availability/quality proxy; intentionally not an object detector."""
    image = np.asarray(gray, dtype=float)
    if image.ndim != 2 or image.size == 0 or not np.isfinite(image).all():
        return 0.0
    contrast = min(1.0, float(image.std()) / 45.0)
    edge = min(1.0, float(np.abs(np.diff(image, axis=1)).mean()) / 25.0) if image.shape[1] > 1 else 0.0
    return max(0.0, min(1.0, 0.6 * contrast + 0.4 * edge))


def time_to_collision(range_m: float, relative_speed_mps: float) -> float:
    """TTC where negative relative speed means the target is approaching."""
    if range_m <= 0.0 or relative_speed_mps >= -1e-3:
        return math.inf
    return range_m / -relative_speed_mps


def shadow_longitudinal_command(range_m: float, relative_speed_mps: float) -> dict[str, float | str | bool]:
    """Deterministic teaching policy. Output is never applied to the vehicle."""
    ttc_s = time_to_collision(range_m, relative_speed_mps)
    if ttc_s < 2.0:
        mode, accel, brake = "AEB", -6.0, 1.0
    elif ttc_s < 4.0 or range_m < 15.0:
        mode, accel, brake = "ACC_BRAKE", -2.0, 0.35
    elif range_m < 30.0:
        mode, accel, brake = "ACC_HOLD", 0.0, 0.0
    else:
        mode, accel, brake = "CRUISE", 1.0, 0.0
    return {"mode": mode, "acceleration_mps2": accel, "brake_0_to_1": brake,
            "ttc_s": ttc_s, "shadow_mode": True}


def pure_pursuit_steering(
    target_x_m: float,
    target_y_left_m: float,
    wheelbase_m: float = 2.8,
    max_abs_steer_rad: float = 0.6,
) -> float:
    """Return a limited front-wheel angle for a target in vehicle frame."""
    if target_x_m <= 0.0 or wheelbase_m <= 0.0 or max_abs_steer_rad <= 0.0:
        raise ValueError("lookahead, wheelbase and steering limit must be positive")
    lookahead_sq = target_x_m * target_x_m + target_y_left_m * target_y_left_m
    curvature = 2.0 * target_y_left_m / lookahead_sq
    steering_rad = math.atan(wheelbase_m * curvature)
    return max(-max_abs_steer_rad, min(max_abs_steer_rad, steering_rad))
