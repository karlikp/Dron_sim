from __future__ import annotations

from dataclasses import dataclass
from typing import Tuple

import numpy as np


@dataclass(frozen=True)
class Roi:
    x0: int
    y0: int
    x1: int
    y1: int


@dataclass(frozen=True)
class DepthStats:
    min_depth: float
    valid_fraction: float


def declare_depth_parameters(node) -> None:
    node.declare_parameter("depth_topic", "/stereo/depth_left")
    node.declare_parameter("stop_topic", "/obstacle_stop")

    node.declare_parameter("roi_x_min", 0.35)
    node.declare_parameter("roi_x_max", 0.65)
    node.declare_parameter("roi_y_min", 0.30)
    node.declare_parameter("roi_y_max", 0.55)

    node.declare_parameter("min_depth_m", 0.2)
    node.declare_parameter("max_depth_m", 50.0)
    node.declare_parameter("threshold_m", 10.0)
    node.declare_parameter("clear_threshold_m", 18.0)
    node.declare_parameter("min_valid_fraction", 0.05)

    node.declare_parameter("log_min_depth", True)
    node.declare_parameter("log_period_s", 0.5)


def compute_roi(shape_hw: Tuple[int, int], node) -> Roi:
    h, w = shape_hw
    x_min_f = float(node.get_parameter("roi_x_min").value)
    x_max_f = float(node.get_parameter("roi_x_max").value)
    y_min_f = float(node.get_parameter("roi_y_min").value)
    y_max_f = float(node.get_parameter("roi_y_max").value)

    x0 = int(np.clip(round(x_min_f * w), 0, w - 1))
    x1 = int(np.clip(round(x_max_f * w), x0 + 1, w))
    y0 = int(np.clip(round(y_min_f * h), 0, h - 1))
    y1 = int(np.clip(round(y_max_f * h), y0 + 1, h))
    return Roi(x0=x0, y0=y0, x1=x1, y1=y1)


def compute_min_depth(depth_roi: np.ndarray, node) -> DepthStats:
    min_depth_m = float(node.get_parameter("min_depth_m").value)
    max_depth_m = float(node.get_parameter("max_depth_m").value)

    valid = np.isfinite(depth_roi) & (depth_roi >= min_depth_m) & (depth_roi <= max_depth_m)

    total = depth_roi.size
    valid_count = int(np.count_nonzero(valid))
    valid_fraction = (valid_count / total) if total > 0 else 0.0

    if valid_count == 0:
        return DepthStats(min_depth=float("nan"), valid_fraction=valid_fraction)

    return DepthStats(min_depth=float(np.min(depth_roi[valid])), valid_fraction=valid_fraction)


def update_stop_state(current_stop: bool, min_depth: float, node) -> bool:
    threshold = float(node.get_parameter("threshold_m").value)
    clear_thr = float(node.get_parameter("clear_threshold_m").value)

    if not current_stop:
        return bool(np.isfinite(min_depth) and min_depth < threshold)
    if np.isfinite(min_depth) and min_depth > clear_thr:
        return False
    return current_stop
