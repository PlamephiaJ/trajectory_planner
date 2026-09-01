"""Top-level orchestration for closed-track trajectory planning."""

from pathlib import Path
from typing import Optional

import numpy as np

from .centerline import build_centerline
from .config import PlannerConfig
from .exceptions import PlanningError
from .geometry import (
    curve_geometry,
    measure_track_widths,
    minimum_dense_clearance,
)
from .map_loader import load_map
from .models import Trajectory
from .optimization import (
    enforce_clearance,
    optimize_racing_line,
    refine_minimum_time_line,
)
from .velocity import profile_lap_time, velocity_profile


def plan_trajectory(
    map_yaml: Path,
    config: Optional[PlannerConfig] = None,
    map_image: Optional[Path] = None,
) -> Trajectory:
    """Plan a safe closed racing line and point-aligned speed profile."""
    config = config or PlannerConfig()
    config.validate()

    map_data = load_map(
        map_yaml, map_image, config.max_occupied_speckle_area)
    track_mask, centerline = build_centerline(map_data, config)
    _, _, normals, _ = curve_geometry(centerline)
    left_width, right_width = measure_track_widths(
        map_data, track_mask, centerline, normals)
    _validate_track_width(left_width, right_width, config)

    offsets = optimize_racing_line(
        centerline, normals, left_width, right_width, config)
    offsets, point_clearance = enforce_clearance(
        map_data, track_mask, centerline, normals, offsets,
        config.required_clearance)
    offsets, point_clearance = _refine_for_lap_time(
        map_data, track_mask, centerline, normals, left_width, right_width,
        offsets, point_clearance, config)

    racing_line = centerline + offsets[:, None] * normals
    _validate_dense_clearance(
        map_data, track_mask, racing_line, point_clearance, config)
    yaw, curvature, _, segment_length = curve_geometry(racing_line)
    speed = velocity_profile(curvature, segment_length, config)
    return Trajectory(
        x=racing_line[:, 0],
        y=racing_line[:, 1],
        yaw=yaw,
        curvature=curvature,
        speed=speed,
        left_width=left_width,
        right_width=right_width,
        center_x=centerline[:, 0],
        center_y=centerline[:, 1],
        lateral_offset=offsets,
        segment_length=segment_length,
        map_data=map_data,
        track_mask=track_mask,
    )


def _validate_track_width(
    left_width: np.ndarray,
    right_width: np.ndarray,
    config: PlannerConfig,
) -> None:
    fifth_percentile = float(
        np.percentile(np.minimum(left_width, right_width), 5))
    if fifth_percentile < config.required_clearance:
        raise PlanningError(
            'Track is narrower than vehicle_width/2 + wall_margin. '
            'Reduce the clearance parameters only after checking the map.')


def _refine_for_lap_time(
    map_data,
    track_mask,
    centerline,
    normals,
    left_width,
    right_width,
    offsets,
    point_clearance,
    config,
):
    baseline = centerline + offsets[:, None] * normals
    _, curvature, _, segments = curve_geometry(baseline)
    best_time = profile_lap_time(
        velocity_profile(curvature, segments, config), segments)
    for _ in range(config.time_optimization_passes):
        candidate = refine_minimum_time_line(
            centerline, normals, left_width, right_width, offsets, config)
        candidate, clearance = enforce_clearance(
            map_data, track_mask, centerline, normals, candidate,
            config.required_clearance)
        line = centerline + candidate[:, None] * normals
        _, curvature, _, segments = curve_geometry(line)
        candidate_time = profile_lap_time(
            velocity_profile(curvature, segments, config), segments)
        if candidate_time + 1.0e-4 >= best_time:
            break
        offsets, point_clearance = candidate, clearance
        best_time = candidate_time
    return offsets, point_clearance


def _validate_dense_clearance(
    map_data,
    track_mask,
    racing_line,
    point_clearance,
    config,
) -> None:
    dense_clearance, segment = minimum_dense_clearance(
        map_data, track_mask, racing_line)
    minimum = min(float(np.min(point_clearance)), dense_clearance)
    if minimum + 0.5 * map_data.resolution < config.required_clearance:
        raise PlanningError(
            f'Trajectory clearance is only {minimum:.3f} m at segment '
            f'{segment}; required {config.required_clearance:.3f} m')
