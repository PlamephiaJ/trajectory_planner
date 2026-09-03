"""Bounded racing-line and minimum-time optimization."""

import math
from typing import Iterable, List, Tuple

import cv2
import numpy as np
from scipy.optimize import lsq_linear, minimize
from scipy.sparse import coo_matrix

from .config import PlannerConfig
from .exceptions import PlanningError
from .geometry import curve_geometry
from .models import MapData
from .velocity import profile_lap_time, velocity_profile


def _append_sparse_row(
    rows: List[int],
    cols: List[int],
    values: List[float],
    rhs: List[float],
    coefficients: Iterable[Tuple[int, float]],
    target: float,
) -> None:
    row = len(rhs)
    for col, value in coefficients:
        rows.append(row)
        cols.append(int(col))
        values.append(float(value))
    rhs.append(float(target))


def optimize_racing_line(
    centerline: np.ndarray,
    normals: np.ndarray,
    left_width: np.ndarray,
    right_width: np.ndarray,
    config: PlannerConfig,
) -> np.ndarray:
    count = len(centerline)
    mean_spacing = float(np.mean(np.linalg.norm(
        np.roll(centerline, -1, axis=0) - centerline, axis=1)))
    clearance = config.required_clearance
    upper = np.maximum(0.0, left_width - clearance) * config.corridor_fraction
    lower = -np.maximum(0.0, right_width - clearance) * config.corridor_fraction
    fixed = upper - lower < 1.0e-9
    lower[fixed] = -1.0e-9
    upper[fixed] = 1.0e-9

    rows: List[int] = []
    cols: List[int] = []
    values: List[float] = []
    rhs: List[float] = []
    curvature_scale = math.sqrt(config.curvature_weight) / max(mean_spacing ** 2, 1.0e-6)
    length_scale = math.sqrt(config.length_weight) / max(mean_spacing, 1.0e-6)
    smooth_scale = math.sqrt(config.offset_smooth_weight) / max(mean_spacing, 1.0e-6)
    center_scale = math.sqrt(config.center_weight)
    curvature_smooth_scale = (
        math.sqrt(config.curvature_smooth_weight) /
        max(mean_spacing ** 3, 1.0e-6))

    for index in range(count):
        previous = (index - 1) % count
        following = (index + 1) % count
        second_center = centerline[following] - 2.0 * centerline[index] + centerline[previous]
        for dimension in (0, 1):
            _append_sparse_row(
                rows, cols, values, rhs,
                [
                    (previous, curvature_scale * normals[previous, dimension]),
                    (index, -2.0 * curvature_scale * normals[index, dimension]),
                    (following, curvature_scale * normals[following, dimension]),
                ],
                -curvature_scale * second_center[dimension],
            )
            first_center = centerline[following, dimension] - centerline[index, dimension]
            _append_sparse_row(
                rows, cols, values, rhs,
                [
                    (index, -length_scale * normals[index, dimension]),
                    (following, length_scale * normals[following, dimension]),
                ],
                -length_scale * first_center,
            )
        _append_sparse_row(
            rows, cols, values, rhs,
            [(index, -smooth_scale), (following, smooth_scale)], 0.0)
        _append_sparse_row(
            rows, cols, values, rhs, [(index, center_scale)], 0.0)
        # Penalize changes in the second difference as well as its magnitude.
        # This suppresses isolated curvature spikes and produces a wider,
        # progressively turning racing line through corner entry and exit.
        second_following = (index + 2) % count
        third_center = (
            centerline[second_following] - 3.0 * centerline[following] +
            3.0 * centerline[index] - centerline[previous])
        for dimension in (0, 1):
            _append_sparse_row(
                rows, cols, values, rhs,
                [
                    (previous, -curvature_smooth_scale * normals[previous, dimension]),
                    (index, 3.0 * curvature_smooth_scale * normals[index, dimension]),
                    (following, -3.0 * curvature_smooth_scale * normals[following, dimension]),
                    (second_following,
                     curvature_smooth_scale * normals[second_following, dimension]),
                ],
                -curvature_smooth_scale * third_center[dimension])

    matrix = coo_matrix((values, (rows, cols)), shape=(len(rhs), count)).tocsr()
    result = lsq_linear(
        matrix,
        np.asarray(rhs),
        bounds=(lower, upper),
        method='trf',
        tol=1.0e-6,
        lsmr_tol='auto',
        max_iter=config.max_optimization_iterations,
        verbose=0,
    )
    # lsq_linear can return its current bounded, finite solution when the
    # iteration cap is reached; that solution is still safe and useful.
    if not result.success and (result.status < 0 or not np.all(np.isfinite(result.x))):
        raise PlanningError('Racing-line optimization failed: ' + result.message)
    return result.x


def enforce_clearance(
    map_data: MapData,
    track_mask: np.ndarray,
    centerline: np.ndarray,
    normals: np.ndarray,
    offsets: np.ndarray,
    required_clearance: float,
) -> Tuple[np.ndarray, np.ndarray]:
    clearance_pixels = cv2.distanceTransform(track_mask.astype(np.uint8), cv2.DIST_L2, 5)
    adjusted = offsets.copy()
    for _ in range(12):
        points = centerline + adjusted[:, None] * normals
        pixel = map_data.world_to_pixel_float(points)
        rows = np.clip(np.rint(pixel[:, 0]).astype(int), 0, track_mask.shape[0] - 1)
        cols = np.clip(np.rint(pixel[:, 1]).astype(int), 0, track_mask.shape[1] - 1)
        clearance = clearance_pixels[rows, cols] * map_data.resolution
        bad = clearance + 0.5 * map_data.resolution < required_clearance
        if not np.any(bad):
            return adjusted, clearance
        expanded = bad | np.roll(bad, 1) | np.roll(bad, -1)
        adjusted[expanded] *= 0.75
    points = centerline + adjusted[:, None] * normals
    pixel = map_data.world_to_pixel_float(points)
    rows = np.clip(np.rint(pixel[:, 0]).astype(int), 0, track_mask.shape[0] - 1)
    cols = np.clip(np.rint(pixel[:, 1]).astype(int), 0, track_mask.shape[1] - 1)
    clearance = clearance_pixels[rows, cols] * map_data.resolution
    return adjusted, clearance


def refine_minimum_time_line(
    centerline: np.ndarray,
    normals: np.ndarray,
    left_width: np.ndarray,
    right_width: np.ndarray,
    initial_offsets: np.ndarray,
    config: PlannerConfig,
) -> np.ndarray:
    """Refine a safe initial line against the full estimated lap time.

    A small periodic Fourier basis keeps the nonlinear optimization cheap and
    smooth. The full friction-circle velocity profile is recomputed for every
    candidate, so the objective rewards useful corner radius and corner-exit
    speed rather than curvature alone.
    """
    count = len(centerline)
    phase = 2.0 * math.pi * np.arange(count, dtype=float) / count
    columns = []
    harmonic = 1
    while len(columns) < config.time_optimization_modes:
        columns.append(np.sin(harmonic * phase))
        if len(columns) < config.time_optimization_modes:
            columns.append(np.cos(harmonic * phase))
        harmonic += 1
    basis = np.column_stack(columns)
    basis /= np.maximum(np.linalg.norm(basis, axis=0, keepdims=True), 1.0e-9)
    # Normalize each mode by sqrt(N), making coefficient bounds represent an
    # intuitive approximate lateral displacement in metres.
    basis *= math.sqrt(float(count))

    clearance = config.required_clearance
    upper = np.maximum(0.0, left_width - clearance) * config.corridor_fraction
    lower = -np.maximum(0.0, right_width - clearance) * config.corridor_fraction

    def candidate(coefficients: np.ndarray) -> np.ndarray:
        return np.clip(initial_offsets + basis @ coefficients, lower, upper)

    def objective(coefficients: np.ndarray) -> float:
        offsets = candidate(coefficients)
        points = centerline + offsets[:, None] * normals
        _, curvature, _, segment_length = curve_geometry(points)
        speed = velocity_profile(curvature, segment_length, config)
        lap_time = profile_lap_time(speed, segment_length)
        change = offsets - initial_offsets
        smooth_change = np.roll(change, -1) - change
        regularization = config.time_offset_regularization * float(
            np.mean(change * change) + 4.0 * np.mean(smooth_change * smooth_change))
        turning_penalty = 0.0
        if config.min_turning_radius > 0.0:
            excess = np.maximum(np.abs(curvature) - config.max_curvature, 0.0)
            # Make a kinematically infeasible line much more expensive than
            # any realistic lap-time improvement while retaining a smooth
            # objective for Powell optimization.
            turning_penalty = 1000.0 * float(np.mean(excess * excess))
        return lap_time + regularization + turning_penalty

    zero = np.zeros(config.time_optimization_modes, dtype=float)
    result = minimize(
        objective,
        zero,
        method='Powell',
        bounds=[(-config.time_optimization_step, config.time_optimization_step)] *
        config.time_optimization_modes,
        options={
            'maxiter': config.max_time_optimization_iterations,
            'xtol': 1.0e-3,
            'ftol': 1.0e-4,
            'disp': False,
        },
    )
    if not np.all(np.isfinite(result.x)) or float(result.fun) >= objective(zero):
        return initial_offsets
    return candidate(np.asarray(result.x, dtype=float))
