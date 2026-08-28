"""Closed-curve geometry, track width, and clearance checks."""

import math
from typing import Tuple

import cv2
import numpy as np

from .models import MapData


def curve_geometry(points: np.ndarray) -> Tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    previous = np.roll(points, 1, axis=0)
    following = np.roll(points, -1, axis=0)
    before = points - previous
    after = following - points
    across = following - previous
    before_norm = np.maximum(np.linalg.norm(before, axis=1), 1.0e-9)
    after_norm = np.maximum(np.linalg.norm(after, axis=1), 1.0e-9)
    across_norm = np.maximum(np.linalg.norm(across, axis=1), 1.0e-9)
    cross = before[:, 0] * after[:, 1] - before[:, 1] * after[:, 0]
    curvature = 2.0 * cross / (before_norm * after_norm * across_norm)
    tangent = following - previous
    tangent /= np.maximum(np.linalg.norm(tangent, axis=1, keepdims=True), 1.0e-9)
    normal = np.column_stack((-tangent[:, 1], tangent[:, 0]))
    yaw = np.arctan2(tangent[:, 1], tangent[:, 0])
    segment_length = np.linalg.norm(following - points, axis=1)
    return yaw, curvature, normal, segment_length


def _point_is_in_mask(map_data: MapData, mask: np.ndarray, point: np.ndarray) -> bool:
    row_float, col_float = map_data.world_to_pixel_float(point)[0]
    row, col = int(round(row_float)), int(round(col_float))
    return (
        0 <= row < mask.shape[0] and 0 <= col < mask.shape[1] and bool(mask[row, col]))


def measure_track_widths(
    map_data: MapData,
    track_mask: np.ndarray,
    centerline: np.ndarray,
    normals: np.ndarray,
) -> Tuple[np.ndarray, np.ndarray]:
    step = max(0.5 * map_data.resolution, 0.01)
    max_distance = 20.0
    widths = []
    for direction in (1.0, -1.0):
        result = np.zeros(len(centerline), dtype=float)
        for index, (point, normal) in enumerate(zip(centerline, normals)):
            distance = 0.0
            while distance <= max_distance:
                candidate = point + direction * distance * normal
                if not _point_is_in_mask(map_data, track_mask, candidate):
                    break
                distance += step
            result[index] = max(0.0, distance - step)
        widths.append(result)
    return widths[0], widths[1]


def minimum_dense_clearance(
    map_data: MapData,
    track_mask: np.ndarray,
    points: np.ndarray,
) -> Tuple[float, int]:
    """Check every trajectory segment, not only its endpoint samples."""
    clearance_pixels = cv2.distanceTransform(track_mask.astype(np.uint8), cv2.DIST_L2, 5)
    sampling_step = max(0.25 * map_data.resolution, 0.005)
    minimum = math.inf
    minimum_segment = -1
    for index, start in enumerate(points):
        end = points[(index + 1) % len(points)]
        length = float(np.linalg.norm(end - start))
        sample_count = max(2, int(math.ceil(length / sampling_step)) + 1)
        ratio = np.linspace(0.0, 1.0, sample_count, endpoint=True)
        samples = start[None, :] + ratio[:, None] * (end - start)[None, :]
        pixel = map_data.world_to_pixel_float(samples)
        rows_float, cols_float = pixel[:, 0], pixel[:, 1]
        outside = (
            (rows_float < -0.5) | (rows_float >= track_mask.shape[0] - 0.5) |
            (cols_float < -0.5) | (cols_float >= track_mask.shape[1] - 0.5))
        rows = np.clip(np.rint(rows_float).astype(int), 0, track_mask.shape[0] - 1)
        cols = np.clip(np.rint(cols_float).astype(int), 0, track_mask.shape[1] - 1)
        values = clearance_pixels[rows, cols] * map_data.resolution
        values[outside] = 0.0
        segment_minimum = float(np.min(values))
        if segment_minimum < minimum:
            minimum = segment_minimum
            minimum_segment = index
    return minimum, minimum_segment
