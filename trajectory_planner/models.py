"""ROS-independent map and trajectory data models."""

import math
from dataclasses import dataclass
from pathlib import Path

import numpy as np


@dataclass
class MapData:
    yaml_path: Path
    image_path: Path
    image: np.ndarray
    free: np.ndarray
    resolution: float
    origin_x: float
    origin_y: float
    origin_yaw: float

    def pixel_to_world(self, rows: np.ndarray, cols: np.ndarray) -> np.ndarray:
        local_x = (np.asarray(cols, dtype=float) + 0.5) * self.resolution
        local_y = (self.image.shape[0] - np.asarray(rows, dtype=float) - 0.5) * self.resolution
        c = math.cos(self.origin_yaw)
        s = math.sin(self.origin_yaw)
        world_x = self.origin_x + c * local_x - s * local_y
        world_y = self.origin_y + s * local_x + c * local_y
        return np.column_stack((world_x, world_y))

    def world_to_pixel_float(self, points: np.ndarray) -> np.ndarray:
        points = np.atleast_2d(np.asarray(points, dtype=float))
        dx = points[:, 0] - self.origin_x
        dy = points[:, 1] - self.origin_y
        c = math.cos(self.origin_yaw)
        s = math.sin(self.origin_yaw)
        local_x = c * dx + s * dy
        local_y = -s * dx + c * dy
        cols = local_x / self.resolution - 0.5
        rows = self.image.shape[0] - local_y / self.resolution - 0.5
        return np.column_stack((rows, cols))


@dataclass
class Trajectory:
    x: np.ndarray
    y: np.ndarray
    yaw: np.ndarray
    curvature: np.ndarray
    speed: np.ndarray
    left_width: np.ndarray
    right_width: np.ndarray
    center_x: np.ndarray
    center_y: np.ndarray
    lateral_offset: np.ndarray
    segment_length: np.ndarray
    map_data: MapData
    track_mask: np.ndarray

    @property
    def length(self) -> float:
        return float(np.sum(self.segment_length))

    @property
    def estimated_lap_time(self) -> float:
        next_speed = np.roll(self.speed, -1)
        mean_speed = np.maximum(0.5 * (self.speed + next_speed), 1.0e-3)
        return float(np.sum(self.segment_length / mean_speed))
