"""Validated planner configuration."""

import math
from dataclasses import dataclass
from typing import Optional


@dataclass
class PlannerConfig:
    """Physical and numerical parameters for the trajectory planner."""

    spacing: float = 0.20
    centerline_smoothing: float = 0.25
    vehicle_width: float = 0.30
    wall_margin: float = 0.05
    max_speed: float = 8.0
    min_speed: float = 0.5
    max_lateral_accel: float = 7.0
    max_accel: float = 3.0
    max_decel: float = 6.0
    lateral_accel_safety_factor: float = 0.90
    front_grip_factor: float = 1.00
    rear_grip_factor: float = 0.95
    drive_front_fraction: float = 0.00
    brake_front_fraction: float = 0.60
    curvature_weight: float = 1.0
    curvature_smooth_weight: float = 0.35
    length_weight: float = 0.04
    offset_smooth_weight: float = 0.15
    center_weight: float = 0.001
    corridor_fraction: float = 0.92
    max_optimization_iterations: int = 100
    max_velocity_iterations: int = 100
    time_optimization_modes: int = 8
    max_time_optimization_iterations: int = 25
    time_optimization_passes: int = 2
    time_optimization_step: float = 0.12
    time_offset_regularization: float = 0.015
    seed_x: Optional[float] = None
    seed_y: Optional[float] = None
    seed_yaw: Optional[float] = None
    reverse: bool = False

    @property
    def required_clearance(self) -> float:
        return 0.5 * self.vehicle_width + self.wall_margin

    def validate(self) -> None:
        positive = {
            'spacing': self.spacing,
            'vehicle_width': self.vehicle_width,
            'max_speed': self.max_speed,
            'max_lateral_accel': self.max_lateral_accel,
            'max_accel': self.max_accel,
            'max_decel': self.max_decel,
            'max_optimization_iterations': self.max_optimization_iterations,
            'max_velocity_iterations': self.max_velocity_iterations,
            'time_optimization_modes': self.time_optimization_modes,
            'max_time_optimization_iterations': self.max_time_optimization_iterations,
            'time_optimization_passes': self.time_optimization_passes,
            'time_optimization_step': self.time_optimization_step,
        }
        bad = [name for name, value in positive.items() if value <= 0.0]
        if bad:
            raise ValueError('Parameters must be positive: ' + ', '.join(bad))
        if self.wall_margin < 0.0:
            raise ValueError('wall_margin cannot be negative')
        if self.min_speed < 0.0 or self.min_speed > self.max_speed:
            raise ValueError('min_speed must be in [0, max_speed]')
        if self.centerline_smoothing < 0.0:
            raise ValueError('centerline_smoothing cannot be negative')
        weights = {
            'curvature_weight': self.curvature_weight,
            'curvature_smooth_weight': self.curvature_smooth_weight,
            'length_weight': self.length_weight,
            'offset_smooth_weight': self.offset_smooth_weight,
            'center_weight': self.center_weight,
            'time_offset_regularization': self.time_offset_regularization,
        }
        bad_weights = [name for name, value in weights.items() if value < 0.0]
        if bad_weights:
            raise ValueError('Optimization weights cannot be negative: ' + ', '.join(bad_weights))
        if not 0.0 < self.lateral_accel_safety_factor <= 1.0:
            raise ValueError('lateral_accel_safety_factor must be in (0, 1]')
        if self.front_grip_factor <= 0.0 or self.rear_grip_factor <= 0.0:
            raise ValueError('front/rear grip factors must be positive')
        for name, value in (
            ('drive_front_fraction', self.drive_front_fraction),
            ('brake_front_fraction', self.brake_front_fraction),
        ):
            if not 0.0 <= value <= 1.0:
                raise ValueError(f'{name} must be in [0, 1]')
        if not 0.0 < self.corridor_fraction <= 1.0:
            raise ValueError('corridor_fraction must be in (0, 1]')
        if self.time_optimization_modes < 2:
            raise ValueError('time_optimization_modes must be at least 2')
        seeds = (self.seed_x, self.seed_y)
        if (seeds[0] is None) != (seeds[1] is None):
            raise ValueError('seed_x and seed_y must be supplied together')
        optional_values = [value for value in (*seeds, self.seed_yaw) if value is not None]
        if not all(math.isfinite(value) for value in optional_values):
            raise ValueError('Seed values must be finite')
