"""Friction-aware closed-loop velocity profiling."""

import math

import numpy as np

from .config import PlannerConfig


def available_longitudinal_accel(
    speed: float,
    curvature: float,
    longitudinal_limit: float,
    config: PlannerConfig,
    braking: bool = False,
) -> float:
    lateral_accel = speed * speed * abs(curvature)
    front_ratio = min(
        lateral_accel / (config.max_lateral_accel * config.front_grip_factor), 1.0)
    rear_ratio = min(
        lateral_accel / (config.max_lateral_accel * config.rear_grip_factor), 1.0)
    front_available = math.sqrt(max(0.0, 1.0 - front_ratio * front_ratio))
    rear_available = math.sqrt(max(0.0, 1.0 - rear_ratio * rear_ratio))
    front_fraction = (
        config.brake_front_fraction if braking else config.drive_front_fraction)
    combined = front_fraction * front_available + (1.0 - front_fraction) * rear_available
    return longitudinal_limit * combined


def velocity_profile(
    curvature: np.ndarray,
    segment_length: np.ndarray,
    config: PlannerConfig,
) -> np.ndarray:
    axle_lateral_limit = config.max_lateral_accel * min(
        config.front_grip_factor, config.rear_grip_factor)
    curve_limit = np.minimum(
        config.max_speed,
        np.sqrt(
            config.lateral_accel_safety_factor * axle_lateral_limit /
            np.maximum(np.abs(curvature), 1.0e-5)),
    )
    speed = curve_limit.copy()
    count = len(speed)
    for _ in range(config.max_velocity_iterations):
        previous_speed = speed.copy()
        for index in range(count):
            following = (index + 1) % count
            available = available_longitudinal_accel(
                float(speed[index]), float(curvature[index]), config.max_accel, config)
            reachable = math.sqrt(max(
                speed[index] ** 2 + 2.0 * available * segment_length[index], 0.0))
            speed[following] = min(speed[following], reachable)
        for index in range(count - 1, -1, -1):
            following = (index + 1) % count
            available = available_longitudinal_accel(
                float(speed[following]), float(curvature[following]),
                config.max_decel, config, braking=True)
            brake_limited = math.sqrt(max(
                speed[following] ** 2 + 2.0 * available * segment_length[index], 0.0))
            speed[index] = min(speed[index], brake_limited)
        if float(np.max(np.abs(speed - previous_speed))) < 1.0e-6:
            break
    # min_speed is a request, never an override of curvature or friction limits.
    floor = min(config.min_speed, float(np.min(speed)))
    return np.minimum(curve_limit, np.maximum(speed, floor))


def profile_lap_time(speed: np.ndarray, segment_length: np.ndarray) -> float:
    mean_speed = np.maximum(0.5 * (speed + np.roll(speed, -1)), 1.0e-3)
    return float(np.sum(segment_length / mean_speed))
