"""ROS parameter declaration and conversion to PlannerConfig."""

import math

from .config import PlannerConfig


NODE_DEFAULTS = {
    'map_yaml': '',
    'clean_map_yaml': '',
    'clean_map_image': '',
    'output_csv': '',
    'preview_png': '',
    'detailed_csv': False,
    'frame_id': 'map',
    'map_topic': '/map',
    'path_topic': '/planned_trajectory',
    'speed_topic': '/planned_speeds',
    'trajectory_data_topic': '/planned_trajectory_data',
    'trajectory_marker_topic': '/planned_trajectory_marker',
    'centerline_marker_topic': '/planned_centerline_marker',
}

PLANNER_DEFAULTS = {
    'spacing': 0.20,
    'centerline_smoothing': 0.25,
    'vehicle_width': 0.30,
    'wall_margin': 0.05,
    'min_turning_radius': 0.0,
    'max_occupied_speckle_area': 2,
    'max_speed': 6.0,
    'min_speed': 0.5,
    'max_lateral_accel': 7.0,
    'max_accel': 3.0,
    'max_decel': 6.0,
    'lateral_accel_safety_factor': 0.90,
    'front_grip_factor': 1.00,
    'rear_grip_factor': 0.95,
    'drive_front_fraction': 0.00,
    'brake_front_fraction': 0.60,
    'curvature_weight': 1.0,
    'curvature_smooth_weight': 0.35,
    'length_weight': 0.04,
    'offset_smooth_weight': 0.15,
    'center_weight': 0.001,
    'corridor_fraction': 0.92,
    'max_optimization_iterations': 100,
    'max_velocity_iterations': 100,
    'time_optimization_modes': 8,
    'max_time_optimization_iterations': 25,
    'time_optimization_passes': 2,
    'time_optimization_step': 0.12,
    'time_offset_regularization': 0.015,
    'seed_x': float('nan'),
    'seed_y': float('nan'),
    'seed_yaw': float('nan'),
    'direction': 'auto',
    'reverse': False,
}


def declare_parameters(node) -> None:
    for name, default in {**NODE_DEFAULTS, **PLANNER_DEFAULTS}.items():
        node.declare_parameter(name, default)


def planner_config_from_node(node) -> PlannerConfig:
    values = {
        name: node.get_parameter(name).value
        for name in PLANNER_DEFAULTS
    }
    for name in ('seed_x', 'seed_y', 'seed_yaw'):
        value = float(values[name])
        values[name] = value if math.isfinite(value) else None
    return PlannerConfig(**values)
