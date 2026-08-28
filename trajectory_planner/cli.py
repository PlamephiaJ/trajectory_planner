"""Small offline command-line adapter around the planner API."""

import argparse
from pathlib import Path

from .config import PlannerConfig
from .exceptions import PlanningError
from .exporters import save_compact_csv, save_detailed_csv, save_preview
from .planner import plan_trajectory


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description='Generate an x,y,speed trajectory from a ROS map.')
    parser.add_argument('map_yaml', type=Path)
    parser.add_argument('-i', '--map-image', type=Path)
    parser.add_argument('-o', '--output', type=Path)
    parser.add_argument('--preview', type=Path)
    parser.add_argument('--detailed-csv', action='store_true')
    parser.add_argument('--spacing', type=float, default=0.20)
    parser.add_argument('--centerline-smoothing', type=float, default=0.25)
    parser.add_argument('--vehicle-width', type=float, default=0.30)
    parser.add_argument('--wall-margin', type=float, default=0.05)
    parser.add_argument('--max-speed', type=float, default=6.0)
    parser.add_argument('--min-speed', type=float, default=0.5)
    parser.add_argument('--max-lateral-accel', type=float, default=7.0)
    parser.add_argument('--max-accel', type=float, default=3.0)
    parser.add_argument('--max-decel', type=float, default=6.0)
    parser.add_argument(
        '--lateral-accel-safety-factor', type=float, default=0.90)
    parser.add_argument('--curvature-weight', type=float, default=1.0)
    parser.add_argument('--length-weight', type=float, default=0.04)
    parser.add_argument('--offset-smooth-weight', type=float, default=0.15)
    parser.add_argument('--center-weight', type=float, default=0.001)
    parser.add_argument('--corridor-fraction', type=float, default=0.92)
    parser.add_argument('--max-optimization-iterations', type=int, default=100)
    parser.add_argument('--max-velocity-iterations', type=int, default=100)
    parser.add_argument('--time-optimization-passes', type=int, default=2)
    parser.add_argument('--seed-x', type=float)
    parser.add_argument('--seed-y', type=float)
    parser.add_argument('--seed-yaw', type=float)
    parser.add_argument(
        '--direction', choices=('auto', 'clockwise', 'counterclockwise'),
        default='auto')
    parser.add_argument('--reverse', action='store_true')
    return parser


def main(args=None) -> None:
    parser = build_parser()
    values = parser.parse_args(args)
    map_yaml = values.map_yaml.expanduser().resolve()
    output = values.output or map_yaml.with_name(
        f'{map_yaml.stem}_trajectory.csv')
    config = PlannerConfig(
        spacing=values.spacing,
        centerline_smoothing=values.centerline_smoothing,
        vehicle_width=values.vehicle_width,
        wall_margin=values.wall_margin,
        max_speed=values.max_speed,
        min_speed=values.min_speed,
        max_lateral_accel=values.max_lateral_accel,
        max_accel=values.max_accel,
        max_decel=values.max_decel,
        lateral_accel_safety_factor=values.lateral_accel_safety_factor,
        curvature_weight=values.curvature_weight,
        length_weight=values.length_weight,
        offset_smooth_weight=values.offset_smooth_weight,
        center_weight=values.center_weight,
        corridor_fraction=values.corridor_fraction,
        max_optimization_iterations=values.max_optimization_iterations,
        max_velocity_iterations=values.max_velocity_iterations,
        time_optimization_passes=values.time_optimization_passes,
        seed_x=values.seed_x,
        seed_y=values.seed_y,
        seed_yaw=values.seed_yaw,
        direction=values.direction,
        reverse=values.reverse,
    )
    try:
        trajectory = plan_trajectory(map_yaml, config, values.map_image)
        csv_path = (
            save_detailed_csv(trajectory, output)
            if values.detailed_csv else save_compact_csv(trajectory, output))
        preview_path = (
            save_preview(trajectory, values.preview)
            if values.preview else None)
    except (PlanningError, FileNotFoundError, OSError, ValueError) as error:
        parser.error(str(error))
    print(f'points: {len(trajectory.x)}')
    print(f'length: {trajectory.length:.2f} m')
    print(f'estimated lap time: {trajectory.estimated_lap_time:.2f} s')
    print(f'direction: {trajectory.direction}')
    print(f'CSV: {csv_path}')
    if preview_path:
        print(f'preview: {preview_path}')
