from pathlib import Path

import cv2
import numpy as np
import yaml

from trajectory_planner import (
    MapData, PlannerConfig, plan_trajectory,
    save_compact_csv, save_detailed_csv,
)
from trajectory_planner.geometry import minimum_dense_clearance
from trajectory_planner.map_loader import load_map
from trajectory_planner.velocity import available_longitudinal_accel


def _synthetic_track(tmp_path: Path) -> Path:
    image = np.full((220, 280), 255, dtype=np.uint8)
    cv2.rectangle(image, (20, 20), (260, 200), 0, thickness=3)
    cv2.rectangle(image, (75, 70), (205, 150), 0, thickness=3)
    image_path = tmp_path / 'track.pgm'
    assert cv2.imwrite(str(image_path), image)
    yaml_path = tmp_path / 'track.yaml'
    yaml_path.write_text(yaml.safe_dump({
        'image': image_path.name,
        'mode': 'trinary',
        'resolution': 0.05,
        'origin': [-7.0, -5.5, 0.0],
        'negate': 0,
        'occupied_thresh': 0.65,
        'free_thresh': 0.196,
    }), encoding='utf-8')
    return yaml_path


def test_closed_trajectory_and_dynamic_limits(tmp_path):
    trajectory = plan_trajectory(
        _synthetic_track(tmp_path),
        PlannerConfig(
            spacing=0.15,
            vehicle_width=0.30,
            wall_margin=0.05,
            max_speed=5.0,
            max_lateral_accel=5.0,
            max_accel=2.5,
            max_decel=5.0,
        ),
    )
    assert len(trajectory.x) > 100
    assert trajectory.length > 20.0
    assert np.all(np.isfinite(trajectory.speed))
    assert np.all(trajectory.speed <= 5.0 + 1.0e-8)
    assert np.linalg.norm(
        np.array([trajectory.x[0], trajectory.y[0]]) -
        np.array([trajectory.x[-1], trajectory.y[-1]])) < 0.5

    following_speed = np.roll(trajectory.speed, -1)
    forward_delta = following_speed ** 2 - trajectory.speed ** 2
    brake_delta = trajectory.speed ** 2 - following_speed ** 2
    assert np.all(forward_delta <= 2.0 * 2.5 * trajectory.segment_length + 1.0e-6)
    assert np.all(brake_delta <= 2.0 * 5.0 * trajectory.segment_length + 1.0e-6)

    minimum_clearance, _ = minimum_dense_clearance(
        trajectory.map_data, trajectory.track_mask,
        np.column_stack((trajectory.x, trajectory.y)))
    assert minimum_clearance + 0.5 * trajectory.map_data.resolution >= 0.20

    lateral = trajectory.speed ** 2 * np.abs(trajectory.curvature)
    assert np.all(lateral <= 0.90 * 5.0 + 1.0e-6)

    csv_path = save_detailed_csv(trajectory, tmp_path / 'trajectory.csv')
    assert csv_path.is_file()
    assert csv_path.read_text(encoding='utf-8').splitlines()[0].startswith('s_m,x_m,y_m')

    compact_csv = save_compact_csv(trajectory, tmp_path / 'xy_speed.csv')
    compact_lines = compact_csv.read_text(encoding='utf-8').splitlines()
    assert compact_lines[0] == 'x,y,speed'
    assert len(compact_lines) == len(trajectory.x) + 1


def test_explicit_map_image_overrides_yaml_image(tmp_path):
    yaml_path = _synthetic_track(tmp_path)
    metadata = yaml.safe_load(yaml_path.read_text(encoding='utf-8'))
    del metadata['image']
    yaml_path.write_text(yaml.safe_dump(metadata), encoding='utf-8')

    map_data = load_map(yaml_path, tmp_path / 'track.pgm')

    assert map_data.image_path == (tmp_path / 'track.pgm').resolve()
    assert map_data.image.shape == (220, 280)


def test_config_validation():
    invalid = [
        PlannerConfig(min_speed=-0.1),
        PlannerConfig(min_speed=9.0, max_speed=8.0),
        PlannerConfig(curvature_weight=-1.0),
        PlannerConfig(lateral_accel_safety_factor=1.1),
        PlannerConfig(seed_x=1.0),
        PlannerConfig(max_velocity_iterations=0),
        PlannerConfig(time_optimization_passes=0),
    ]
    for config in invalid:
        try:
            config.validate()
        except ValueError:
            continue
        raise AssertionError(f'Invalid configuration was accepted: {config}')


def test_map_coordinate_round_trip():
    map_data = MapData(
        yaml_path=Path('/tmp/map.yaml'), image_path=Path('/tmp/map.pgm'),
        image=np.zeros((40, 60), dtype=np.uint8),
        free=np.ones((40, 60), dtype=bool), resolution=0.05,
        origin_x=-2.0, origin_y=1.5, origin_yaw=0.37,
    )
    rows = np.array([0, 12, 39])
    cols = np.array([0, 25, 59])
    world = map_data.pixel_to_world(rows, cols)
    recovered = map_data.world_to_pixel_float(world)
    assert np.allclose(recovered, np.column_stack((rows, cols)))


def test_axle_grip_and_brake_bias_affect_friction_budget():
    config = PlannerConfig(
        max_lateral_accel=6.0, front_grip_factor=1.0,
        rear_grip_factor=0.7, drive_front_fraction=0.0,
        brake_front_fraction=0.7)
    acceleration = available_longitudinal_accel(
        speed=2.0, curvature=0.8, longitudinal_limit=3.0,
        config=config, braking=False)
    braking = available_longitudinal_accel(
        speed=2.0, curvature=0.8, longitudinal_limit=3.0,
        config=config, braking=True)
    assert 0.0 < acceleration < braking < 3.0
