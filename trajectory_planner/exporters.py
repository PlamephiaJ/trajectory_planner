"""CSV and image exporters for planned trajectories."""

import csv
import os
from pathlib import Path
from typing import Optional

import cv2
import numpy as np
import yaml

from .models import MapData, Trajectory


def save_clean_map(
    map_data: MapData,
    output_yaml: Path,
    output_image: Optional[Path] = None,
) -> tuple[Path, Path]:
    """Write the filtered raster and a matching ROS map YAML.

    The source files are deliberately protected from accidental overwrite so
    callers always retain the original SLAM map.
    """
    output_yaml = Path(output_yaml).expanduser().resolve()
    output_image = (
        Path(output_image).expanduser().resolve()
        if output_image is not None else output_yaml.with_suffix('.pgm'))
    if output_yaml == map_data.yaml_path:
        raise ValueError('Clean map YAML must not overwrite the source YAML')
    if output_image == map_data.image_path:
        raise ValueError('Clean map image must not overwrite the source image')
    if output_image == output_yaml:
        raise ValueError('Clean map YAML and image paths must be different')

    with map_data.yaml_path.open('r', encoding='utf-8') as stream:
        metadata = yaml.safe_load(stream)
    if not isinstance(metadata, dict):
        raise ValueError(f'Invalid ROS map YAML: {map_data.yaml_path}')

    output_yaml.parent.mkdir(parents=True, exist_ok=True)
    output_image.parent.mkdir(parents=True, exist_ok=True)
    if not cv2.imwrite(str(output_image), map_data.image):
        raise OSError(f'Unable to write clean map image: {output_image}')
    metadata['image'] = os.path.relpath(output_image, output_yaml.parent)
    with output_yaml.open('w', encoding='utf-8') as stream:
        yaml.safe_dump(metadata, stream, sort_keys=False)
    return output_yaml, output_image


def save_compact_csv(trajectory: Trajectory, output_path: Path) -> Path:
    """Write the controller-facing ``x,y,speed`` CSV."""
    output_path = _prepare_output(output_path)
    with output_path.open('w', newline='', encoding='utf-8') as stream:
        writer = csv.writer(stream)
        writer.writerow(['x', 'y', 'speed'])
        writer.writerows(zip(trajectory.x, trajectory.y, trajectory.speed))
    return output_path


def save_detailed_csv(trajectory: Trajectory, output_path: Path) -> Path:
    """Write geometry, speed, centerline, and corridor diagnostics."""
    output_path = _prepare_output(output_path)
    distance = np.concatenate(
        ([0.0], np.cumsum(trajectory.segment_length[:-1])))
    with output_path.open('w', newline='', encoding='utf-8') as stream:
        writer = csv.writer(stream)
        writer.writerow([
            's_m', 'x_m', 'y_m', 'yaw_rad', 'curvature_1pm',
            'speed_mps', 'left_width_m', 'right_width_m',
            'center_x_m', 'center_y_m', 'lateral_offset_m',
        ])
        writer.writerows(zip(
            distance, trajectory.x, trajectory.y, trajectory.yaw,
            trajectory.curvature, trajectory.speed, trajectory.left_width,
            trajectory.right_width, trajectory.center_x, trajectory.center_y,
            trajectory.lateral_offset,
        ))
    return output_path


def save_preview(trajectory: Trajectory, output_path: Path) -> Path:
    """Render the centerline and speed-colored racing line over the map."""
    output_path = _prepare_output(output_path)
    canvas = cv2.cvtColor(trajectory.map_data.image, cv2.COLOR_GRAY2BGR)
    center = trajectory.map_data.world_to_pixel_float(
        np.column_stack((trajectory.center_x, trajectory.center_y)))
    center_pixels = np.rint(center[:, ::-1]).astype(np.int32)
    cv2.polylines(
        canvas, [center_pixels.reshape((-1, 1, 2))], True,
        (255, 180, 0), 1, cv2.LINE_AA)

    racing = trajectory.map_data.world_to_pixel_float(
        np.column_stack((trajectory.x, trajectory.y)))
    racing_pixels = np.rint(racing[:, ::-1]).astype(np.int32)
    minimum = float(np.min(trajectory.speed))
    span = max(float(np.max(trajectory.speed)) - minimum, 1.0e-6)
    for index, start in enumerate(racing_pixels):
        ratio = float((trajectory.speed[index] - minimum) / span)
        color = (0, int(255 * ratio), int(255 * (1.0 - ratio)))
        end = racing_pixels[(index + 1) % len(racing_pixels)]
        cv2.line(canvas, tuple(start), tuple(end), color, 2, cv2.LINE_AA)
    if not cv2.imwrite(str(output_path), canvas):
        raise OSError(f'Unable to write preview image: {output_path}')
    return output_path


def _prepare_output(output_path: Path) -> Path:
    output_path = Path(output_path).expanduser().resolve()
    output_path.parent.mkdir(parents=True, exist_ok=True)
    return output_path
