"""CSV and image exporters for planned trajectories."""

import csv
from pathlib import Path

import cv2
import numpy as np

from .models import Trajectory


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
