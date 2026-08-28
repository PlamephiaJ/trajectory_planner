"""ROS map YAML and raster loading."""

import math
from pathlib import Path
from typing import Optional

import cv2
import numpy as np
import yaml

from .exceptions import PlanningError
from .models import MapData


def load_map(yaml_path: Path, image_path: Optional[Path] = None) -> MapData:
    """Load ROS map metadata, optionally overriding its image path.

    The override makes the offline API explicit and also allows a PGM and YAML
    copied from different directories to be used without editing the YAML.
    """
    yaml_path = Path(yaml_path).expanduser().resolve()
    if not yaml_path.is_file():
        raise FileNotFoundError(f'Map YAML does not exist: {yaml_path}')
    with yaml_path.open('r', encoding='utf-8') as stream:
        metadata = yaml.safe_load(stream)
    if (
        not isinstance(metadata, dict) or
        (image_path is None and 'image' not in metadata)
    ):
        raise PlanningError(f'Invalid ROS map YAML: {yaml_path}')

    if image_path is None:
        image_path = Path(str(metadata['image'])).expanduser()
        if not image_path.is_absolute():
            image_path = yaml_path.parent / image_path
        image_path = image_path.resolve()
    else:
        image_path = Path(image_path).expanduser().resolve()
    image = cv2.imread(str(image_path), cv2.IMREAD_GRAYSCALE)
    if image is None:
        raise FileNotFoundError(f'Unable to read map image: {image_path}')

    try:
        resolution = float(metadata['resolution'])
    except (KeyError, TypeError, ValueError) as error:
        raise PlanningError('Map resolution is missing or invalid') from error
    if not math.isfinite(resolution) or resolution <= 0.0:
        raise PlanningError('Map resolution must be positive and finite')
    origin = metadata.get('origin', [0.0, 0.0, 0.0])
    if not isinstance(origin, (list, tuple)) or len(origin) < 3:
        raise PlanningError('Map origin must contain [x, y, yaw]')
    if not all(math.isfinite(float(value)) for value in origin[:3]):
        raise PlanningError('Map origin must contain finite values')
    negate = bool(int(metadata.get('negate', 0)))
    free_threshold = float(metadata.get('free_thresh', 0.196))
    shade = image.astype(np.float32) / 255.0
    occupancy_probability = shade if negate else 1.0 - shade
    mode = str(metadata.get('mode', 'trinary')).lower()
    if mode not in ('trinary', 'scale', 'raw'):
        raise PlanningError(f'Unsupported ROS map mode: {mode}')
    if mode == 'raw':
        # Raw maps store occupancy values directly; 0 is free and 100 occupied.
        raw_value = image if not negate else 255 - image
        free = raw_value <= int(round(100.0 * free_threshold))
    else:
        free = occupancy_probability < free_threshold
    if mode == 'trinary':
        free &= np.abs(image.astype(np.int16) - 205) > 1

    if np.count_nonzero(free) < 100:
        raise PlanningError('Map contains too few free cells')
    return MapData(
        yaml_path=yaml_path,
        image_path=image_path,
        image=image,
        free=free,
        resolution=resolution,
        origin_x=float(origin[0]),
        origin_y=float(origin[1]),
        origin_yaw=float(origin[2]),
    )
