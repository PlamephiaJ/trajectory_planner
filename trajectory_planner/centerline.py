"""Closed-track selection and centerline extraction."""

import math
from collections import deque
from typing import Dict, List, Optional, Sequence, Tuple

import cv2
import numpy as np
from scipy.interpolate import CubicSpline
from scipy.ndimage import gaussian_filter1d

from .config import PlannerConfig
from .exceptions import PlanningError
from .models import MapData


def _select_track_component(
    map_data: MapData,
    seed_x: Optional[float],
    seed_y: Optional[float],
) -> np.ndarray:
    count, labels, stats, _ = cv2.connectedComponentsWithStats(
        map_data.free.astype(np.uint8), connectivity=8)
    if count <= 1:
        raise PlanningError('No connected free-space component found')

    if seed_x is not None and seed_y is not None:
        rc = map_data.world_to_pixel_float(np.array([[seed_x, seed_y]]))[0]
        row, col = np.rint(rc).astype(int)
        if not (0 <= row < labels.shape[0] and 0 <= col < labels.shape[1]):
            raise PlanningError('Seed point is outside the map')
        label = int(labels[row, col])
        if label == 0:
            nearest_distance, nearest_indices = _nearest_free_indices(map_data.free)
            if nearest_distance[row, col] * map_data.resolution > 2.0:
                raise PlanningError('Seed point is more than 2 m from free track space')
            row = int(nearest_indices[0, row, col])
            col = int(nearest_indices[1, row, col])
            label = int(labels[row, col])
        if label == 0:
            raise PlanningError('Unable to associate seed point with a track')
        return labels == label

    height, width = labels.shape
    image_area = height * width
    image_diagonal = math.hypot(height, width)
    minimum_area = max(500, int(0.003 * image_area))
    candidates: List[Tuple[float, int]] = []
    for label in range(1, count):
        x, y, component_width, component_height, area = stats[label]
        touches_border = (
            x == 0 or y == 0 or x + component_width == width or
            y + component_height == height)
        bbox_diagonal = math.hypot(component_width, component_height)
        if touches_border or area < minimum_area or bbox_diagonal < 0.20 * image_diagonal:
            continue
        component = (labels == label).astype(np.uint8)
        clearance = cv2.distanceTransform(component, cv2.DIST_L2, 5)
        median_clearance = float(np.median(clearance[component > 0]))
        # A race track is a long, relatively thin enclosed component. This
        # normalized clearance rejects large infields on maps with two walls.
        score = median_clearance / math.sqrt(float(area))
        candidates.append((score, label))
    if not candidates:
        raise PlanningError(
            'No closed track component found. Supply seed_x/seed_y inside the track.')
    return labels == min(candidates)[1]


def _nearest_free_indices(free: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
    from scipy.ndimage import distance_transform_edt

    return distance_transform_edt(~free, return_indices=True)


def _zhang_suen_thinning(mask: np.ndarray) -> np.ndarray:
    image = np.pad(mask.astype(bool), 1, mode='constant')
    for _ in range(1000):
        changed = False
        for second_step in (False, True):
            p1 = image[1:-1, 1:-1]
            p2 = image[:-2, 1:-1]
            p3 = image[:-2, 2:]
            p4 = image[1:-1, 2:]
            p5 = image[2:, 2:]
            p6 = image[2:, 1:-1]
            p7 = image[2:, :-2]
            p8 = image[1:-1, :-2]
            p9 = image[:-2, :-2]
            neighbors = (
                p2.astype(np.uint8) + p3 + p4 + p5 + p6 + p7 + p8 + p9)
            transitions = (
                ((~p2) & p3).astype(np.uint8) + ((~p3) & p4) +
                ((~p4) & p5) + ((~p5) & p6) + ((~p6) & p7) +
                ((~p7) & p8) + ((~p8) & p9) + ((~p9) & p2))
            common = p1 & (neighbors >= 2) & (neighbors <= 6) & (transitions == 1)
            if second_step:
                remove = common & ~(p2 & p4 & p8) & ~(p2 & p6 & p8)
            else:
                remove = common & ~(p2 & p4 & p6) & ~(p4 & p6 & p8)
            if np.any(remove):
                p1[remove] = False
                changed = True
        if not changed:
            return image[1:-1, 1:-1]
    raise PlanningError('Track skeletonization did not converge')


def _skeletonize_component(track_mask: np.ndarray) -> np.ndarray:
    rows, cols = np.nonzero(track_mask)
    if len(rows) == 0:
        raise PlanningError('Selected track component is empty')
    padding = 2
    r0 = max(int(rows.min()) - padding, 0)
    r1 = min(int(rows.max()) + padding + 1, track_mask.shape[0])
    c0 = max(int(cols.min()) - padding, 0)
    c1 = min(int(cols.max()) + padding + 1, track_mask.shape[1])
    cropped = _zhang_suen_thinning(track_mask[r0:r1, c0:c1])
    skeleton = np.zeros_like(track_mask, dtype=bool)
    skeleton[r0:r1, c0:c1] = cropped
    return skeleton


def _pixel_graph(skeleton: np.ndarray) -> Tuple[np.ndarray, List[List[int]]]:
    coordinates = np.column_stack(np.nonzero(skeleton)).astype(np.int32)
    if len(coordinates) < 20:
        raise PlanningError('Track skeleton is too short')
    lookup: Dict[Tuple[int, int], int] = {
        (int(row), int(col)): index
        for index, (row, col) in enumerate(coordinates)
    }
    adjacency: List[List[int]] = [[] for _ in range(len(coordinates))]
    for index, (row_value, col_value) in enumerate(coordinates):
        row, col = int(row_value), int(col_value)
        for dr in (-1, 0, 1):
            for dc in (-1, 0, 1):
                if dr == 0 and dc == 0:
                    continue
                neighbor = lookup.get((row + dr, col + dc))
                if neighbor is None:
                    continue
                # Suppress diagonal shortcuts around an existing orthogonal
                # connection; they create tiny artificial graph cycles.
                if dr != 0 and dc != 0:
                    if ((row + dr, col) in lookup or (row, col + dc) in lookup):
                        continue
                adjacency[index].append(neighbor)
    return coordinates, adjacency


def _prune_leaves(adjacency: Sequence[Sequence[int]]) -> np.ndarray:
    active = np.ones(len(adjacency), dtype=bool)
    degree = np.array([len(neighbors) for neighbors in adjacency], dtype=np.int32)
    queue = deque(int(index) for index in np.flatnonzero(degree < 2))
    while queue:
        node = queue.popleft()
        if not active[node]:
            continue
        active[node] = False
        for neighbor in adjacency[node]:
            if active[neighbor]:
                degree[neighbor] -= 1
                if degree[neighbor] == 1:
                    queue.append(neighbor)
    return active


def _tree_path(parent: np.ndarray, first: int, second: int) -> List[int]:
    first_chain: List[int] = []
    first_positions: Dict[int, int] = {}
    node = first
    while node >= 0:
        first_positions[node] = len(first_chain)
        first_chain.append(node)
        node = int(parent[node])
    second_chain: List[int] = []
    node = second
    while node not in first_positions:
        if node < 0:
            return []
        second_chain.append(node)
        node = int(parent[node])
    lca = node
    return first_chain[:first_positions[lca] + 1] + list(reversed(second_chain))


def _extract_longest_cycle(
    coordinates: np.ndarray,
    adjacency: Sequence[Sequence[int]],
    active: np.ndarray,
) -> np.ndarray:
    parent = np.full(len(adjacency), -2, dtype=np.int32)
    non_tree_edges = set()
    for root in np.flatnonzero(active):
        root = int(root)
        if parent[root] != -2:
            continue
        parent[root] = -1
        stack = [root]
        while stack:
            node = stack.pop()
            for neighbor in adjacency[node]:
                if not active[neighbor]:
                    continue
                if parent[neighbor] == -2:
                    parent[neighbor] = node
                    stack.append(neighbor)
                elif parent[node] != neighbor and parent[neighbor] != node:
                    non_tree_edges.add(tuple(sorted((node, neighbor))))

    best_path: List[int] = []
    best_length = 0.0
    for first, second in non_tree_edges:
        path = _tree_path(parent, first, second)
        if len(path) < 8:
            continue
        points = coordinates[path].astype(float)
        closed = np.vstack((points, points[0]))
        length = float(np.sum(np.linalg.norm(np.diff(closed, axis=0), axis=1)))
        if length > best_length:
            best_length = length
            best_path = path
    if not best_path:
        raise PlanningError(
            'The selected free-space component has no closed centerline. '
            'Check that both track walls form a closed loop.')
    return coordinates[np.asarray(best_path, dtype=int)]


def _resample_closed_curve(points: np.ndarray, spacing: float, smoothing: float) -> np.ndarray:
    if len(points) < 8:
        raise PlanningError('Not enough centerline points to resample')
    step_guess = float(np.median(np.linalg.norm(np.roll(points, -1, axis=0) - points, axis=1)))
    if smoothing > 0.0 and step_guess > 0.0:
        sigma = min(smoothing / step_guess, max(len(points) / 50.0, 1.0))
        points = np.column_stack((
            gaussian_filter1d(points[:, 0], sigma=sigma, mode='wrap'),
            gaussian_filter1d(points[:, 1], sigma=sigma, mode='wrap'),
        ))
    closed = np.vstack((points, points[0]))
    segment = np.linalg.norm(np.diff(closed, axis=0), axis=1)
    keep = np.concatenate(([True], segment[:-1] > 1.0e-8))
    points = points[keep]
    closed = np.vstack((points, points[0]))
    segment = np.linalg.norm(np.diff(closed, axis=0), axis=1)
    distance = np.concatenate(([0.0], np.cumsum(segment)))
    total = float(distance[-1])
    if total < 10.0 * spacing:
        raise PlanningError('Extracted centerline is too short')
    sample_count = max(20, int(math.ceil(total / spacing)))
    sample_distance = np.linspace(0.0, total, sample_count, endpoint=False)
    spline_x = CubicSpline(distance, closed[:, 0], bc_type='periodic')
    spline_y = CubicSpline(distance, closed[:, 1], bc_type='periodic')
    return np.column_stack((spline_x(sample_distance), spline_y(sample_distance)))


def _orient_and_rotate_cycle(
    points: np.ndarray,
    config: PlannerConfig,
) -> np.ndarray:
    if config.seed_x is not None and config.seed_y is not None:
        seed = np.array([config.seed_x, config.seed_y])
        start = int(np.argmin(np.linalg.norm(points - seed, axis=1)))
        points = np.roll(points, -start, axis=0)
    if config.direction == 'auto':
        should_reverse = config.reverse
        if config.seed_yaw is not None:
            forward = points[1] - points[-1]
            heading = np.array([
                math.cos(config.seed_yaw), math.sin(config.seed_yaw)])
            if float(np.dot(forward, heading)) < 0.0:
                should_reverse = not should_reverse
    else:
        following = np.roll(points, -1, axis=0)
        signed_twice_area = float(np.sum(
            points[:, 0] * following[:, 1] -
            following[:, 0] * points[:, 1]))
        is_counterclockwise = signed_twice_area > 0.0
        wants_counterclockwise = config.direction == 'counterclockwise'
        should_reverse = is_counterclockwise != wants_counterclockwise
    if should_reverse:
        points = np.concatenate((points[:1], points[:0:-1]), axis=0)
    return points


def build_centerline(map_data: MapData, config: PlannerConfig):
    """Return the selected track mask and oriented, resampled centerline."""
    track_mask = _select_track_component(
        map_data, config.seed_x, config.seed_y)
    # Thin free-space spurs and loops in SLAM maps can be connected to the
    # actual track.  Skeletonizing the raw component lets those artifacts
    # participate in cycle selection, which can pull the centerline into a
    # passage that is much narrower than the vehicle.  Extract topology from
    # the traversable core instead, while retaining the original component for
    # track-width measurement and clearance validation.
    clearance = cv2.distanceTransform(
        track_mask.astype(np.uint8), cv2.DIST_L2, 5)
    clearance *= map_data.resolution
    traversable_core = track_mask & (
        clearance + 0.5 * map_data.resolution >= config.required_clearance)
    if np.count_nonzero(traversable_core) < 100:
        raise PlanningError(
            'Track contains too little free space at the required vehicle '
            'clearance. Check the map or vehicle dimensions.')

    skeleton = _skeletonize_component(traversable_core)
    coordinates, adjacency = _pixel_graph(skeleton)
    active = _prune_leaves(adjacency)
    cycle_pixels = _extract_longest_cycle(coordinates, adjacency, active)
    raw = map_data.pixel_to_world(cycle_pixels[:, 0], cycle_pixels[:, 1])
    centerline = _resample_closed_curve(
        raw, config.spacing, config.centerline_smoothing)
    return track_mask, _orient_and_rotate_cycle(centerline, config)
