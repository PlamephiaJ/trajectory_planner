import numpy as np

from trajectory_planner.config import PlannerConfig
from trajectory_planner.optimization import _corner_anticipation_targets


def test_corner_anticipation_moves_outside_before_left_turn():
    # Counterclockwise closed path: its corners have positive (left) curvature.
    centerline = np.array([
        [0.0, 0.0], [1.0, 0.0], [2.0, 0.0], [2.7, 0.3],
        [3.0, 1.0], [3.0, 2.0], [3.0, 3.0], [2.0, 3.0],
        [1.0, 3.0], [0.0, 3.0], [0.0, 2.0], [0.0, 1.0],
    ])
    count = len(centerline)
    lower = np.full(count, -0.8)
    upper = np.full(count, 0.8)
    widths = np.full(count, 1.0)
    config = PlannerConfig(
        min_turning_radius=1.0,
        corner_lookahead_distance=2.0,
        corner_anticipation_weight=1.0,
        corner_anticipation_fraction=0.75,
        corner_trigger_fraction=0.30,
    )

    targets = _corner_anticipation_targets(
        centerline, widths, widths, lower, upper, 0.75, config)

    # Positive curvature is a left turn, so preparation must be on the right:
    # negative lateral offset along the left-pointing normal.
    assert np.any(targets < -1.0e-6)
    assert np.all(targets <= upper + 1.0e-12)
    assert np.all(targets >= lower - 1.0e-12)


def test_corner_anticipation_is_disabled_without_turning_radius():
    centerline = np.array([
        [0.0, 0.0], [1.0, 0.0], [1.0, 1.0], [0.0, 1.0],
    ])
    count = len(centerline)
    lower = np.full(count, -0.5)
    upper = np.full(count, 0.5)
    widths = np.full(count, 1.0)
    config = PlannerConfig(min_turning_radius=0.0)

    targets = _corner_anticipation_targets(
        centerline, widths, widths, lower, upper, 1.0, config)

    assert np.allclose(targets, 0.0)


def test_corner_anticipation_parameter_validation():
    invalid = [
        PlannerConfig(corner_lookahead_distance=-0.1),
        PlannerConfig(corner_anticipation_weight=-0.1),
        PlannerConfig(corner_anticipation_fraction=1.1),
        PlannerConfig(corner_trigger_fraction=0.0),
        PlannerConfig(corner_trigger_fraction=1.0),
    ]
    for config in invalid:
        try:
            config.validate()
        except ValueError:
            continue
        raise AssertionError(f'Invalid configuration was accepted: {config}')
