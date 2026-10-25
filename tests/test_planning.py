"""Continuous geometry, graph-search correctness, and the path/trajectory bridge."""

from pathlib import Path

import numpy as np
import pytest

from autonomous_explorer.control import invert_acceleration
from autonomous_explorer.evidence import read_json
from autonomous_explorer.path_reference import build_rounded_reference
from autonomous_explorer.planning import (
    World,
    grid_search,
    rover_radius,
    segment_intersects_box,
    shortcut_path,
)

CONFIG = Path(__file__).parents[1] / "configs/navigation.json"


@pytest.mark.parametrize(
    "a,b,expected",
    [
        ([0, 0], [4, 4], True),
        ([0, 2], [4, 2], True),
        ([0, 1], [4, 1], True),
        ([0, 0], [0, 4], False),
        ([1.5, 1.5], [1.5, 1.5], True),
        ([0, 0], [0, 0], False),
        ([0, 0], [1, 1], True),
    ],
)
def test_closed_segment_box_intersections(a, b, expected):
    assert segment_intersects_box(a, b, [1, 1, 3, 3]) == expected


def test_thin_obstacle_cannot_be_skipped_between_free_endpoints():
    world = World((0, 5, 0, 5), ((2.49, 0, 2.51, 5),))
    assert world.point_free((1, 2)) and world.point_free((4, 2))
    assert not world.segment_free((1, 2), (4, 2))
    result = grid_search(world, [1, 2], [4, 2], 1)
    assert result.status == "unreachable_on_grid"


def test_rover_radius_matches_the_preserved_rendered_geometry():
    from autonomous_explorer.theme import BODY_LENGTH_M, TRACK_M, WHEEL_WIDTH_M

    config = read_json(CONFIG)
    assert config["body_length_m"] == BODY_LENGTH_M
    assert config["outer_wheel_width_m"] == pytest.approx(TRACK_M + WHEEL_WIDTH_M)
    radius = rover_radius(config)
    corners = np.array([[x, y] for x in [-0.24, 0.24] for y in [-0.225, 0.225]])
    assert np.all(np.linalg.norm(corners, axis=1) <= radius + 1e-14)
    for theta in np.linspace(-np.pi, np.pi, 31):
        rotation = np.array([[np.cos(theta), -np.sin(theta)], [np.sin(theta), np.cos(theta)]])
        assert np.all(np.linalg.norm(corners @ rotation, axis=1) <= radius + 1e-14)


def test_inflation_and_signed_clearance():
    world = World((0, 10, 0, 10), ((3, 3, 5, 5),))
    inflated = world.inflate(0.5)
    assert inflated.bounds == (0.5, 9.5, 0.5, 9.5)
    assert inflated.obstacles == ((2.5, 2.5, 5.5, 5.5),)
    assert not inflated.point_free([2.5, 4])
    np.testing.assert_allclose(world.clearance([[2, 4], [4, 4], [0.2, 1]], 0.3), [0.7, -1.3, -0.1])
    assert not inflated.point_free([0.5, 1])


def test_astar_matches_dijkstra_and_uses_fewer_expansions_on_empty_grid():
    world = World((0, 8, 0, 8), ())
    astar = grid_search(world, [1, 1], [6, 6], 1, "astar")
    dijkstra = grid_search(world, [1, 1], [6, 6], 1, "dijkstra")
    assert astar.status == dijkstra.status == "found"
    assert astar.cost == pytest.approx(5 * np.sqrt(2))
    assert astar.cost == pytest.approx(dijkstra.cost)
    assert len(astar.expanded) < len(dijkstra.expanded)
    for path in [astar.path, dijkstra.path]:
        for a, b in zip(path[:-1], path[1:], strict=True):
            assert world.segment_free(a, b)


def test_search_is_deterministic_and_refuses_silent_snapping():
    world = World((0, 8, 0, 8), ())
    first, second = [grid_search(world, [1, 1], [6, 6], 0.5) for _ in range(2)]
    np.testing.assert_array_equal(first.path, second.path)
    np.testing.assert_array_equal(first.expanded, second.expanded)
    with pytest.raises(ValueError, match="no implicit snapping"):
        grid_search(world, [1.1, 1], [6, 6], 0.5)
    assert grid_search(world, [0, 0], [6, 6], 1).status == "invalid_endpoint"


def test_diagonal_cannot_cut_a_blocked_grid_corner():
    world = World((0, 3, 0, 3), ((1.8, 0.8, 2.2, 1.2), (0.8, 1.8, 1.2, 2.2)))
    assert world.segment_free([1, 1], [2, 2])  # Geometry alone permits this thin gap.
    assert grid_search(world, [1, 1], [2, 2], 1).status == "unreachable_on_grid"


def test_shortcut_checked_as_entire_segments():
    world = World((0, 8, 0, 8), ((3, 0, 4, 4),))
    result = grid_search(world, [1, 1], [6, 6], 0.5)
    reduced = shortcut_path(result.path, world)
    assert len(reduced) < len(result.path)
    np.testing.assert_array_equal(reduced[[0, -1]], result.path[[0, -1]])
    assert not world.segment_free(result.path[0], result.path[-1])
    assert all(world.segment_free(a, b) for a, b in zip(reduced[:-1], reduced[1:], strict=True))


def example_reference():
    world = World((0, 6, 0, 6), ((1.1, 3.5, 1.8, 3.9),))
    path = np.array([[1, 1], [1, 4], [4, 4]])
    reference, certificate = build_rounded_reference(path, world, 0.4, 0.4, 1.0, 0.02)
    return world, reference, certificate


def test_blind_corner_rounding_would_collide_but_certificate_shrinks_it():
    world, reference, certificate = example_reference()
    # The original large 90-degree corner has u=1/2 at (1.3375, 3.6625), inside the obstacle.
    assert not world.point_free([1.3375, 3.6625])
    assert certificate["corner_rounding"][0]["halvings"] > 0
    states, _, _ = reference.arrays(np.linspace(0, reference.duration, 2001))
    assert all(world.point_free(p) for p in states[:, :2])
    for points in reference.controls:
        if np.linalg.norm(points[2] - 2 * points[1] + points[0]) > 1e-12:
            assert world.hull_box_free(points)
        else:
            assert world.segment_free(points[0], points[2])


def test_rounded_reference_c1_and_nonzero_endpoints():
    _, reference, _ = example_reference()
    for k in range(len(reference.controls) - 1):
        previous, following = reference.controls[k], reference.controls[k + 1]
        np.testing.assert_allclose(previous[-1], following[0], atol=1e-12)
        left_velocity = 2 * (previous[2] - previous[1]) / reference.durations[k]
        right_velocity = 2 * (following[1] - following[0]) / reference.durations[k + 1]
        np.testing.assert_allclose(left_velocity, right_velocity, atol=1e-12)
    assert reference.sample(0).state.v > 0
    assert reference.sample(reference.duration).state.v > 0
    np.testing.assert_allclose(reference.sample(0).state.array()[:2], [1, 1])
    np.testing.assert_allclose(reference.sample(reference.duration).state.array()[:2], [4, 4])


def test_timing_bounds_cover_between_sample_extrema_and_derivatives():
    _, reference, certificate = example_reference()
    bounds = certificate["bounds"]
    assert bounds["max_speed_mps"] <= 0.4 + 1e-12
    assert bounds["max_abs_acceleration_mps2"] <= 0.4 + 1e-12
    assert bounds["max_abs_yaw_rate_radps"] <= 1.0 + 1e-12
    assert reference.duration / 0.02 == pytest.approx(round(reference.duration / 0.02))
    for t in np.linspace(0, reference.duration, 501):
        sample = reference.sample(float(t))
        u = invert_acceleration(sample.state, sample.acceleration)
        assert bounds["min_speed_mps"] - 1e-12 <= sample.state.v <= bounds["max_speed_mps"] + 1e-12
        assert abs(u[0]) <= bounds["max_abs_acceleration_mps2"] + 1e-12
        assert abs(u[1]) <= bounds["max_abs_yaw_rate_radps"] + 1e-12
    t = reference.boundaries[1] + reference.durations[1] * 0.3
    h = 1e-5
    a, b, center = reference.sample(t - h), reference.sample(t + h), reference.sample(t)
    np.testing.assert_allclose(
        (b.state.array()[:2] - a.state.array()[:2]) / (2 * h), center.velocity, atol=1e-9
    )
    np.testing.assert_allclose((b.velocity - a.velocity) / (2 * h), center.acceleration, atol=1e-9)


def test_reversing_corner_is_explicitly_rejected():
    with pytest.raises(ValueError, match="Zero-speed"):
        build_rounded_reference(
            [[1, 1], [3, 1], [1, 1]], World((0, 5, 0, 5), ()), 0.4, 0.4, 1, 0.02
        )
