import math

import numpy as np
import pytest

from src.mission_config import contains_in_order, load_mission, ordered_route
from src.planning_utils import a_star_through_points, heuristic, heuristic_octile
from tests.reference import assert_safe_segments, dijkstra


@pytest.mark.parametrize("points", [[(0, 4), (4, 0), (4, 4)],
                                     [(0, 0), (0, 0), (4, 4), (4, 4)],
                                     [(0, 4), (4, 0), (0, 4)]])
def test_checkpoint_order_and_cost(points):
    grid = np.zeros((5, 5))
    path, cost, stats = a_star_through_points(grid, heuristic_octile, (0, 0), points, (4, 4), True)
    assert contains_in_order(path, points)
    sequence = [(0, 0)] + points + [(4, 4)]
    expected = sum(dijkstra(grid, a, b) for a, b in zip(sequence, sequence[1:]))
    assert cost == pytest.approx(expected)
    assert all(a != b for a, b in zip(path, path[1:]))
    assert stats["geometric_distance"] == pytest.approx(sum(heuristic(a, b) for a, b in zip(path, path[1:])))
    assert_safe_segments(grid, path)


def test_unreachable_intermediate_and_invalid_goal():
    grid = np.zeros((3, 3))
    grid[1, :] = 1
    for points, goal in [([(2, 2)], (0, 2)), ([], (5, 5)), ([(1, 1)], (0, 2))]:
        assert a_star_through_points(grid, heuristic_octile, (0, 0), points, goal) == ([], math.inf)


def test_configuration_defaults_and_optional_shaping():
    config = load_mission()
    route, required, goal = ordered_route(config, (0, 0))
    assert route == required == [(20, 30), (40, -5), (60, 30)]
    assert goal == (155, -15)
    config["use_route_shaping"] = True
    shaped, _, _ = ordered_route(config, (0, 0))
    assert len(shaped) == 11 and contains_in_order(shaped, required)
    reverse, required_reverse, goal = ordered_route(config, (155, -15))
    assert reverse == shaped[::-1] and required_reverse == required[::-1]
    assert goal == (0, 0)
