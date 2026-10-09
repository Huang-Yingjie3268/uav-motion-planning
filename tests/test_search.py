import math

import numpy as np
import pytest

from src.planning_utils import a_star, heuristic, heuristic_octile, iterative_astar, valid_actions
from tests.reference import assert_search_path, dijkstra

SOLVERS = [a_star, iterative_astar]


@pytest.mark.parametrize("solver", SOLVERS)
def test_list_endpoints(solver):
    path, cost = solver(np.zeros((2, 2)), heuristic_octile, [0, 0], [1, 1])
    assert path == [(0, 0), (1, 1)] and cost == pytest.approx(math.sqrt(2))


@pytest.mark.parametrize("solver", SOLVERS)
@pytest.mark.parametrize("start,goal", [((1, 1), (1, 1)), ((0, 0), (0, 4)),
                                        ((0, 0), (4, 4)), ((0, 2), (4, 2))])
def test_open_grid(solver, start, goal):
    grid = np.zeros((5, 5))
    path, cost = solver(grid, heuristic_octile, start, goal)
    assert_search_path(grid, path, cost, start, goal)
    assert cost == pytest.approx(dijkstra(grid, start, goal))


@pytest.mark.parametrize("solver", SOLVERS)
@pytest.mark.parametrize("start,goal", [((-1, 0), (2, 2)), ((0, 0), (3, 0)),
                                        ((0, 0), (0, 0)), ((2, 2), (0, 0)),
                                        ((0.5, 1), (2, 2))])
def test_invalid_endpoints(solver, start, goal):
    grid = np.zeros((3, 3))
    grid[0, 0] = 1
    assert solver(grid, heuristic_octile, start, goal) == ([], 0.0)


@pytest.mark.parametrize("solver", SOLVERS)
def test_blocked_corner_and_unreachable(solver):
    grid = np.array([[0, 1], [1, 0]])
    assert valid_actions(grid, (0, 0)) == []
    assert solver(grid, heuristic_octile, (0, 0), (1, 1)) == ([], 0.0)


@pytest.mark.parametrize("solver", SOLVERS)
def test_detour(solver):
    grid = np.zeros((4, 5))
    grid[:3, 2] = 1
    path, cost = solver(grid, heuristic_octile, (0, 0), (0, 4))
    assert_search_path(grid, path, cost, (0, 0), (0, 4))
    assert cost == pytest.approx(dijkstra(grid, (0, 0), (0, 4)))


@pytest.mark.parametrize("solver", SOLVERS)
def test_all_512_three_by_three_obstacle_masks(solver):
    """128 free-endpoint layouts, plus 384 invalid-endpoint layouts."""
    for mask in range(512):
        grid = np.array([(mask >> i) & 1 for i in range(9)]).reshape(3, 3)
        path, cost = solver(grid, heuristic_octile, (0, 0), (2, 2))
        expected = dijkstra(grid, (0, 0), (2, 2))
        if math.isinf(expected):
            assert path == [] and cost == 0.0
        else:
            assert_search_path(grid, path, cost, (0, 0), (2, 2))
            assert cost == pytest.approx(expected)


@pytest.mark.parametrize("seed", range(30))
def test_random_layouts_with_independent_dijkstra(seed):
    grid = (np.random.default_rng(seed).random((3, 4)) < 0.3).astype(int)
    grid[0, 0] = grid[2, 3] = 0
    expected = dijkstra(grid, (0, 0), (2, 3))
    for solver in SOLVERS:
        for h in (heuristic, heuristic_octile):
            path, cost = solver(grid, h, (0, 0), (2, 3))
            if math.isinf(expected):
                assert path == []
            else:
                assert_search_path(grid, path, cost, (0, 0), (2, 3))
                assert cost == pytest.approx(expected)


def test_admissible_inconsistent_heuristic_reopens_closed_nodes():
    reopened = stale = 0
    grid = np.zeros((5, 5))
    for seed in range(20):
        rng = np.random.default_rng(seed)
        estimates = {p: (dijkstra(grid, p, (4, 4)) if rng.random() < 0.5 else 0.0)
                     for p in np.ndindex(grid.shape)}
        h = lambda p, _: estimates[p]
        path, cost, stats = a_star(grid, h, (0, 0), (4, 4), True)
        assert_search_path(grid, path, cost, (0, 0), (4, 4))
        assert cost == pytest.approx(4 * math.sqrt(2))
        reopened += stats["reopened"]
        stale += stats["stale_entries"]
    assert reopened > 0
    assert stale > 0


def test_idastar_long_corridor_avoids_python_recursion_limit():
    grid = np.zeros((1, 1100))
    path, cost = iterative_astar(grid, heuristic_octile, (0, 0), (0, 1099))
    assert len(path) == 1100 and cost == 1099


def test_idastar_thresholds_and_total_expansions():
    grid = np.zeros((4, 5))
    grid[:3, 2] = 1
    path, cost, stats = iterative_astar(grid, heuristic_octile, (0, 0), (0, 4), True)
    assert cost == pytest.approx(dijkstra(grid, (0, 0), (0, 4)))
    assert stats["iterations"] > 1
    assert stats["expanded"] == sum(stats["iteration_expanded"])
    assert stats["thresholds"][0] == 4.0
    assert all(a < b for a, b in zip(stats["thresholds"], stats["thresholds"][1:]))
