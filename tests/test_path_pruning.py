import numpy as np
import pytest

from src.planning_utils import a_star, bresenham, heuristic_octile, line_of_sight, prune_path
from tests.reference import assert_safe_segments, intersects_closed_cell


def test_supercover_matches_independent_closed_square_oracle():
    for a in np.ndindex((5, 5)):
        for b in np.ndindex((5, 5)):
            expected = {c for c in np.ndindex((5, 5)) if intersects_closed_cell(a, b, c)}
            assert set(bresenham(a, b)) == expected
            assert set(bresenham(b, a)) == expected


def test_corner_contact_and_narrow_gap_are_blocked():
    grid = np.zeros((3, 3))
    grid[0, 1] = grid[1, 0] = 1
    assert not line_of_sight(grid, (0, 0), (2, 2))
    with pytest.raises(ValueError):
        prune_path([(0, 0), (1, 1)], grid)


def test_rounded_sampling_misses_touched_cell_regression():
    grid = np.zeros((4, 3))
    grid[1, 0] = 1
    assert not line_of_sight(grid, (0, 0), (3, 2))


def test_short_paths_and_invalid_adjacent_fallback():
    grid = np.zeros((3, 3))
    assert prune_path([], grid) == []
    assert prune_path([(0, 0)], grid) == [(0, 0)]
    grid[0, 1] = 1
    for path in [[(0, 1)], [(0, 0), (0, 2)], [(0, 0), (0, 2), (2, 2)]]:
        with pytest.raises(ValueError):
            prune_path(path, grid)


@pytest.mark.parametrize("seed", range(10))
def test_pruned_routes_are_geometrically_safe(seed):
    grid = (np.random.default_rng(seed).random((7, 7)) < 0.2).astype(int)
    grid[0, 0] = grid[6, 6] = 0
    path, _ = a_star(grid, heuristic_octile, (0, 0), (6, 6))
    if path:
        pruned = prune_path(path, grid)
        assert pruned[0] == path[0] and pruned[-1] == path[-1]
        assert_safe_segments(grid, pruned)
