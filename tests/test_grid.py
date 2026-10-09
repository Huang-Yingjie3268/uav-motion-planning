import numpy as np
import pytest

from src.planning_utils import create_grid, is_free


def test_inflation_bounds_altitude_equality_and_coordinate_round_trip():
    data = np.array([[2.2, -3.2, 3, 0.2, 0.2, 1]])
    grid, north, east = create_grid(data, 5, 1)
    # Height + extent + margin equals altitude: obstacle must remain occupied.
    for local in [(1, -4), (2, -3), (3, -2)]:
        node = (local[0] - north, local[1] - east)
        assert grid[node] == 1
        assert (node[0] + north, node[1] + east) == local
    assert is_free(grid, (0, 0))
    free, _, _ = create_grid(data, 5.01, 1)
    assert not free.any()


@pytest.mark.parametrize("data,altitude,safety", [([], 5, 1), ([[0] * 6], 5, -1),
                                                   ([[0, 0, 0, -1, 0, 0]], 5, 1),
                                                   ([[0] * 6], float("nan"), 1)])
def test_invalid_grid_data(data, altitude, safety):
    with pytest.raises(ValueError):
        create_grid(data, altitude, safety)
