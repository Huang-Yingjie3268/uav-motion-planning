import numpy as np
import pytest

from src.planning_utils import heuristic, heuristic_octile
from tests.reference import dijkstra, neighbors


@pytest.mark.parametrize("h", [heuristic, heuristic_octile])
def test_admissibility_and_consistency(h):
    grid = np.zeros((4, 5))
    grid[1:3, 2] = 1
    for goal in np.ndindex(grid.shape):
        if grid[goal]:
            continue
        assert h(goal, goal) == 0
        for node in np.ndindex(grid.shape):
            if grid[node]:
                continue
            assert h(node, goal) <= dijkstra(grid, node, goal) + 1e-10
            for nxt, cost in neighbors(grid, node):
                assert h(node, goal) <= cost + h(nxt, goal) + 1e-10
