"""Optional local map verification; never redistribute coursework map data."""

import os

import numpy as np
import pytest

from src.mission_config import contains_in_order, load_mission, ordered_route
from src.planning_utils import a_star, a_star_through_points, create_grid, heuristic_octile
from tests.reference import dijkstra


@pytest.mark.skipif(not os.environ.get("UAV_COLLIDERS_CSV"), reason="External colliders.csv not configured; set UAV_COLLIDERS_CSV for offline map validation")
def test_external_map_direct_and_checkpoint_routes():
    data = np.loadtxt(os.environ["UAV_COLLIDERS_CSV"], delimiter=",", skiprows=2, ndmin=2)
    grid, north, east = create_grid(data, 5, 5)
    convert = lambda p: (p[0] - north, p[1] - east)
    start, goal = convert((0, 0)), convert((155, -15))
    _, cost = a_star(grid, heuristic_octile, start, goal)
    assert cost == pytest.approx(dijkstra(grid, start, goal))
    config = load_mission()
    for shaping in (False, True):
        config["use_route_shaping"] = shaping
        for current in [(0, 0), (155, -15)]:
            route, required, destination = ordered_route(config, current)
            path, cost = a_star_through_points(grid, heuristic_octile, convert(current),
                                               [convert(p) for p in route], convert(destination))
            assert path and contains_in_order(path, [convert(p) for p in required])
