"""Eight-connected grid planning. Coursework starter ancestry: see docs.

Cells are closed unit squares centered at integer north/east coordinates.
Nonzero cells are occupied. No diagonal corner cutting is allowed.
"""

from enum import Enum
from numbers import Integral
from queue import PriorityQueue
import math

import numpy as np

EPSILON = 1.0e-10


def create_grid(data, drone_altitude, safety_distance):
    """Build a conservative 1 m grid from N,E,alt,dN,dE,dAlt obstacles.

    Inflate before determining bounds. Mark every closed cell intersecting an
    inflated horizontal box, including contact at the requested flight altitude.
    Returns (grid, north_offset, east_offset); centers map to local = cell + offset.
    """
    data = np.asarray(data, dtype=float)
    if (data.ndim != 2 or data.shape[1] != 6 or not len(data)
            or not np.isfinite(data).all() or (data[:, 3:] < 0).any()):
        raise ValueError("Obstacle data must be a nonempty finite N x 6 array")
    if (not math.isfinite(drone_altitude) or not math.isfinite(safety_distance)
            or safety_distance < 0):
        raise ValueError("Altitude must be finite and safety distance nonnegative")
    lower = data[:, :2] - data[:, 3:5] - safety_distance
    upper = data[:, :2] + data[:, 3:5] + safety_distance
    offsets = np.floor(lower.min(axis=0) - 0.5).astype(int)
    maxima = np.ceil(upper.max(axis=0) + 0.5).astype(int)
    grid = np.zeros(tuple(maxima - offsets + 1), dtype=np.uint8)
    for row, lo, hi in zip(data, lower, upper):
        if row[2] + row[5] + safety_distance >= drone_altitude:
            first = np.ceil(lo - 0.5 - offsets).astype(int)
            last = np.floor(hi + 0.5 - offsets).astype(int)
            grid[first[0]:last[0] + 1, first[1]:last[1] + 1] = 1
    return grid, int(offsets[0]), int(offsets[1])


class Action(Enum):
    """North/east deltas and graph costs (1 or sqrt(2))."""

    WEST = (0, -1, 1.0)
    EAST = (0, 1, 1.0)
    NORTH = (-1, 0, 1.0)
    SOUTH = (1, 0, 1.0)
    NORTH_EAST = (-1, 1, math.sqrt(2))
    NORTH_WEST = (-1, -1, math.sqrt(2))
    SOUTH_EAST = (1, 1, math.sqrt(2))
    SOUTH_WEST = (1, -1, math.sqrt(2))

    @property
    def cost(self):
        return self.value[2]

    @property
    def delta(self):
        return self.value[:2]


def is_free(grid, node):
    """Return False for invalid, out-of-bounds or occupied integer cells."""
    return (getattr(grid, "ndim", 0) == 2 and len(node) == 2
            and all(isinstance(x, Integral) for x in node)
            and 0 <= node[0] < grid.shape[0]
            and 0 <= node[1] < grid.shape[1] and grid[node[0], node[1]] == 0)


def valid_actions(grid, current_node):
    """Free successors; diagonal moves require both side cells to be free."""
    if not is_free(grid, current_node):
        return []
    x, y = current_node
    valid = []
    for action in Action:
        dx, dy = action.delta
        if not is_free(grid, (x + dx, y + dy)):
            continue
        if dx and dy and not (is_free(grid, (x + dx, y))
                              and is_free(grid, (x, y + dy))):
            continue
        valid.append(action)
    return valid


def _result(path, cost, stats, return_stats):
    stats["explored"] = stats["expanded"]  # Existing stats-key compatibility.
    return (path, cost, stats) if return_stats else (path, cost)


def a_star(grid, h, start, goal, return_stats=False):
    """Find a minimum-cost path with admissible h and h(goal, goal) = 0.

    Reopen improved nodes, so consistency is not required. Invalid/unreachable
    endpoints return ([], 0.0), preserving the original failure convention.
    expanded counts successor-list evaluations, excluding the goal.
    """
    stats = {"expanded": 0, "stale_entries": 0, "reopened": 0}
    if not is_free(grid, start) or not is_free(grid, goal):
        return _result([], 0.0, stats, return_stats)
    start, goal = tuple(start), tuple(goal)
    queue = PriorityQueue()
    queue.put((h(start, goal), 0.0, start))
    best_cost, branch, closed = {start: 0.0}, {}, set()
    while not queue.empty():
        _, current_cost, current_node = queue.get()
        if current_cost != best_cost[current_node] or current_node in closed:
            stats["stale_entries"] += 1
            continue
        if current_node == goal:
            path = [goal]
            while path[-1] != start:
                path.append(branch[path[-1]])
            return _result(path[::-1], current_cost, stats, return_stats)
        closed.add(current_node)
        stats["expanded"] += 1
        for action in valid_actions(grid, current_node):
            dx, dy = action.delta
            next_node = (current_node[0] + dx, current_node[1] + dy)
            next_cost = current_cost + action.cost
            if next_cost < best_cost.get(next_node, math.inf):
                best_cost[next_node] = next_cost
                branch[next_node] = current_node
                if next_node in closed:
                    closed.remove(next_node)
                    stats["reopened"] += 1
                queue.put((next_cost + h(next_node, goal), next_cost, next_node))
    return _result([], 0.0, stats, return_stats)


def iterative_astar(grid, h, start, goal, return_stats=False):
    """IDA* with explicit bounded DFS, ordered successors and cycle prevention.

    A dominance key includes the forbidden ancestor set: node-only dominance
    would conflate different available continuations. The table resets each
    threshold iteration. This conservative table can require exponential memory;
    the implementation does not claim classical linear-memory IDA* behavior.
    """
    stats = {"expanded": 0, "iterations": 0, "iteration_expanded": [],
             "thresholds": [], "dominated": 0}
    if not is_free(grid, start) or not is_free(grid, goal):
        return _result([], 0.0, stats, return_stats)
    start, goal = tuple(start), tuple(goal)
    if start == goal:
        return _result([start], 0.0, stats, return_stats)
    threshold = h(start, goal)
    while True:
        stats["iterations"] += 1
        stats["thresholds"].append(float(threshold))
        expanded, next_threshold = 0, math.inf
        path, path_nodes, best_path_cost = [start], {start}, {}
        # Each frame stores g and either None or its ordered successor iterator.
        stack = [[0.0, None]]
        while stack:
            cost, successors = stack[-1]
            node = path[-1]
            if successors is None:
                estimate = cost + h(node, goal)
                if estimate > threshold + EPSILON:
                    next_threshold = min(next_threshold, estimate)
                elif node == goal:
                    stats["expanded"] += expanded
                    stats["iteration_expanded"].append(expanded)
                    return _result(list(path), cost, stats, return_stats)
                else:
                    key = (node, frozenset(path_nodes))
                    if cost >= best_path_cost.get(key, math.inf):
                        stats["dominated"] += 1
                    else:
                        best_path_cost[key] = cost
                        expanded += 1
                        candidates = []
                        for action in valid_actions(grid, node):
                            dx, dy = action.delta
                            nxt = (node[0] + dx, node[1] + dy)
                            if nxt not in path_nodes:
                                g = cost + action.cost
                                candidates.append((g + h(nxt, goal), g, nxt))
                        candidates.sort(key=lambda item: item[0])
                        stack[-1][1] = iter(candidates)
                        continue
                stack.pop()
                path_nodes.remove(path.pop())
                continue
            candidate = next(successors, None)
            if candidate is None:
                stack.pop()
                path_nodes.remove(path.pop())
            else:
                _, g, nxt = candidate
                path.append(nxt)
                path_nodes.add(nxt)
                stack.append([g, None])
        stats["expanded"] += expanded
        stats["iteration_expanded"].append(expanded)
        if math.isinf(next_threshold):
            return _result([], 0.0, stats, return_stats)
        threshold = next_threshold


def heuristic(position, goal_position):
    """Euclidean lower bound for the eight-connected graph."""
    return math.hypot(position[0] - goal_position[0],
                      position[1] - goal_position[1])


def heuristic_octile(position, goal_position):
    """Exact obstacle-free distance for costs 1 and sqrt(2)."""
    dx, dy = (abs(position[i] - goal_position[i]) for i in (0, 1))
    return max(dx, dy) + (math.sqrt(2) - 1) * min(dx, dy)


def bresenham(p1, p2):
    """Conservative supercover traversal; legacy function name retained.

    Enumerate every closed unit cell touched by a center-to-center segment.
    Exact integer comparisons identify corner ties and include both side cells.
    """
    if not all(isinstance(v, Integral) for p in (p1, p2) for v in p):
        raise ValueError("Supercover endpoints must be integer cell centers")
    x, y = p1
    nx, ny = abs(p2[0] - x), abs(p2[1] - y)
    sx, sy = (1 if p2[0] > x else -1), (1 if p2[1] > y else -1)
    ix = iy = 0
    cells = [(x, y)]
    while ix < nx or iy < ny:
        difference = (1 + 2 * ix) * ny - (1 + 2 * iy) * nx
        if difference == 0:
            cells.extend([(x + sx, y), (x, y + sy)])
            x, y, ix, iy = x + sx, y + sy, ix + 1, iy + 1
        elif difference < 0:
            x, ix = x + sx, ix + 1
        else:
            y, iy = y + sy, iy + 1
        cells.append((x, y))
    return cells


def line_of_sight(grid, p1, p2):
    """Closed-cell visibility; obstacles must already include safety inflation."""
    return all(is_free(grid, cell) for cell in bresenham(p1, p2))


def prune_path(path, grid):
    """Greedily shorten a valid route without changing endpoint constraints.

    Raises ValueError for an unsafe input, including short paths. Apply this
    separately to each checkpoint segment so required endpoints survive.
    """
    path = list(path)
    if (any(not is_free(grid, p) for p in path)
            or any(not line_of_sight(grid, a, b)
                   for a, b in zip(path, path[1:]))):
        raise ValueError("Input path contains an occupied cell or unsafe segment")
    if not path:
        return []
    pruned, i = [path[0]], 0
    while i < len(path) - 1:
        j = len(path) - 1
        while j > i + 1 and not line_of_sight(grid, path[i], path[j]):
            j -= 1
        if path[j] != pruned[-1]:
            pruned.append(path[j])
        i = j
    return pruned


def a_star_through_points(grid, h, start, fixed_points, goal,
                          return_stats=False):
    """Join independently pruned segments in the specified checkpoint order.

    Cost is the sum of optimal raw grid costs, not the pruned geometric length.
    Any invalid/unreachable segment returns ([], inf). Consecutive repeated
    checkpoints are fulfilled by one visit. Optional stats expose both lengths.
    """
    checkpoints = [start] + list(fixed_points) + [goal]
    complete_path, total_cost = [], 0.0
    stats = {"expanded": 0, "raw_path_length": 0,
             "geometric_distance": math.inf, "segments": []}
    for a, b in zip(checkpoints, checkpoints[1:]):
        segment, cost, search_stats = a_star(grid, h, a, b, True)
        stats["expanded"] += search_stats["expanded"]
        if not segment:
            return _result([], math.inf, stats, return_stats)
        stats["raw_path_length"] += len(segment) - (1 if complete_path else 0)
        total_cost += cost
        pruned = prune_path(segment, grid)
        complete_path.extend(pruned[1:] if complete_path else pruned)
        stats["segments"].append({"start": a, "goal": b, "grid_cost": cost})
    stats["geometric_distance"] = sum(
        heuristic(a, b) for a, b in zip(complete_path, complete_path[1:]))
    return _result(complete_path, total_cost, stats, return_stats)
