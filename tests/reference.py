"""Independent test oracles: no production movement or visibility helpers."""

import heapq
import math


def neighbors(grid, node):
    r, c = node
    rows, cols = grid.shape
    for dr in (-1, 0, 1):
        for dc in (-1, 0, 1):
            if dr == dc == 0:
                continue
            rr, cc = r + dr, c + dc
            if not (0 <= rr < rows and 0 <= cc < cols) or grid[rr, cc] != 0:
                continue
            if dr and dc and (grid[r + dr, c] != 0 or grid[r, c + dc] != 0):
                continue
            yield (rr, cc), math.sqrt(2) if dr and dc else 1.0


def dijkstra(grid, start, goal):
    for r, c in (start, goal):
        if not (0 <= r < grid.shape[0] and 0 <= c < grid.shape[1]) or grid[r, c]:
            return math.inf
    queue, costs = [(0.0, start)], {start: 0.0}
    while queue:
        cost, node = heapq.heappop(queue)
        if cost != costs[node]:
            continue
        if node == goal:
            return cost
        for nxt, step in neighbors(grid, node):
            new = cost + step
            if new < costs.get(nxt, math.inf):
                costs[nxt] = new
                heapq.heappush(queue, (new, nxt))
    return math.inf


def assert_search_path(grid, path, cost, start, goal):
    assert path[0] == start and path[-1] == goal
    total = 0.0
    for a, b in zip(path, path[1:]):
        options = dict(neighbors(grid, a))
        assert b in options
        total += options[b]
    assert math.isclose(total, cost, abs_tol=1e-9)


def intersects_closed_cell(a, b, cell):
    """Independent slab intersection against a closed unit square."""
    low, high = 0.0, 1.0
    for axis in (0, 1):
        left, right = cell[axis] - 0.5, cell[axis] + 0.5
        delta = b[axis] - a[axis]
        if delta == 0:
            if not left <= a[axis] <= right:
                return False
        else:
            t1, t2 = (left - a[axis]) / delta, (right - a[axis]) / delta
            low, high = max(low, min(t1, t2)), min(high, max(t1, t2))
            if low > high + 1e-12:
                return False
    return True


def assert_safe_segments(grid, path):
    obstacles = list(zip(*grid.nonzero()))
    for a, b in zip(path, path[1:]):
        assert not any(intersects_closed_cell(a, b, cell) for cell in obstacles)
