"""Shared synthetic fixture and plotting for runnable examples."""

from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]


def synthetic_mission():
    grid = np.zeros((9, 12), dtype=np.uint8)
    grid[3:6, 5:7] = 1
    grid[6, 9] = 1
    return grid, (1, 1), [(1, 8), (7, 2), (7, 8)], (7, 10)


def plot_route(grid, path, start, goal, checkpoints, output):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.colors import ListedColormap
    from matplotlib.patches import Patch

    fig, ax = plt.subplots(figsize=(9, 6), layout="constrained")
    ax.imshow(grid, origin="lower", cmap=ListedColormap(["#f4f7fb", "#39485d"]),
              vmin=0, vmax=1, interpolation="nearest")
    if path:
        ax.plot([p[1] for p in path], [p[0] for p in path], "o-",
                color="#276ac7", markersize=4, label="Pruned planned route")
    ax.scatter(start[1], start[0], s=140, marker="s", color="#15936d", label="Start", zorder=5)
    ax.scatter(goal[1], goal[0], s=180, marker="*", color="#c23c50", label="Goal", zorder=5)
    if checkpoints:
        ax.scatter([p[1] for p in checkpoints], [p[0] for p in checkpoints],
                   s=110, marker="D", color="#ed9e19", edgecolor="black",
                   label="Required checkpoints (ordered)", zorder=5)
        for i, p in enumerate(checkpoints, 1):
            ax.annotate("C{}".format(i), (p[1], p[0]), xytext=(7, 7), textcoords="offset points")
    ax.set_xticks(np.arange(-0.5, grid.shape[1], 1), minor=True)
    ax.set_yticks(np.arange(-0.5, grid.shape[0], 1), minor=True)
    ax.grid(which="minor", color="#ccd4df", linewidth=0.6)
    ax.set(xlabel="East (grid cells)", ylabel="North (grid cells)",
           title="Ordered UAV route on a synthetic occupancy grid\nPlanning only; no simulator flight")
    handles, _ = ax.get_legend_handles_labels()
    handles.append(Patch(facecolor="#39485d", label="Occupied cell"))
    ax.legend(handles=handles, loc="upper left", bbox_to_anchor=(1.01, 1))
    output = Path(output)
    output.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(output, dpi=180)
    plt.close(fig)


def measure_search(grid, start, goal):
    from time import perf_counter
    from src.planning_utils import (a_star, heuristic, heuristic_octile,
                                    iterative_astar, prune_path)

    rows = []
    for label, solver, h in [("A* Euclidean", a_star, heuristic),
                             ("A* Octile", a_star, heuristic_octile),
                             ("IDA* Octile", iterative_astar, heuristic_octile)]:
        begin = perf_counter()
        path, cost, stats = solver(grid, h, start, goal, return_stats=True)
        elapsed = perf_counter() - begin
        if not path:
            raise RuntimeError("Benchmark route is unreachable")
        rows.append({"method": label, "search_cost": cost,
                     "expanded_nodes": stats["expanded"], "elapsed_seconds": elapsed,
                     "raw_path_length": len(path),
                     "pruned_waypoint_count": len(prune_path(path, grid))})
    return rows
