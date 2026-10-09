"""Measure deterministic synthetic search; optional external colliders map."""

import argparse
import csv
import hashlib
import json
from pathlib import Path
import platform

import numpy as np

from examples.common import ROOT, measure_search, synthetic_mission
from src.planning_utils import create_grid


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--map", type=Path, help="Optional private colliders.csv")
    parser.add_argument("--start", type=int, nargs=2, default=(0, 0), help="Local north/east for external map")
    parser.add_argument("--goal", type=int, nargs=2, default=(155, -15))
    parser.add_argument("--altitude", type=float, default=5.0)
    parser.add_argument("--safety", type=float, default=5.0)
    parser.add_argument("--output", type=Path, default=ROOT / "assets" / "benchmark.csv")
    args = parser.parse_args()
    grid, start, _, goal = synthetic_mission()
    description = "Synthetic 9 x 12 grid from examples.common.synthetic_mission"
    if args.map:
        data = np.loadtxt(args.map, delimiter=",", skiprows=2, ndmin=2)
        grid, north, east = create_grid(data, args.altitude, args.safety)
        start = (args.start[0] - north, args.start[1] - east)
        goal = (args.goal[0] - north, args.goal[1] - east)
        description = "External colliders grid; map not redistributed"
    rows = measure_search(grid, start, goal)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    with args.output.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)
    metadata = {"python": platform.python_version(), "numpy": np.__version__,
                "map": description, "grid_shape": list(grid.shape), "start": start, "goal": goal,
                "altitude": args.altitude if args.map else None,
                "safety_distance": args.safety if args.map else None,
                "timing": "One perf_counter sample, search only; not a stable speed ranking",
                "expansions": "Successor-list evaluations; excludes goal and stale entries; IDA* sums all iterations"}
    if args.map:
        metadata["map_sha256"] = hashlib.sha256(args.map.read_bytes()).hexdigest()
    args.output.with_suffix(".json").write_text(json.dumps(metadata, indent=2) + "\n", encoding="utf-8")
    for row in rows:
        print(row)
    if not args.map:
        plot_comparison(rows, args.output.parent / "heuristic_comparison.png")


def plot_comparison(rows, output):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    fig, axes = plt.subplots(1, 2, figsize=(10, 4), layout="constrained")
    labels = [r["method"] for r in rows]
    for ax, key, title in [(axes[0], "expanded_nodes", "Successor-list evaluations"),
                            (axes[1], "search_cost", "Optimal raw grid cost")]:
        bars = ax.bar(labels, [r[key] for r in rows], color=["#708db5", "#276ac7", "#15936d"])
        ax.bar_label(bars, fmt="%.3f" if key == "search_cost" else "%d", padding=3)
        ax.set_title(title)
        ax.set_ylim(0, max(r[key] for r in rows) * 1.25)
    fig.suptitle("Synthetic 9 x 12 grid; identical endpoints and movement rules\nIDA* expansions summed across all threshold iterations")
    fig.savefig(output, dpi=180)
    plt.close(fig)


if __name__ == "__main__":
    main()
