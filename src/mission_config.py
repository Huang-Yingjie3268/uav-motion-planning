"""Small JSON mission configuration, independent of the simulator SDK."""

import json
import math
from pathlib import Path

DEFAULT_CONFIG = Path(__file__).resolve().parents[1] / "config" / "mission.json"


def load_mission(path=DEFAULT_CONFIG):
    """Load and validate integer local checkpoints and finite control settings."""
    with open(str(path), encoding="utf-8") as stream:
        config = json.load(stream)
    positive = ["target_altitude", "transit_acceptance_radius",
                "fixed_point_acceptance_radius", "checkpoint_acceptance_radius",
                "turn_acceptance_radius", "final_acceptance_radius",
                "altitude_acceptance_radius", "transit_cross_track_limit"]
    for name in positive + ["safety_distance", "transit_flyby_progress"]:
        value = config[name]
        if isinstance(value, bool) or not isinstance(value, (int, float)):
            raise ValueError("{} must be numeric".format(name))
        if not math.isfinite(value) or value < 0 or (name in positive and value == 0):
            raise ValueError("Invalid mission setting: {}".format(name))
    if not 0 < config["transit_flyby_progress"] <= 1:
        raise ValueError("Flyby progress must be in (0, 1]")
    for name in ["use_route_shaping", "auto_reverse"]:
        if not isinstance(config[name], bool):
            raise ValueError("{} must be boolean".format(name))
    groups = config["shaping_after_checkpoint"]
    if len(groups) != len(config["required_checkpoints"]):
        raise ValueError("One shaping group is required per required checkpoint")
    points = (config["required_checkpoints"] + config["entry_points"]
              + config["cruise_points"] + [p for group in groups for p in group]
              + [config["route_origin"], config["final_destination"]])
    for point in points:
        if len(point) != 2 or any(type(v) is not int for v in point):
            raise ValueError("Mission points must be integer local north/east pairs")
    return config


def ordered_route(config, start_xy):
    """Return route points, required points and goal; shaping is opt-in."""
    required = [tuple(p) for p in config["required_checkpoints"]]
    if config["use_route_shaping"]:
        route = [tuple(p) for p in config["entry_points"]]
        for point, shaping in zip(required, config["shaping_after_checkpoint"]):
            route.append(point)
            route.extend(tuple(p) for p in shaping)
        route.extend(tuple(p) for p in config["cruise_points"])
    else:
        route = list(required)
    origin, goal = tuple(config["route_origin"]), tuple(config["final_destination"])
    distance = lambda p: math.hypot(start_xy[0] - p[0], start_xy[1] - p[1])
    if config["auto_reverse"] and distance(goal) < distance(origin):
        return route[::-1], required[::-1], origin
    return route, required, goal


def contains_in_order(path, checkpoints):
    """Check a subsequence; consecutive duplicate requests share one visit."""
    index = 0
    for point in checkpoints:
        while index < len(path) and path[index] != point:
            index += 1
        if index == len(path):
            return False
    return True
