"""Optional Udacity simulator controller, derived from the FCND starter.

See docs/contribution_verification.md for attribution and publication blockers.
Run from the repository root with python -m src.motion_planning --map <csv>.
"""

import argparse
from enum import Enum, auto
import json
from pathlib import Path
import time

import msgpack
import numpy as np
from udacidrone import Drone
from udacidrone.connection import MavlinkConnection
from udacidrone.messaging import MsgID
from udacidrone.frame_utils import global_to_local

from .mission_config import DEFAULT_CONFIG, contains_in_order, load_mission, ordered_route
from .planning_utils import (a_star_through_points, create_grid,
                             heuristic_octile, line_of_sight)


class States(Enum):
    MANUAL = auto()
    ARMING = auto()
    TAKEOFF = auto()
    WAYPOINT = auto()
    LANDING = auto()
    DISARMING = auto()
    PLANNING = auto()


class MotionPlanning(Drone):
    """Validate a full route before takeoff and track sampled checkpoint visits."""

    def __init__(self, connection, config_path=DEFAULT_CONFIG, map_path="colliders.csv"):
        super().__init__(connection)
        self.config = load_mission(config_path)
        self.map_path = Path(map_path)
        self.target_position = np.array([0.0, 0.0, 0.0, 0.0])
        self.waypoints = []
        self.in_mission = True
        self.route_ready = False
        self.turning_waypoints = set()
        self.route_checkpoints = set()
        self.required_fixed_points = set()
        self.required_order = []
        self.required_cursor = 0
        self.checkpoint_visits = []
        self.segment_start = np.array([0.0, 0.0])
        self.flight_state = States.MANUAL
        self.register_callback(MsgID.LOCAL_POSITION, self.local_position_callback)
        self.register_callback(MsgID.LOCAL_VELOCITY, self.velocity_callback)
        self.register_callback(MsgID.STATE, self.state_callback)

    def local_position_callback(self):
        if self.flight_state == States.TAKEOFF:
            if abs(-self.local_position[2] - self.config["target_altitude"]) <= self.config["altitude_acceptance_radius"]:
                self.waypoint_transition()
        elif self.flight_state == States.WAYPOINT:
            distance = np.linalg.norm(self.target_position[:2] - self.local_position[:2])
            altitude_ok = abs(-self.local_position[2] - self.target_position[2]) <= self.config["altitude_acceptance_radius"]
            final = not self.waypoints
            fixed = self.is_required_fixed_waypoint()
            if final:
                radius = self.config["final_acceptance_radius"]
                if fixed:
                    radius = min(radius, self.config["fixed_point_acceptance_radius"])
            elif fixed:
                radius = self.config["fixed_point_acceptance_radius"]
            elif self.is_route_checkpoint():
                radius = self.config["checkpoint_acceptance_radius"]
            elif self.is_transit_waypoint():
                radius = self.config["transit_acceptance_radius"]
            else:
                radius = self.config["turn_acceptance_radius"]
            flyby = (self.is_transit_waypoint() and not fixed
                     and self.target_progress() >= self.config["transit_flyby_progress"]
                     and self.cross_track_distance() <= self.config["transit_cross_track_limit"])
            # Overshoot alone never certifies a required checkpoint or final goal.
            if altitude_ok and (distance <= radius or flyby):
                self.record_checkpoint_visit(float(distance))
                if self.waypoints:
                    self.waypoint_transition()
                elif (self.required_cursor == len(self.required_order)
                      and np.linalg.norm(self.local_velocity[:2]) < 1.0):
                    self.landing_transition()

    def waypoint_key(self, position):
        return (int(round(position[0])), int(round(position[1])))

    def is_transit_waypoint(self):
        key = self.waypoint_key(self.target_position)
        return (bool(self.waypoints) and key not in self.turning_waypoints
                and key not in self.route_checkpoints
                and key not in self.required_fixed_points)

    def is_route_checkpoint(self):
        return self.waypoint_key(self.target_position) in self.route_checkpoints

    def is_required_fixed_waypoint(self):
        return self.waypoint_key(self.target_position) in self.required_fixed_points

    def record_checkpoint_visit(self, distance):
        """Record measured samples, distinct from planned route membership."""
        key = self.waypoint_key(self.target_position)
        while (self.required_cursor < len(self.required_order)
               and key == self.required_order[self.required_cursor]
               and distance <= self.config["fixed_point_acceptance_radius"]):
            self.checkpoint_visits.append({
                "checkpoint": key, "position_ned": self.local_position.tolist(),
                "horizontal_distance": distance, "time": time.time()})
            self.required_cursor += 1

    def passed_target(self):
        return self.target_progress() >= 1.0

    def target_progress(self):
        segment = self.target_position[:2] - self.segment_start
        length_squared = np.dot(segment, segment)
        if length_squared < 1.0e-6:
            return 0.0
        return np.dot(self.local_position[:2] - self.segment_start, segment) / length_squared

    def cross_track_distance(self):
        """Distance to the target segment, including endpoint overshoot."""
        progress = np.clip(self.target_progress(), 0.0, 1.0)
        nearest = self.segment_start + progress * (self.target_position[:2] - self.segment_start)
        return float(np.linalg.norm(self.local_position[:2] - nearest))

    def velocity_callback(self):
        if self.flight_state == States.LANDING:
            if (self.global_position[2] - self.global_home[2] < 0.1
                    and abs(self.local_position[2]) < 0.01):
                self.disarming_transition()

    def state_callback(self):
        if not self.in_mission:
            return
        if self.flight_state == States.MANUAL:
            self.arming_transition()
        elif self.flight_state == States.ARMING and self.armed and self.guided:
            self.plan_path()
        elif self.flight_state == States.PLANNING and self.route_ready:
            self.takeoff_transition()
        elif self.flight_state == States.DISARMING and not self.armed and not self.guided:
            self.manual_transition()

    def arming_transition(self):
        self.flight_state = States.ARMING
        self.arm()
        self.take_control()

    def takeoff_transition(self):
        if self.flight_state != States.PLANNING or not self.route_ready or not self.waypoints:
            return
        self.flight_state = States.TAKEOFF
        self.takeoff(self.target_position[2])

    def waypoint_transition(self):
        if not self.waypoints:
            self.landing_transition()
            return
        self.flight_state = States.WAYPOINT
        self.segment_start = self.local_position[:2].copy()
        self.target_position = np.asarray(self.waypoints.pop(0), dtype=float)
        self.cmd_position(*self.target_position)

    def landing_transition(self):
        self.flight_state = States.LANDING
        self.land()

    def disarming_transition(self):
        self.route_ready = False
        self.flight_state = States.DISARMING
        self.disarm()
        self.release_control()

    def manual_transition(self):
        self.in_mission = False
        self.flight_state = States.MANUAL
        self.stop()
        if self.checkpoint_visits:
            Path("Logs").mkdir(exist_ok=True)
            with open("Logs/checkpoint_visits.json", "w") as stream:
                json.dump(self.checkpoint_visits, stream, indent=2)

    def send_waypoints(self, waypoints=None):
        """Preserve the starter's private simulator display protocol."""
        points = self.waypoints if waypoints is None else waypoints
        self.connection._master.write(msgpack.dumps(points))

    def plan_path(self):
        self.flight_state = States.PLANNING
        self.route_ready = False
        try:
            self._prepare_route()
        except Exception as error:
            # SDK/display failures must also abort while still on the ground.
            print("Preflight planning failed: {}".format(error))
            self.waypoints = []
            self.disarming_transition()
            return
        self.route_ready = True
        self.takeoff_transition()

    def _prepare_route(self):
        self.target_position[2] = self.config["target_altitude"]
        with open(str(self.map_path)) as stream:
            lat_part, lon_part = stream.readline().split(",")
        lat0, lon0 = float(lat_part.split()[1]), float(lon_part.split()[1])
        if not np.isfinite([lat0, lon0]).all():
            raise ValueError("Nonfinite map origin")
        self.set_home_position(lon0, lat0, 0)
        current = global_to_local(self.global_position, self.global_home)
        if not np.isfinite(current).all():
            raise ValueError("Nonfinite measured position")
        data = np.loadtxt(str(self.map_path), delimiter=",", dtype=float, skiprows=2, ndmin=2)
        grid, north_offset, east_offset = create_grid(
            data, self.config["target_altitude"], self.config["safety_distance"])

        def to_grid(point):
            return (int(np.rint(point[0] - north_offset)),
                    int(np.rint(point[1] - east_offset)))

        route, required, goal = ordered_route(self.config, current[:2])
        start = to_grid(current)
        route_grid = [to_grid(p) for p in route]
        path, cost = a_star_through_points(grid, heuristic_octile, start, route_grid, to_grid(goal))
        if not path or not contains_in_order(path, route_grid):
            raise RuntimeError("No complete ordered route")
        if not all(line_of_sight(grid, a, b) for a, b in zip(path, path[1:])):
            raise RuntimeError("Unsafe route after pruning")
        self.required_order = required
        self.required_cursor = 0
        self.checkpoint_visits = []
        self.route_checkpoints = set(route)
        self.required_fixed_points = set(required)
        self.turning_waypoints = {(p[0] + north_offset, p[1] + east_offset) for p in path}
        # Include the snapped start center; this also handles a start checkpoint.
        self.waypoints = [[p[0] + north_offset, p[1] + east_offset,
                           self.config["target_altitude"], 0] for p in path]
        print("Software route validated; raw grid cost: {:.6f}".format(cost))
        self.send_waypoints()

    def start(self):
        self.start_log("Logs", "NavLog.txt")
        try:
            self.connection.start()
        finally:
            self.stop_log()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", type=int, default=5760)
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--map", required=True, type=Path)
    parser.add_argument("--config", default=DEFAULT_CONFIG, type=Path)
    args = parser.parse_args()
    load_mission(args.config)
    if not args.map.is_file():
        parser.error("Map file does not exist")
    connection = MavlinkConnection("tcp:{}:{}".format(args.host, args.port), timeout=60)
    drone = MotionPlanning(connection, args.config, args.map)
    time.sleep(1)
    drone.start()


if __name__ == "__main__":
    main()
