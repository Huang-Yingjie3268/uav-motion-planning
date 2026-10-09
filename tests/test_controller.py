"""Callback behavior with SDK doubles; these tests do not certify flight."""

import importlib.util
import os
from pathlib import Path
import sys
import types

import numpy as np
import pytest


@pytest.fixture
def controller(monkeypatch):
    class FakeDrone:
        def __init__(self, connection):
            self.connection = connection
            self.local_position = np.zeros(3)
            self.local_velocity = np.zeros(3)
            self.global_position = np.zeros(3)
            self.global_home = np.zeros(3)
            self.armed = self.guided = True
            self.commands = []

        def register_callback(self, *args):
            pass

        def takeoff(self, altitude):
            self.commands.append(("takeoff", altitude))

        def cmd_position(self, *point):
            self.commands.append(("position", point))

        def land(self):
            self.commands.append(("land",))

        def disarm(self):
            self.commands.append(("disarm",))

        def release_control(self):
            self.commands.append(("release",))

        def stop(self):
            self.commands.append(("stop",))

        def set_home_position(self, lon, lat, alt):
            self.global_home = np.array([lon, lat, alt])

    modules = {name: types.ModuleType(name) for name in
               ["udacidrone", "udacidrone.connection", "udacidrone.messaging",
                "udacidrone.frame_utils", "msgpack"]}
    modules["udacidrone"].Drone = FakeDrone
    modules["udacidrone.connection"].MavlinkConnection = object
    modules["udacidrone.messaging"].MsgID = types.SimpleNamespace(LOCAL_POSITION=1, LOCAL_VELOCITY=2, STATE=3)
    modules["udacidrone.frame_utils"].global_to_local = lambda *args: np.zeros(3)
    modules["msgpack"].dumps = lambda points: b"mocked display"
    for name, module in modules.items():
        monkeypatch.setitem(sys.modules, name, module)
    # Load an isolated module so mocks cannot leak into integration imports.
    path = Path(__file__).resolve().parents[1] / "src" / "motion_planning.py"
    spec = importlib.util.spec_from_file_location("src._controller_under_test", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    drone = module.MotionPlanning(types.SimpleNamespace(_master=types.SimpleNamespace(write=lambda _: None)))
    return drone, module.States


def active_target(drone, states, target=(10, 0), more=True):
    drone.flight_state = states.WAYPOINT
    drone.target_position = np.array([*target, 5.0, 0.0])
    drone.local_position = np.array([0.0, 0.0, -5.0])
    drone.waypoints = [[20, 0, 5, 0]] if more else []


def test_planning_callback_cannot_takeoff_before_validation(controller):
    drone, states = controller
    drone.flight_state = states.PLANNING
    drone.waypoints = [[10, 0, 5, 0]]
    drone.state_callback()
    assert drone.commands == []
    drone.route_ready = True
    drone.state_callback()
    drone.state_callback()
    assert drone.commands == [("takeoff", 0.0)]


@pytest.mark.parametrize("error", [ValueError("bad checkpoint"), FileNotFoundError("missing map"),
                                  RuntimeError("unreachable"), AttributeError("SDK display failure")])
def test_planning_failure_disarms_before_takeoff(controller, monkeypatch, error):
    drone, states = controller
    def fail():
        raise error
    monkeypatch.setattr(drone, "_prepare_route", fail)
    drone.plan_path()
    assert drone.flight_state == states.DISARMING
    assert not drone.route_ready
    assert drone.commands == [("disarm",), ("release",)]


def test_required_overshoot_is_not_a_visit(controller):
    drone, states = controller
    active_target(drone, states)
    drone.required_order = [(10, 0)]
    drone.required_fixed_points = {(10, 0)}
    drone.local_position = np.array([14.0, 0.0, -5.0])
    drone.local_position_callback()
    assert drone.required_cursor == 0 and drone.commands == []
    drone.local_position[0] = 11.0
    drone.local_position_callback()
    assert drone.required_cursor == 1
    assert drone.checkpoint_visits[0]["horizontal_distance"] == 1.0


def test_final_required_point_uses_tighter_radius_and_low_speed(controller):
    drone, states = controller
    active_target(drone, states, more=False)
    drone.required_order = [(10, 0)]
    drone.required_fixed_points = {(10, 0)}
    assert drone.is_required_fixed_waypoint()
    drone.local_position[0] = 12.0
    drone.local_position_callback()
    assert drone.commands == []
    drone.local_position[0] = 10.0
    drone.local_velocity[0] = 2.0
    drone.local_position_callback()
    assert drone.commands == [] and drone.required_cursor == 1
    drone.local_velocity[0] = 0.0
    drone.local_position_callback()
    assert drone.commands == [("land",)]


def test_transit_projection_requires_lateral_bound(controller):
    drone, states = controller
    active_target(drone, states)
    drone.local_position = np.array([9.5, 20.0, -5.0])
    drone.local_position_callback()
    assert drone.commands == []
    drone.local_position[1] = 0.1
    drone.local_position_callback()
    assert drone.commands[0][0] == "position"


def test_required_altitude_and_duplicate_visit_tracking(controller):
    drone, states = controller
    active_target(drone, states)
    drone.required_order = [(10, 0), (10, 0)]
    drone.required_fixed_points = {(10, 0)}
    drone.local_position = np.array([10.0, 0.0, -3.0])
    drone.local_position_callback()
    assert drone.required_cursor == 0
    drone.local_position[2] = -5.0
    drone.local_position_callback()
    assert drone.required_cursor == 2 and len(drone.checkpoint_visits) == 2


def test_disarming_requires_both_flags_clear(controller, monkeypatch):
    drone, states = controller
    drone.flight_state = states.DISARMING
    drone.armed, drone.guided = False, True
    drone.state_callback()
    assert drone.in_mission
    drone.guided = False
    drone.state_callback()
    assert not drone.in_mission and drone.commands == [("stop",)]


def test_route_validation_and_display_before_takeoff(controller, monkeypatch):
    drone, states = controller
    events = []
    def prepare():
        drone.state_callback()  # A state event while planning is still underway.
        assert drone.commands == []
        events.append("validated and displayed")
        drone.waypoints = [[10, 0, 5, 0]]
        drone.target_position[2] = 5
    monkeypatch.setattr(drone, "_prepare_route", prepare)
    drone.plan_path()
    assert events == ["validated and displayed"]
    assert drone.commands == [("takeoff", 5)]


@pytest.mark.skipif(not os.environ.get("UAV_COLLIDERS_CSV"), reason="External colliders.csv not configured for mocked full preflight")
@pytest.mark.parametrize("shaping,reverse", [(False, False), (False, True), (True, False), (True, True)])
def test_full_preflight_on_external_map_with_sdk_doubles(controller, monkeypatch, shaping, reverse):
    drone, states = controller
    drone.map_path = Path(os.environ["UAV_COLLIDERS_CSV"])
    drone.config["use_route_shaping"] = shaping
    start = np.array([155.0, -15.0, 0.0]) if reverse else np.zeros(3)
    monkeypatch.setitem(drone._prepare_route.__globals__, "global_to_local", lambda *args: start)
    drone.plan_path()
    assert drone.route_ready and drone.flight_state == states.TAKEOFF
    assert len(drone.waypoints) >= 4
    assert drone.commands == [("takeoff", 5)]
    expected = [(20, 30), (40, -5), (60, 30)]
    assert drone.required_order == (expected[::-1] if reverse else expected)
    assert drone.checkpoint_visits == []  # Route membership cannot fabricate a visit.
