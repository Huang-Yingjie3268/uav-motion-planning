# Optional simulator setup and current verification boundary

The standalone installation does not install `udacidrone` or `msgpack`. Keep the legacy simulator environment separate from the modern plotting/testing environment.

## Sources inspected

- [Official UdaciDrone getting started](https://udacity.github.io/udacidrone/docs/getting-started.html)
- [SDK setup.py](https://github.com/udacity/udacidrone/blob/master/setup.py)
- [FCND Motion Planning starter and simulator instructions](https://github.com/udacity/FCND-Motion-Planning)
- [Official simulator releases](https://github.com/udacity/FCND-Simulator-Releases/releases)

The inspected SDK setup identifies version 0.3.5 and a Python 3.6 classifier, with old pins for future, lxml, pymavlink, utm and websockets. Its dependency constraints should not be assumed compatible with a current Python interpreter. `msgpack` is used directly by the starter display code and must be available separately.

## Existing local environment evidence

The supplied project directory contains a historical setup note and a dependency list reporting Python 3.6.15, NumPy 1.19.5, pymavlink 2.4.43, msgpack 0.5.6 and udacidrone 0.3.5 installed from local source with `--no-deps`. These are historical observations, not a newly verified portable installation recipe. The dependency list contains an editable source path inconsistent with the setup note, whose consistency remains unresolved. It is not copied into this repository as a reusable lock file.

Python 3.6 syntax compilation of all three src modules succeeded with `PYTHONHASHSEED=0`. Actual import in that environment failed during NumPy random initialization with a Windows provider DLL error. A prior attempt also failed during Python random initialization. No controller import success, MAVLink connection or live flight is claimed. Do not bypass NumPy random initialization with fake data to make the import appear verified.

## Procedure after validating the SDK environment

1. Obtain an authorized map and compatible Udacity Motion Planning simulator from the official course resources.
2. Use the known working course environment, or resolve the old SDK pins in a dedicated environment. The official SDK is installed from source; review its requirements before using its documented source-install command. This project does not supply a newly validated SDK lock.
3. Confirm imports in that environment before connecting:

   ```bash
   python -c "import numpy, msgpack, udacidrone; from src.motion_planning import MotionPlanning; print('imports OK')"
   ```

4. Start the simulator and enter Motion Planning. The legacy connection constructor may retry indefinitely if the simulator is unavailable.
5. From this repository root, run:

   ```bash
   python -m src.motion_planning --map /path/to/colliders.csv --config config/mission.json --host 127.0.0.1 --port 5760
   ```

6. Supervise the flight. Check physical/simulator movement against the planned route, required-point acceptance radii, altitude, final arrival and low-speed landing condition. Collect NavLog plus checkpoint_visits.json; both are ignored by Git.

For an opt-in connectivity test in an environment with pytest and the actual SDK, set `UAV_SIMULATOR_CONNECTIVITY=1` and run the integration marker. That checks SDK import and TCP availability only; it does not validate flight.

## Configuration semantics

Default `use_route_shaping=false` plans only the required three points and destination. The optional shaping mode preserves entry, intermediate zigzag and cruise constraints from the coursework version. `auto_reverse=true` reverses order and returns to route_origin when starting closer to final_destination. Coordinates are integer local north/east meters; altitude commands are positive upward, while SDK local positions are NED.

The full pruned route, ordered membership and collision constraints are checked before readiness is set. The snapped start center is included as the first waypoint rather than silently skipping the quantization offset. Continuous clearance while moving from the measured start to that center remains an integration consideration.

Required checkpoint samples need the configured horizontal radius and altitude tolerance. The final waypoint uses the final radius, or the smaller of final and required radii when applicable, and requires horizontal speed below 1 m/s. Overshoot does not certify arrival. Configured proximity is not equivalent to passing exactly through the coordinate.
