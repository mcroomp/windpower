# Stack Integration Tests

This directory contains the RAWES stack integration smoke test.

The goal is to validate the controller and transport path between ArduPilot SITL and the Python simulation bridge without requiring a live MBDyn process or Mission Planner.

## What This Test Covers

The smoke test launches:

- real ArduPilot SITL via `sim_vehicle.py`
- real `simulation/mediator.py`
- a fake plant that stands in for MBDyn over the UNIX socket protocol
- a fake ground station that connects over MAVLink and waits for telemetry

The test currently verifies that:

- SITL starts and exposes a MAVLink endpoint
- the mediator starts and exchanges packets with SITL
- the mediator connects to the fake plant over the force/state sockets
- at least two force packets are generated
- the latest force packet has the expected 6-element shape
- the force values are finite
- MAVLink heartbeat, attitude, and position messages are observed

## What This Test Does Not Cover

This is intentionally a stack integration test, not a full end-to-end simulation test.

It does not validate:

- a real MBDyn process
- real Mission Planner or MAVProxy usage
- full rotor or tether physics fidelity
- controller performance against a physical truth model
- flight-mode behavior such as takeoff, autonomous flight, or landing

## Test File

- `test_stack_integration.py`: end-to-end smoke test for SITL + mediator + fake plant + fake GCS

## Prerequisites

The test is opt-in and will skip unless the required environment is present.

Required environment variables:

- `RAWES_RUN_STACK_INTEGRATION=1`
- one of:
  - `RAWES_SIM_VEHICLE=/path/to/ardupilot/Tools/autotest/sim_vehicle.py`
  - `RAWES_ARDUPILOT_PATH=/path/to/ardupilot`

Additional requirements:

- `pymavlink` installed in the test environment
- ArduPilot SITL available
- an environment with `AF_UNIX` socket support

In practice, this repo is set up to run the stack integration test inside the Docker image rather than directly on Windows.

## Recommended Run Path

### Windows host

Build the container image with ArduPilot included:

```cmd
simulation\build.cmd ardupilot
```

Then run the stack integration smoke test inside Docker:

```cmd
simulation\run_stack_integration.cmd -s
```

The `-s` option is passed through to `pytest` so test output is shown directly.

### Linux or WSL

If the image already exists, run:

```bash
docker run --rm -it \
  -v "$(pwd)/simulation:/rawes/simulation" \
  rawes-sim \
  bash /rawes/simulation/test_stack.sh -s
```

The container-side script sets `RAWES_RUN_STACK_INTEGRATION=1` automatically and, when available, defaults `RAWES_SIM_VEHICLE` to `/ardupilot/Tools/autotest/sim_vehicle.py`.

## Running Pytest Directly

If you already have the right dependencies and environment variables in place, you can invoke the test directly:

```bash
python -m pytest simulation/tests/stack -s
```

## Expected Behavior

On success, the test should:

- connect to SITL over MAVLink TCP on `127.0.0.1:5760`
- receive a heartbeat
- receive `ATTITUDE` and `GLOBAL_POSITION_INT` messages
- record force packets from the mediator-to-plant socket path

The test process will terminate SITL and the mediator when it exits.

## Troubleshooting

### Test is skipped immediately

Common causes:

- `RAWES_RUN_STACK_INTEGRATION` is not set to `1`
- neither `RAWES_SIM_VEHICLE` nor `RAWES_ARDUPILOT_PATH` is set
- `pymavlink` is missing
- the platform does not support `AF_UNIX`

### `simulation\run_stack_integration.cmd` fails before pytest starts

Check that:

- Docker Desktop is installed and running
- the `rawes-sim` image exists
- the image was built with `simulation\build.cmd ardupilot`

### SITL starts but the test times out

Check that:

- `sim_vehicle.py` is present in the expected ArduPilot checkout
- port `5760` is not blocked or already in use
- the container includes the ArduPilot SITL build

## Maintenance Notes

If the mediator transport, socket paths, MAVLink endpoint, or SITL launch arguments change, update this README alongside `test_stack_integration.py`, `simulation/test_stack.sh`, and `simulation/run_stack_integration.cmd`.