# ROS 2 Gait Controller — CI & Testing Demo

This repository demonstrates a practical testing and CI approach for a ROS 2 robot controller.

The project is based on a real hydraulic climbing robot control stack. Because the physical actuators can only be exercised realistically underwater, the CI environment replaces the physical actuator layer with a deterministic ROS 2 cylinder simulator.

The goal is to test as much of the real control software as possible without requiring physical hardware.

## Architecture

```text
Foxglove / ROS 2 action client
            |
            v
gait_sequencer_time_action_node
            |
            v
     gait_main_node
            |
            v
    /arduino_command
            |
            v
   cylinder_simulator
       |    |    |
       v    v    v
    /cu_pos /cp_pos /cl_pos
       \    |    /
        \   |   /
         gait_main_node
```

The simulator implements the same ROS topic interface used by the controller's actuator layer.

This allows the real gait-control logic to run unchanged while the physical actuator boundary is replaced during CI.

## Why a deterministic actuator simulator?

The real robot uses hydraulic cylinders and physical hardware that cannot be meaningfully exercised in a normal CI runner.

Rather than replacing the gait controller with a separate mock implementation, the tests keep the actual ROS 2 controller and substitute only the hardware boundary.

This provides a useful middle ground between:

* unit tests of individual functions
* hardware-dependent tests
* full simulation

The simulator is deterministic, fast, and suitable for automated testing.

## Test levels

### Unit tests

Pure Python tests verify gait target construction and state/direction mappings.

Example:

```bash
python3 -m pytest -q \
  src/gait_controller/test/test_gait_targets.py
```

### ROS 2 integration test

The integration test starts:

* `cylinder_simulator`
* `gait_main_node`

It then sends a real `MoveCylinder` action and verifies that:

1. the controller accepts the command
2. the controller commands the simulated actuator
3. the simulator updates the cylinder position
4. the controller observes the position
5. the target is reached
6. the controller explicitly stops the actuator

The test also verifies explicit stop behaviour.

There is intentionally **no artificial movement timeout** in the simulator. A cylinder continues moving until the controller explicitly commands `stop`.

### Gait sequence system test

A higher-level test starts:

* `cylinder_simulator`
* `gait_main_node`
* `gait_sequencer_time_action_node`

It sends the `start` gait action and verifies the complete sequence.

For example:

```text
cu in
cu stop
cp in
cp stop
cl in
cl stop
```

This exercises communication between multiple ROS 2 nodes and the actuator simulation.

## Hardware boundary

The real `gait_main_node` supports the physical Modbus interface.

Production:

```text
use_modbus:=true
```

CI:

```text
use_modbus:=false
```

When Modbus is disabled, the ROS actuator interface remains active and communicates with the deterministic cylinder simulator.

This means the CI tests exercise the same controller/action/topic path used by the real system rather than testing a separate CI-only implementation of the gait logic.

## Running locally

Source ROS 2 Humble:

```bash
source /opt/ros/humble/setup.bash
```

Build:

```bash
colcon build
```

Source the workspace:

```bash
source install/setup.bash
```

Run the unit tests:

```bash
python3 -m pytest -q \
  src/gait_controller/test/test_gait_targets.py
```

Run the gait controller integration test:

```bash
launch_test \
  src/gait_controller/test/gait_main_integration.test.py
```

Run the gait sequence system test:

```bash
launch_test \
  src/gait_controller/test/gait_sequencer_integration.test.py
```

Run the ROS package tests:

```bash
colcon test
colcon test-result --verbose
```

## Continuous Integration

GitHub Actions runs the same build and test workflow on every push and pull request.

The CI pipeline:

1. checks out the repository
2. installs ROS 2 Humble dependencies
3. builds the complete workspace
4. runs the Python unit tests
5. runs the ROS 2 gait-controller integration test
6. runs the complete gait-sequence system test
7. runs the package test suite
8. publishes the test results in the GitHub Actions log

The CI environment does not require physical robot hardware.

## Repository structure

```text
.
├── .github/
│   └── workflows/
│       └── ci.yml
├── src/
│   ├── cylinder_simulator/
│   │   └── deterministic actuator simulation
│   ├── gait_controller/
│   │   ├── gait_main_node.py
│   │   ├── gait_sequencer_time_action_node.py
│   │   ├── gait_targets.py
│   │   └── test/
│   │       ├── test_gait_targets.py
│   │       ├── gait_main_integration.test.py
│   │       └── gait_sequencer_integration.test.py
│   └── gait_controller_msgs/
│       └── MoveCylinder.action
├── .gitignore
└── README.md
```

## What this demonstrates

This project demonstrates a practical approach to robotics software testing:

* isolating hardware boundaries
* deterministic actuator simulation
* unit testing
* ROS 2 action testing
* multi-node integration testing
* system-level testing
* reproducible ROS 2 builds
* automated GitHub Actions CI
* testing real controller logic without physical hardware