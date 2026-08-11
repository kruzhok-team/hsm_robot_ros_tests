# The HSM ROS2 Testing Environment

The testing environment of the HSM-to-ROS2 framework. It covers the whole framework chain: a
CyberiadaML HSM diagram, the ROS2 node generated from it, the HSM Robot API nodes, the platform
driver and the robot simulator.

The tests of a single package live in that package. This project holds what cannot: the test
utilities shared by the tiers, the fake robot node, and the tests running several packages at
once.

The code is distributed under the GNU General Public License (version 3).

## Documentation

* The HSM Robot testing architecture - `docs/hsm_robot_testing.md`

## The Project Content

* `hsm_tests/test_utils` - the event probe, the topic recorders, the service probes, the
  simulated clock and the pytest fixtures;
* `hsm_tests/fake_robot` - the deterministic simulation backend;
* `test/integration` - the integration tests (the L3 tier);
* `test/system` - the system tests (the L4 tier).

These four directories are the placeholders of the current version: the testing architecture is
defined, the code is added step by step.

## Requirements

* ROS 2 Jazzy
* The API interfaces - https://github.com/kruzhok-team/hsm-robot-ros-interfaces
* The HSM Robot API nodes - https://github.com/kruzhok-team/hsm_robot_ros_api
* The HSM-to-ROS2 code generator - https://github.com/kruzhok-team/hsm_robot_ros_generator
* The turtle driver and `ros-jazzy-turtlesim` - the system tests only

## Running the Tests

```
colcon test --packages-select hsm_tests
colcon test-result --verbose
```

See `docs/hsm_robot_testing.md` for the separate test tiers.
