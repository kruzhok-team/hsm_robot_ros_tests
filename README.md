# The HSM ROS2 Testing Environment

This is the project with the testing environment of the HSM-to-ROS2 framework. The environment
covers the whole framework chain: a CyberiadaML HSM diagram, the ROS2 node generated from it,
the HSM Robot API nodes, the platform driver and the robot simulator.

The tests of a single package live in that package and are run by `colcon test` together with
it. This project holds what cannot belong to a single package: the test utilities shared by all
the tiers, the deterministic simulation backend (the fake robot node), and the integration and
system tests running several packages at once.

The code is distributed under the GNU General Public License (version 3).

## Documentation

The documentation is located in the `docs` directory and contains:

* The HSM Robot testing architecture - `hsm_robot_testing.md`

## The Project Content

* `hsm_tests/test_utils` - the test utilities: the HSM event probe, the topic recorders, the
  service probes, the simulated clock and the pytest fixtures;
* `hsm_tests/fake_robot` - the fake robot node, the deterministic simulation backend;
* `test/integration` - the integration tests (the L3 tier);
* `test/system` - the system tests (the L4 tier).

The `hsm_tests/test_utils`, `hsm_tests/fake_robot`, `test/integration` and `test/system`
directories are the placeholders of the current version: the testing architecture is defined,
the code is added step by step.

## Requirements

* ROS 2 Jazzy
* The API interfaces - https://github.com/kruzhok-team/hsm-robot-ros-interfaces
* The HSM Robot API nodes - https://github.com/kruzhok-team/hsm_robot_ros_api
* The HSM-to-ROS2 code generator - https://github.com/kruzhok-team/hsm_robot_ros_generator
* The turtle driver and `ros-jazzy-turtlesim` - required by the system tests only

## Running the Tests

```
colcon test --packages-select hsm_tests
colcon test-result --verbose
```

See `docs/hsm_robot_testing.md` for the separate test tiers and for the offscreen mode of the
turtlesim simulator.
