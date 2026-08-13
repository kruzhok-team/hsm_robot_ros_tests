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
* How to write a test, for every tier - `docs/howto_write_a_test.md`

## The Project Content

The repository holds two ROS2 packages:

* `hsm_test_utils` - the probe and the pytest fixtures shared by the L2, L3 and L4 tiers. It
  is a package of its own so that the packages it helps test can depend on it without a cycle:
  it names nothing of the framework but the interfaces;
* `hsm_tests` - the suite:
  * `hsm_tests/fake_robot` - the deterministic simulation backend;
  * `hsm_tests/launch_support.py` - the launch descriptions of the two upper tiers;
  * `test/integration` - the integration tests (the L3 tier);
  * `test/system` - the system tests (the L4 tier).

## Requirements

* ROS 2 Jazzy
* The API interfaces - https://github.com/kruzhok-team/hsm-robot-ros-interfaces
* The HSM Robot API nodes - https://github.com/kruzhok-team/hsm_robot_ros_api
* The HSM-to-ROS2 code generator - https://github.com/kruzhok-team/hsm_robot_ros_generator
* The turtle driver and `ros-jazzy-turtlesim` - the system tests only

## Running the Tests

```
colcon test --packages-select hsm_tests hsm_test_utils
colcon test-result --verbose
```

One tier at a time, by its marker:

```
colcon test --packages-select hsm_tests --pytest-args -m integration    # L3
colcon test --packages-select hsm_tests --pytest-args -m system         # L4
```

See `docs/hsm_robot_testing.md` for the tiers of the whole framework, and
`docs/howto_write_a_test.md` for how to add a test to any of them.
