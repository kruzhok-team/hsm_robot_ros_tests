# The HSM-to-ROS2 Testing Architecture

The HSM-to-ROS2 framework converts a CyberiadaML HSM diagram into a ROS2 node and runs it
against a robot or a simulator. A change anywhere in that chain - in the diagram serialization
library, in the code generator, in the robot API, in the platform driver - can break the robot
behaviour without breaking anything that a compiler or a linter is able to notice. The testing
environment exists to make the whole chain checkable automatically.

The principle of the environment: **the framework must be verifiable without a human watching
the simulator window**. Every property the framework promises - the generator produces the
declared state machine, the API method reaches the robot, the event returns to the diagram, the
robot arrives at the point - is checked by a test that reports a result and needs no display.

## 1. The Testing Layers

The chain is tested on five layers. Each layer tests what the layer below cannot, and refuses
the dependencies of the layer above: the lower the layer, the faster and the more precise the
test, the higher the layer, the closer to what the framework user actually runs.

```
  L4  SYSTEM                                       real time, offscreen simulator
      .graphml -> hsm_generator.py -> colcon build -> ros2 launch
      +---------------+   +----------+   +--------------+   +-----------+
      | generated     |-->| HSM Robot|-->| turtle_driver|-->| turtlesim |
      | HSM controller|<--| API nodes|<--|  (the driver)|<--| (offscr.) |
      +---------------+   +----------+   +--------------+   +-----------+
                     checks: the trajectory, the event order, the final position

  L3  INTEGRATION                                  simulated time, no display
      +---------------+   +----------+   +--------------------------+
      | generated     |-->| HSM Robot|-->|      the fake robot      |
      | HSM controller|<--| API nodes|<--| /odom, /scan <- scripted |
      +---------------+   +----------+   +--------------------------+
                     checks: the HSM state sequence, the API calls, the events

  L2  NODE                                         a single node and a probe
      +-----------+        +------------------------+
      | the node  | <----> | the event probe, the   |
      | under test|        | recorders, the service |
      +-----------+        | probes                 |
                           +------------------------+
                     checks: the ROS2 contract of the node

  L1  UNIT                                         no robot, no simulator
      the code generator: the generated code, the state machine behaviour,
      the diagram validation errors

  L0  STATIC                                       no ROS2 runtime
      the coding style: ament_flake8, ament_pep257
```

### L0 - the static layer

The `ament_flake8` and `ament_pep257` checks of every package, configured to the coding style
of the framework rather than to the ament defaults. The layer runs in every package and costs
nothing, so it gates everything else.

### L1 - the unit layer

The layer tests the code generator, which is the only substantial piece of the framework that is
pure computation: a diagram file goes in, a Python module comes out. It lives in the
`hsm_robot_ros_generator` package and needs no robot, no simulator and no running ROS2 graph.
See section 7.

The layer is not free of ROS2 entirely: `hsm_controller/constants.py` imports `hsm_interfaces.msg`
to name the message codes, so the tests need a workspace with the built interfaces package
sourced. That is the only ROS2 dependency of the layer.

### L2 - the node layer

The layer tests one ROS2 node at a time through its declared interface only - the services it
advertises, the topics it subscribes to and the topics it publishes. The node is the real node,
started in the test process; everything around it is a probe.

The layer lives in the package that owns the node: the API module nodes are tested in
`hsm_robot_ros_api`, the platform driver in `turtle_driver`. A node test must not start any
other node of the framework: if a check needs two nodes, it belongs to L3.

This is where the exact contract is pinned - the topic a service writes to, the sign of the
velocity it publishes, the message code of the event it emits, the conditions under which the
event is emitted once rather than repeatedly.

### L3 - the integration layer

The layer tests the framework as a whole with the simulator replaced by the fake robot node. The
generated HSM controller, the API module nodes and the fake robot run together; the test drives
the robot state and reads back the state of the HSM.

The layer answers the questions no single node can: does an API method called from the diagram
reach the module node, does the event return to the state machine, does the state machine move
to the state the diagram declares. It is also the only layer able to test the laser scan events
(section 3).

The layer runs on the simulated time (section 5), so it is fast and repeatable.

### L4 - the system layer

The layer runs what the framework user runs: the code generator on a diagram file, the build of
the generated package, and the launch of the whole system with the turtlesim simulator. It runs
in real time and is therefore deliberately small - a smoke test set, not a coverage set.

Its purpose is not to find fine bugs but to prove that the documented workflow works: that the
generator output builds, that the launch files bring the system up, and that the robot in a real
simulator does what the diagram says.

## 2. Where the Tests Live

```
  hsm_robot_ros_generator/test/     L0, L1   the generator, the diagrams, the errors
  hsm_robot_ros_interfaces/         L0       the lints of the interfaces package
  hsm_robot_ros_api/test/           L0, L2   the API module nodes
  turtle_driver/test/               L0, L2   the platform driver node
  hsm_robot_ros_tests/              L0, L3, L4, the utilities and the fake robot
    hsm_tests/test_utils/                    the probes, the recorders, the clock
    hsm_tests/fake_robot/                    the deterministic simulation backend
    test/integration/                        the L3 tier
    test/system/                             the L4 tier
```

The rule is that a test lives with the code it tests, so a package remains testable on its own
and its patch remains self-contained. The tests that need several packages at once cannot follow
that rule and live in this project instead.

The tiers of this project are selected by the pytest markers declared in `setup.cfg`:

```
colcon test --packages-select hsm_tests --pytest-args -m integration
colcon test --packages-select hsm_tests --pytest-args -m system
```

## 3. The Two Simulation Backends

The framework is tested against two different simulation backends, because neither of them alone
is sufficient.

**The turtlesim simulator** is what the framework actually ships with today, driven through the
`turtle_driver` node. It is realistic - it is the real simulator, the real driver and the real
topics - but it has three properties that make it a poor foundation for the bulk of the tests:

* it does not publish the laser scan. There is no `/scan` topic anywhere in the turtlesim
  system, so the whole obstacle detection branch of the navigation module - the
  `COLLISION_WARNING`, `COLLISION_DETECTED` and `RIGHT_OPEN_SPACE` events - is unreachable. The
  turtle driver compensates in the robot behaviour by reporting a collision when the distance to
  the goal stops decreasing, but that is a different code path in a different node: it does not
  test the navigation module at all;
* it runs in real time. A test of the one minute timer tick costs a minute;
* it is a Qt application. It needs a display, or at least the offscreen platform plugin, and it
  is a heavy process to start and stop between test cases.

**The fake robot node** is the answer to those three properties. It is a plain Python ROS2 node
implementing the same contract as the turtle driver, with no graphics and no wall clock. It is
not a better simulator - it is a simpler one, built to be driven by a test rather than watched by
a human.

The division of labour:

| | the fake robot (L3) | turtlesim (L4) |
| --- | --- | --- |
| The robot movement | yes | yes |
| The laser scan and the obstacle events | yes | no |
| The time | simulated, controlled by the test | real |
| The display | not needed | the offscreen mode |
| The number of tests | the bulk | a few smoke tests |
| What it proves | the framework logic is correct | the shipped system runs |

Both backends are driven through the same ROS2 objects, so a test written against one describes
the same robot API as a test written against the other.

## 4. The Fake Robot Contract

The fake robot node replaces both the platform driver and the simulator. It implements the ROS2
objects the HSM Robot API uses (see `hsm_robot_architecture.md`, "The ROS2 Incapsulation"):

```
                        +---------------------------+
     /cmd_vel     ----> |                           | ----> /odom
     (the velocity)     |       the fake robot      |       (the position)
                        |                           |
     /goal_pose   ----> |   the unicycle model on   | ----> /scan
     (the target)       |     the simulated time    |       (the laser scan)
                        |                           |
     the scan     ----> |                           |
     script             +---------------------------+
     the teleport ----> 
```

* `/cmd_vel` (`geometry_msgs/Twist`) - the velocity command, integrated by a simple unicycle
  model;
* `/goal_pose` (`geometry_msgs/PoseStamped`) - the navigation target, including the cancellation
  protocol of the navigation module (the empty pose with the special frame identifier);
* `/odom` (`nav_msgs/Odometry`) - the resulting robot position and velocity;
* `/scan` (`sensor_msgs/LaserScan`) - the laser scan, either generated from a static obstacle
  layout given by the node parameters, or injected directly by the test.

Two objects exist for the tests only and have no counterpart in a real robot:

* the scan script - the test sets the exact ranges the node publishes, so a test can place the
  obstacle precisely at the threshold the navigation module compares against, and check the
  `COLLISION_WARNING`, `COLLISION_DETECTED` and `RIGHT_OPEN_SPACE` events one by one;
* the teleport - the test resets the robot position between the cases without restarting the
  node.

The important rule: **the fake robot implements the same contract as the turtle driver and adds
nothing to it**. A test must never reach the robot through anything but the ROS2 objects of the
robot API, so that the same behaviour can be checked on both backends. The scan script and the
teleport are the setup of the test, not the subject of it.

## 5. The Time and the Determinism

The API module nodes contain the timers: the tick timer, the one second tick timer, the one
minute tick timer and the timers created by the `Timer` module on request. Tested against the
wall clock they make the test suite slow, and worse, flaky: a test that waits for an event with a
timeout either wastes the timeout or fails on a loaded machine.

The integration tier therefore runs on the ROS2 simulated time. Every node of the tier is started
with the `use_sim_time` parameter set, so the node clocks follow the `/clock` topic instead of
the wall clock, and the test harness publishes that topic itself. The test does not wait for the
time to pass - it moves the time forward and then lets the nodes process what that produced:

```
    the test                        the nodes
    --------                        ---------
    clock.advance(60.0)     --->    /clock
                                    the one minute tick timer fires
    probe.wait_for(TICK_1M) <---    /hsm_ros_msg
```

A one minute tick is checked in milliseconds, and it is checked at exactly the same simulated
moment on every run.

The system tier cannot do this: the turtlesim simulator has no simulated time support. That tier
runs on the wall clock, bounds every test with a timeout, and keeps the checked timings coarse.

## 6. The Test Utilities

The utilities of `hsm_tests/test_utils` are the shared vocabulary of the L2 and L3 tests. They
exist so that a test states what it checks rather than how it spins an executor.

* **the event probe** - a node subscribed to the HSM message topics `/hsm_ros_msg` and
  `/hsm_ros_str_msg`. It records the events of the framework and lets a test wait for a message
  code, read the string argument of a message, and check that a sequence of events arrived in the
  declared order. It is the main instrument of the L2 and L3 tiers, because the events are the
  only thing the API modules report;
* **the topic recorders** - the same for an arbitrary topic and message type, used to check what
  a node publishes: the velocity on `/cmd_vel`, the target on `/goal_pose`, the pump state on
  `/pump`;
* **the service probes** - the fake service servers advertising the API services and recording
  the requests. They let the caller objects of the generated controller be tested without any
  module node running, and they let a test check the arguments an API method actually sent;
* **the simulated clock** - the `/clock` publisher of section 5, able to move the simulated time
  forward and to run the nodes until a condition holds;
* **the fixtures** - the pytest fixtures creating and destroying the ROS2 context and the
  executor around a test, and the fixture isolating the long-term storage of the `Storage`
  module, so that the tests never write into the storage directory of the developer.

## 7. Testing the Code Generator

The generator is tested on three independent properties, because each of them fails differently.

**The generated code** is compared with the reference output stored in the package. Every example
diagram is converted into a temporary directory and the result is compared with the stored file.
This catches every unintended change of the templates and of the conversion, including the
changes that are still valid Python. The reference files are regenerated by a documented command
when the change is intended, and the difference is then reviewed as a part of the patch.

The generator must always be run into a temporary directory in the tests. Its default output
directory is the current one, and the generator project is itself the generated package, so a
test generating into the current directory would overwrite the sources.

**The behaviour of the generated state machine** is checked by importing the generated module,
replacing the API caller objects with the probes and dispatching the events into the state
machine directly. The state machine is then asked for its current state. This checks what the
reference comparison cannot: that the transitions of the diagram, including the transitions of
the nested composite states, are the transitions the state machine actually performs.

**The diagram validation** is checked by converting the deliberately invalid diagrams stored in
the generator package and asserting the error the generator reports. The generator distinguishes
the parsing errors, the generating errors and the conversion errors, and returns a different exit
code for each, so both the exception and the exit code of the command are checked. The invalid
diagrams cover the cases the generator rejects: the missing and the duplicated initial
pseudostate, the unknown event name, the unknown module name, the duplicated and the malformed
state name, and the transition without a trigger.

## 8. Running the Tests

The whole environment is headless. The system tier sets the offscreen mode of the Qt platform
itself, so no test requires a display and no test depends on the shell of the developer.

```
# everything, in every package
colcon test
colcon test-result --verbose

# the static layer only
colcon test --pytest-args -m linter

# the generator, the unit layer
colcon test --packages-select hsm_generator --pytest-args -m "not linter"

# the node layer
colcon test --packages-select hsm_robot turtle_driver --pytest-args -m "not linter"

# the integration and the system tiers
colcon test --packages-select hsm_tests --pytest-args -m integration
colcon test --packages-select hsm_tests --pytest-args -m system
```

The environment is also built as a container image, so that the test result does not depend on
what happens to be installed on the machine. The image is the reference environment of the
framework and is the one the continuous integration uses. The package list it is built from is
described in the framework documentation, `docs/hsm-ros2-testing-environment.md`.

## 9. The Known Gaps

The gaps are listed rather than hidden, because a test suite that appears to cover everything is
worse than one whose limits are written down.

* **The laser scan events are tested on the fake robot only.** The turtlesim simulator cannot
  publish the scan, so the obstacle detection of the navigation module is never exercised in the
  system tier. On a real robot platform that branch is exercised for the first time on the
  platform itself.
* **The system tier checks the robot behaviour coarsely.** It runs in real time against a real
  simulator, so it checks that the robot visits the declared points in the declared order, not
  the precise trajectory between them.
