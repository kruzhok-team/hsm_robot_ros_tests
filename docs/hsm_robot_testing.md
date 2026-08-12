# The HSM-to-ROS2 Testing Architecture

The framework converts a CyberiadaML diagram into a ROS2 node and runs it against a robot. A
change anywhere in that chain can break the robot behaviour without breaking anything a compiler
or a linter notices.

The principle: **the framework must be verifiable without a human watching the simulator
window.**

## 1. The Layers

```
  L4  SYSTEM                                       real time, offscreen simulator
      .graphml -> hsm_generator -> colcon build -> ros2 launch
      +---------------+   +----------+   +--------------+   +-----------+
      | generated     |-->| HSM Robot|-->| turtle_driver|-->| turtlesim |
      | HSM controller|<--| API nodes|<--|  (the driver)|<--| (offscr.) |
      +---------------+   +----------+   +--------------+   +-----------+

  L3  INTEGRATION                                  shortened periods, no display
      +---------------+   +----------+   +--------------------------+
      | generated     |-->| HSM Robot|-->|      the fake robot      |
      | HSM controller|<--| API nodes|<--| /odom, /scan <- scripted |
      +---------------+   +----------+   +--------------------------+

  L2  NODE                                         a single node and a probe
      +-----------+        +------------------------+
      | the node  | <----> | the event probe, the   |
      | under test|        | recorders, the probes  |
      +-----------+        +------------------------+

  L1  UNIT       the code generator                 no robot, no simulator
  L0  STATIC     the coding style                   no ROS2 runtime
```

| | checks | lives in | rule |
| --- | --- | --- | --- |
| L0 | `ament_flake8`, `ament_pep257` | every package | - |
| L1 | the generated code, the state machine behaviour, the diagram validation | `hsm_robot_ros_generator/test` | no robot, no simulator |
| L2 | the ROS2 contract of one node | `hsm_robot_ros_api/test`, `turtle_driver/test` | never starts a second framework node |
| L3 | the HSM state sequence, the API calls, the events | `hsm_robot_ros_tests/hsm_tests/test/integration` | a domain of its own, no display |
| L4 | the trajectory, the event order, the final position | `hsm_robot_ros_tests/test/system` | real time, a smoke set only |

A test lives with the code it tests, so a package stays testable on its own. Only the tests
needing several packages at once live in the testing project.

L1 is not free of ROS2 entirely: `hsm_controller/constants.py` imports `hsm_interfaces.msg`, so a
workspace with the built interfaces package must be sourced.

## 2. The Two Simulation Backends

The turtlesim simulator is what the framework ships with, but it is a poor foundation for the
bulk of the tests: it publishes no `/scan`, so the obstacle detection of the navigation module
(`COLLISION_WARNING`, `COLLISION_DETECTED`, `RIGHT_OPEN_SPACE`) is unreachable; it runs in real
time; and it is a Qt application. The fake robot node answers all three.

| | the fake robot (L3) | turtlesim (L4) |
| --- | --- | --- |
| The robot movement | yes | yes |
| The laser scan and the obstacle events | yes | no |
| The time | shortened by the parameters | real |
| The display | not needed | the offscreen mode |
| The number of tests | the bulk | a few smoke tests |
| What it proves | the framework logic is correct | the shipped system runs |

## 3. The Fake Robot Contract

```
                        +---------------------------+
     /cmd_vel     ----> |                           | ----> /odom
     (the velocity)     |       the fake robot      |       (the position)
                        |                           |
     /goal_pose   ----> |   the unicycle model on   | ----> /scan
     (the target)       |     a unicycle model      |       (the laser scan)
                        |                           |
     the scan     ----> |                           |
     script             +---------------------------+
     the teleport ----> 
```

* `/cmd_vel` (`geometry_msgs/Twist`) - integrated by a simple unicycle model;
* `/goal_pose` (`geometry_msgs/PoseStamped`) - the navigation target, including the cancellation
  protocol (the empty pose with the special frame identifier);
* `/odom` (`nav_msgs/Odometry`) - the resulting position and velocity;
* `/scan` (`sensor_msgs/LaserScan`) - from a static obstacle layout, or injected by the test.

Two objects exist for the tests only: the **scan script**, which sets the exact ranges published,
so an obstacle can be placed precisely at the threshold the navigation module compares against;
and the **teleport**, which resets the position between cases without restarting the node.

The rule: **the fake robot implements the same contract as the turtle driver and adds nothing to
it.** A test reaches the robot only through the ROS2 objects of the robot API, so the same
behaviour can be checked on both backends. The scan script and the teleport are the setup of a
test, never its subject.

## 4. The Time

Tested against the wall clock as it ships, the tick timers of the API modules would make the
suite slow: a diagram waiting for the one minute tick would cost a minute. The periods are
parameters of the nodes, so every tier shortens them instead:

```
    tick_period 0.02   second_period 0.05   minute_period 0.1
    control_period 0.02   scan_period 0.05
```

The minute tick is then checked in a tenth of a second, and the node and the integration tiers
together finish in under a minute of real time.

The simulated time of ROS2 was measured for this and not taken: it costs more than it buys here.
The deadlines a test waits on run off the node clock and freeze with it; the `/clock` topic is
best effort and one deep, so a node which starts late misses it; and a single step of a minute
fires a periodic timer once rather than sixty times, so a minute has to be stepped six hundred
times. The parameters give the same determinism for none of that.

Each launch description of the integration tier runs in a ROS2 domain of its own. The
descriptions of a tier run one after another in the same session, and the nodes of the previous
one are not always gone when the next starts - two robots publishing the odometry make the events
of the modules meaningless. It also keeps a graph the developer is running out of the tests.

## 5. The Test Utilities

`hsm_test_utils` is the shared vocabulary of the L2 and L3 tests, so that a test states
what it checks rather than how it spins an executor:

* **the event probe** - subscribes `/hsm_ros_msg` and `/hsm_ros_str_msg`, waits for a message
  code, reads a string argument, checks an event sequence. The main instrument of L2 and L3: the
  events are the only thing the API modules report;
* **the topic recorders** - the same for an arbitrary topic, to check what a node publishes;
* **the service probes** - fake service servers recording the requests, so the caller objects can
  be tested with no module node running;
* **the context** - the ROS2 context, the executor and the isolated domain around a test, and
  the isolation of the `Storage` directory, so the tests never write into the storage of the
  developer.

## 6. Testing the Code Generator

Three independent properties, because each fails differently:

* **the generated code** is compared with the reference output stored in the package. This
  catches every unintended change of the templates and of the conversion, including the changes
  that are still valid Python. The reference files are regenerated by a documented command and
  the difference is reviewed as part of the patch;
* **the state machine behaviour** is checked by importing the generated module with the API
  callers replaced by probes and dispatching events into the state machine. This checks what the
  reference comparison cannot: that the transitions of the diagram, including those of the nested
  composite states, are the transitions actually performed;
* **the diagram validation** is checked with deliberately invalid diagrams, asserting both the
  exception and the exit code of the command, which differ per error class. The diagrams cover
  the missing and duplicated initial pseudostate, the unknown event and module name, the
  duplicated and malformed state name, and the transition without a trigger.

The tests always generate into a temporary directory: the generator writes a whole package into
the directory given by `-o`.

## 7. Running the Tests

The environment is headless - the system tier sets the offscreen Qt platform itself, so no test
needs a display or depends on the shell of the developer.

```
colcon test && colcon test-result --verbose        # everything

colcon test --pytest-args -m linter                                        # L0
colcon test --packages-select hsm_generator --pytest-args -m "not linter"  # L1
colcon test --packages-select hsm_robot turtle_driver \
            --pytest-args -m "not linter"                                  # L2
colcon test --packages-select hsm_tests --pytest-args -m integration       # L3
colcon test --packages-select hsm_tests --pytest-args -m system            # L4
```

The environment is also built as a container image - the reference environment of the framework
and the one the continuous integration uses. Its package list is described in
`docs/hsm-ros2-testing-environment.md`.

## 8. The Known Gaps

* **The laser scan events are tested on the fake robot only.** turtlesim cannot publish the scan,
  so on a real robot platform that branch is exercised for the first time on the platform itself.
* **The system tier checks the robot behaviour coarsely** - the declared points in the declared
  order, not the precise trajectory between them.
