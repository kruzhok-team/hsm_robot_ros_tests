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

| | checks | lives in | marker | rule |
| --- | --- | --- | --- | --- |
| L0 | `ament_flake8`, `ament_pep257` | every package | `linter` | - |
| L1 | the generated code, the state machine behaviour, the diagram validation | `hsm_robot_ros_generator/test` | `unit` | no robot, no simulator |
| L2 | the ROS2 contract of one node | `hsm_robot_ros_api/test`, `turtle_driver/test` | `node` | never starts a second framework node |
| L3 | the HSM state sequence, the API calls, the events | `hsm_robot_ros_tests/hsm_tests/test/integration` | `integration` | a domain of its own, no display |
| L4 | the trajectory, the event order, the packaging | `hsm_robot_ros_tests/hsm_tests/test/system` | `system` | real time, a smoke set only |

Every tier is selected by its own marker, so a tier can be run alone; the commands are in §8.
How to add a test to any of them is in `howto_write_a_test.md`.

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

The recipe for a test which uses it is in `howto_write_a_test.md`.

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

## 4. The System Tier

The system tier runs what the framework ships and nothing else: the simulator, the driver which
adapts it to the API, the module nodes, and a controller generated from a diagram. Nothing
written for the tests stands between the diagram and the turtle.

```
    a diagram --> the generator --> a package --> colcon --> the controller
                                                                  |
                        the module nodes of the API <-------------+
                                    |
                            the turtle driver --> turtlesim (offscreen)
```

Three things belong to this tier and to no other:

* **the position the simulator reports.** The integration tier believes the odometry, and the
  odometry comes from the same fake robot which decides where the robot is. Here the position is
  computed by the simulator and the odometry of the driver is a copy of it, so `/turtle1/pose` is
  a ground truth an assertion can stand on;
* **the world has walls.** turtlesim is eleven units across and holds the turtle at the edge, so
  a diagram can look correct while its trajectory is nonsense. The tier asserts the turtle stays
  inside, which is the only check that says the speeds of the framework suit the world it drives
  in;
* **the packaging.** Every other tier runs the generated controller out of the directory it was
  generated into, because a generated package is always named `hsm_controller` and a workspace
  holds one of those at a time. A user instead generates into a workspace, builds it and runs the
  installed executable, and that half of the workflow is otherwise never exercised. One test
  pays for it - it builds, which costs a second and a half - and it starts the simulator through
  the launch file which ships with the driver rather than through a description written for the
  tests.

**The tier does not shorten anything but the startup handshake.** The integration tier can
shorten the clock because the clock of the fake robot is a parameter; the simulator moves the
turtle at the speed the driver asks for, and the only thing a parameter could change is that
speed - which is the behaviour the tier exists to check. Driving the turtle faster than the
framework ships also takes it out of the world: it is asked to turn no faster than it turns, so
a fast enough turtle leaves the square as an arc and is stopped by the wall. A square takes the
twenty seconds a square takes, and the whole tier costs about a minute.

## 5. The Time

Tested against the wall clock as it ships, the tick timers of the API modules would make the
suite slow: a diagram waiting for the one minute tick would cost a minute. The periods are
parameters of the nodes, so the node and integration tiers shorten them:

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

Each launch description of the integration and system tiers runs in a ROS2 domain of its own. The
descriptions of a tier run one after another in the same session, and the nodes of the previous
one are not always gone when the next starts - two robots publishing the odometry make the events
of the modules meaningless. It also keeps a graph the developer is running out of the tests, and
it matters more at the system tier, where the simulator owns the `turtle1` names of the whole
graph and a turtlesim left running by the developer would join a test silently.

## 6. The Test Utilities

`hsm_test_utils` is the shared vocabulary of the L2, L3 and L4 tests, so that a test states
what it checks rather than how it spins an executor:

* **the probe** (`probes.py`) - one node which subscribes `/hsm_ros_msg` and `/hsm_ros_str_msg`
  and waits for a message code, records an arbitrary topic, publishes into one, and calls a
  service. The main instrument of the three tiers: the events are the only thing the API modules
  report;
* **the context** (`context.py`) - the ROS2 context, the executor and the isolated domain around
  a test (`node_factory`, `isolated_domain`), the parameter overrides a node can only be given
  through `rclpy.init`, and the isolation of the `Storage` directory, so the tests never write
  into the storage of the developer.

The recipe for using them is §4 of `howto_write_a_test.md`.

## 7. Testing the Code Generator

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

## 8. Running the Tests

The environment is headless - the system tier sets the offscreen Qt platform itself, so no test
needs a display or depends on the shell of the developer. The launch file which ships with the
driver takes a `headless` argument for the same reason, so the simulator a user starts can run
without a display too:

```
ros2 launch turtle_driver start.launch.py headless:=true
```

```
colcon test && colcon test-result --verbose        # everything

colcon test --pytest-args -m linter                                        # L0
colcon test --packages-select hsm_generator --pytest-args -m unit          # L1
colcon test --packages-select hsm_robot turtle_driver --pytest-args -m node  # L2
colcon test --packages-select hsm_tests --pytest-args -m integration       # L3
colcon test --packages-select hsm_tests --pytest-args -m system            # L4
```

The environment is also built as a container image - the reference environment of the framework
and the one the continuous integration uses. Its package list is described in
`docs/hsm-ros2-testing-environment.md`.

## 9. The Known Gaps

* **The laser scan events are tested on the fake robot only.** turtlesim cannot publish the scan,
  so on a real robot platform that branch is exercised for the first time on the platform itself.
* **The system tier checks the robot behaviour coarsely** - the declared points in the declared
  order and the world the turtle stayed inside, not the precise trajectory between them.
* **One system test builds, the others do not.** The packaging is proven once; a break which only
  appears for a particular diagram would not be seen.
* **The system tier drives four diagrams**, the ones the framework supports today. The diagrams
  which need a module method that is not implemented yet cannot be driven by any tier.

## 10. The Diagram Test Runner

The L3 framework without pytest: `hsm_robot_test` runs one diagram against the fake robot
and checks the expectations the diagram itself declares. It is what the `test` action of the
CyberiadaPro plugin (`hsm_robot_ros_generator/docs/editor_plugin.md`) runs. The generated
controller runs on the HSM dispatcher `hsmd` (`python3-hsm-dispatcher`); the runner checks it
imports before it starts anything.

```
 hsm_robot_test diagram.graphml
   │ 1 generate the controller           launch_support.generate_controller, a tmp dir
   │ 2 start the 6 API nodes + fake robot   ┐ one session each, a random domain 150-199
   │ 3 start the controller after 3 s       ┘ FAST_PARAMETERS unless --real-time
   │ 4 record /hsm_ros_msg, /hsm_ros_str_msg, /odom        Probe of hsm_test_utils
   │ 5 stop: every positive rule holds | the duration | the controller ends | SIGTERM
   ▼
 stdout: the timeline, the controller output, a line per rule     --json: diagnostics
```

```
ros2 run hsm_tests hsm_robot_test DIAGRAM [--duration S] [--pose X Y THETA]
    [--scan FORWARD RIGHT ELSEWHERE] [--real-time] [--domain N] [--json FILE] [--verbose]
```

The expectations are the lines of the diagram's `ros2 hsm test` comment, parsed by the
generator (`gencode.parse_test_rules`):

| line | holds when |
|---|---|
| `EVENT [xN] [within S]` | `EVENT` is reported at least `N` times within `S` s of the controller start |
| `never EVENT` | `EVENT` is not reported while the run lasts |

Without the comment the run is a smoke run: the controller must survive the duration or end
with 0. The timer ticks are counted but left out of the timeline.

| exit | meaning | diagnostics |
|---|---|---|
| 0 | passed | `[]` |
| 1 | a rule failed, the controller ended with an error, or the diagram is rejected | `test-failed` on the comment, `test-controller` with the controller's last error, or the generator's finding |
| 2 | a usage error | - |
| 4 | `hsmd` is not installed, or a node of the framework failed to start | - |

An error of the diagram code stops the dispatcher, and the controller exits with 1; its log
line (`The state machine is stopped: ...`, with the state and the line) is the message of the
`test-controller` finding.

Every process is started in a session of its own; the end of the run, SIGTERM and SIGINT
stop each session (SIGTERM, SIGKILL after 3 s) and remove the temporary directory, so a
cancelled run leaves no node behind in the domain.
