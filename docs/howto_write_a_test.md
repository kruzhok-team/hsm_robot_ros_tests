# How to Write a Test

The framework is tested in five tiers. Each has its own place in the workspace, its own
marker and its own conventions, and this document is the recipe for adding a test to any of
them. What the tiers are for and why they are split this way is in
`hsm_robot_testing.md`.

## 1. Which tier does your test belong to

Start from what you are checking, not from where the code lives.

| What you are checking | Tier | Where the test goes | Marker |
| --- | --- | --- | --- |
| the coding style of a package | L0 | `<package>/test/test_flake8.py`, `test_pep257.py` | `linter` |
| the code the generator writes, or a diagram it must reject | L1 | `hsm_robot_ros_generator/test` | `unit` |
| the ROS2 objects one node declares - its topics, services and parameters | L2 | `hsm_robot_ros_api/test`, `turtle_driver/test` | `node` |
| a whole diagram: an API call reaching a module and the event coming back | L3 | `hsm_tests/test/integration` | `integration` |
| the shipped system: the real driver and the real simulator | L4 | `hsm_tests/test/system` | `system` |

Two rules keep the tiers apart. A test belongs to the **lowest** tier which can check the
thing - the higher tiers are slower and fail in more ways, so a question a node test can
answer should not be asked of a diagram. And a tier never reaches past its own boundary: a
node test starts one node, an L1 test starts no ROS2 at all.

## 2. A static check (L0)

There is nothing to write. A new package gets the tier by copying three files from an
existing one, for example from `hsm_robot_ros_generator/test`:

```
test/test_flake8.py
test/test_pep257.py
test/ament_flake8.ini
```

and declaring `ament_flake8`, `ament_pep257` and `python3-pytest` in its `package.xml`,
plus the `linter` marker in its `setup.cfg`.

**Copy the framework's `test_flake8.py`, not the one ament generates.** The framework
passes its own configuration explicitly:

```python
config = os.path.join(os.path.dirname(__file__), 'ament_flake8.ini')
rc, errors = main_with_errors(argv=['--config', config])
```

The stock ament template takes no `--config`, so a file copied from upstream lints against
ament's defaults instead and silently disagrees with the rest of the workspace.

What `ament_flake8.ini` decides, and why - it **replaces** the ament configuration rather
than extending it, which is why it repeats ament's own defaults:

* `max-line-length = 120`, raised from ament's 99;
* the import ordering (`I100`, `I101`, `I201`) and the quote style (`Q000`) are off; the
  framework does not follow the ament conventions there;
* `jobs = 1`, because flake8 parallelises by forking, colcon runs the tests in a worker
  thread, and the timeout plugin holds a timer thread across that fork - which Python
  reports as a possible deadlock. The packages are small enough for one process.

## 3. A generator test (L1)

The generator is tested with no ROS2 running and no robot of any kind. A workspace with
`hsm_interfaces` built and sourced is still needed, because the generated
`hsm_controller/constants.py` imports it.

Three properties are checked separately, because each fails differently. The fixtures for
all of them are in `test/conftest.py`: `generate` and `run_generator` run the generator the
way a user runs it, `controller_module` finds the generated module inside a package,
`generated` generates into a temporary directory, and `EXIT_OK` / `EXIT_PARSER_ERROR` /
`EXIT_GENERATOR_ERROR` name the exit codes.

### Adding a diagram the generator must accept

Drop the file into `test/diagrams/valid/`. It is enrolled automatically - `diagrams(kind)`
globs the directory - so it immediately joins `test_golden.py` and
`test_generated_package.py`. Then record its reference output:

```
./test/regold.sh
```

and commit the recorded file with the diagram. The script needs the sourced workspace, and
it regolds the diagrams of `test/diagrams/valid/` **and** of `examples/`; it normalises the
copyright year, which is why `test_golden.py` carries a matching `YEAR_RE`.

The golden test fails in both directions on purpose: a diagram with no reference file
fails with *"no reference file for X; run test/regold.sh to record it"*, and a reference
file with no diagram fails too, so a deleted diagram cannot leave its golden behind.

Run `regold.sh` again after any intended change to the templates or the conversion, and
review the difference as part of the patch - that difference is the point of the tier.

### Adding a diagram the generator must reject

This one is a **two-place edit**. Drop the file into `test/diagrams/invalid/`, and add an
entry to the `REPORTED` dictionary in `test_errors.py`:

```python
REPORTED = {
    'no_initial': 'no initial pseudostate',
    ...
    'your_new_fixture': 'the phrase its report has to contain',
}
```

A fixture without an entry **fails** rather than being skipped, because the test asserts
every invalid diagram is named there. The phrase is what makes the report useful: the
message has to name the problem and not only the file.

### Checking what the state machine does

`test_sm_behaviour.py` imports the generated module with the API callers replaced by the
stubs of `hsm_stubs.py` and dispatches events into the state machine. Use it when the
question is *which transition runs*, which the reference comparison cannot answer - a
generated file can change and still be correct, and can stay identical and still be wrong.

Worked examples: `test_golden.py`, `test_errors.py`, `test_sm_behaviour.py`.

## 4. A node test (L2)

A node test starts **one** node of the framework and drives it through the ROS2 objects it
declares. Use it for everything about a single node: the services it serves, what it
publishes, how it reacts to a topic, what its parameters do.

### The skeleton

The fixtures live in `hsm_test_utils` and are re-exported by the package's own
`test/conftest.py` so pytest can see them:

```python
from hsm_test_utils import (isolated_domain, node_factory,  # noqa: F401
                            storage_directory)
```

A test then reads:

```python
import pytest

from hsm_interfaces.srv import DebugPrint
from hsm_robot.debug import ROSDebug

pytestmark = pytest.mark.node


def test_the_node_answers_the_call(node_factory):
    ctx = node_factory(ROSDebug)
    request = DebugPrint.Request()
    request.s = 'hello'
    response = ctx.probe.call('hsm_ros_debug_print', DebugPrint, request)
    assert response is not None
```

`node_factory(NodeClass, **parameters)` initialises rclpy, builds a
`SingleThreadedExecutor`, constructs the node, adds it and a probe to the executor, and
tears all of it down afterwards. It returns a `NodeUnderTest` with `.node`, `.probe` and
`.executor`.

**One node at a time.** The factory asserts it. The node names are constants of the
classes, so a second node of the same class would clash with the first.

### Overriding a parameter

This is the part which surprises people. The node classes take no constructor arguments,
so a test cannot hand them values. Every node reads the arguments of the context it is
built in, so the overrides go to `rclpy.init`, and `node_factory` does that for you:

```python
ctx = node_factory(ROSTimer, tick_period=0.02, second_period=0.05)
```

**The type comes from the literal.** A parameter declared as a double needs `1.0` and not
`1`, exactly as on the command line. A test which shortens a period is the normal case:
the tiers above run in real time, so a node test that waits a real minute for a minute tick
is a test nobody will run.

### What the probe gives you

The same `Probe` serves this tier and the two above it.

| | |
| --- | --- |
| `record(topic, type)` | start recording, returns the list the messages arrive in |
| `publish(topic, message)` | send a message, waiting for a listener first |
| `call(service, type, request)` | call a service and return the response |
| `wait_for(condition, timeout)` | spin until the condition holds, returns whether it did |
| `wait_for_event(code, timeout)` | wait for an HSM event, or fail naming the code |
| `wait_for_str_event(code, timeout)` | the same for an event with a string argument |
| `expect_no_event(code, duration)` | assert an event does **not** arrive |
| `count_events(code)`, `event_codes()` | what has been reported so far |
| `spin(duration)`, `clear()` | let the framework run; forget what was recorded |

`probe.events` and `probe.str_events` hold everything from `/hsm_ros_msg` and
`/hsm_ros_str_msg`; they are recorded from the moment the probe is built.

### The rest of the conventions

* the tests never join the graph of the developer: `isolated_domain` sets domain 73,
  overridable with `HSM_TEST_DOMAIN_ID`, plus localhost-only discovery. `node_factory`
  pulls it in, so a test gets the isolation by using the factory;
* a test of the `Storage` module takes `storage_directory` as well, so the module writes
  into the test's own directory and never into the storage of the developer;
* message builders belong in the package's `test/conftest.py` next to the fixtures -
  `odometry()` in the API package, `turtle_pose()`, `goal()` and `cancellation()` in the
  driver;
* a package hosting this tier declares `hsm_test_utils` and `python3-pytest-timeout` in
  `package.xml`, and the `node` marker with `timeout = 60` in `setup.cfg`.

Worked examples: **`test_debug_node.py`**, which is the whole pattern on one screen, and
`test_timer_node.py` for the period-shrinking idiom.

## 5. An integration test (L3)

An integration test drives a whole diagram: the controller generated from it, the module
nodes of the API and the fake robot run as separate processes, and the test watches them
from the outside. Use it to check that something reaches the robot and comes back - an API
call, an event, a state change.

### The file

One file per scenario in `hsm_tests/test/integration/`, named `test_<what it checks>.py`.
The launch description belongs to the file, not to the case, so a scenario which needs the
framework in a particular state gets a file of its own.

### The skeleton

```python
import shutil
import unittest

import launch_testing
import pytest
import rclpy
from launch import LaunchDescription

from nav_msgs.msg import Odometry

from hsm_interfaces.msg import SimpleMessage
from hsm_tests.launch_support import framework, use_domain
from hsm_test_utils import Probe

STARTUP = 60.0
# a domain of this file's own; 91, 93, 94, 95 and 96 are taken, and 101-104 by the
# system tier
DOMAIN = 92


@pytest.mark.launch_test
def generate_test_description():
    use_domain(DOMAIN)
    actions, directory = framework('turtle-square', DOMAIN,
                                   goal_tolerance=0.3, arrival_tolerance=0.05)
    generate_test_description.directory = directory
    return LaunchDescription(actions + [launch_testing.actions.ReadyToTest()])


class TestSomething(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.probe = Probe(name='hsm_something_probe')
        # start recording before the first case waits for anything
        cls.odometry = cls.probe.record('/odom', Odometry)

    @classmethod
    def tearDownClass(cls):
        cls.probe.destroy_node()
        rclpy.shutdown()
        shutil.rmtree(getattr(generate_test_description, 'directory', ''),
                      ignore_errors=True)

    def test_the_robot_moves(self):
        assert self.probe.wait_for(
            lambda: any(o.twist.twist.linear.x > 0.0 for o in self.odometry),
            timeout=STARTUP), 'the diagram never set the robot moving'
```

`framework(diagram, domain, **parameters)` generates the diagram into a temporary
directory and returns the launch actions plus that directory, which `tearDownClass`
removes. Any keyword is passed to every node as a parameter, so this is where a test
shortens a period or retunes a tolerance. Only the diagrams the framework supports today
are worth driving: **turtle-square**, **turtle-square-wheels**, **move-along-traj** and
**save-path**. The last two read and write a storage, so a test of them passes
`storage_path` as well - the module then writes into the directory of the test instead of
the storage of the developer, and a test which needs a route to start from writes the JSON
file itself, as `test_trajectory_loop.py` does.

The probe is the one described in §4; here it is built directly rather than by the node
factory, because the nodes are separate processes and the probe is the only one in this
one.

### Driving the robot

The fake robot is told what the world looks like through two topics, both
`std_msgs/Float64MultiArray`:

```python
from std_msgs.msg import Float64MultiArray

def place(probe, forward, right, elsewhere=10.0):
    # what the laser scan reports ahead, to the right and everywhere else
    msg = Float64MultiArray()
    msg.data = [float(forward), float(right), float(elsewhere)]
    probe.publish('/fake_robot/obstacles', msg)

def teleport(probe, x, y, theta=0.0):
    # move the robot without driving it
    msg = Float64MultiArray()
    msg.data = [float(x), float(y), float(theta)]
    probe.publish('/fake_robot/teleport', msg)
```

The distances the navigation module compares against are its own parameters, so a test can
place an obstacle exactly at a threshold: `collision_range` 0.15, `obstacle_range` 0.45,
`open_space_range` 0.5 by default.

The starting world is better set at launch than afterwards, because a diagram reacts to
the scan from the moment it starts:

```python
framework('turtle-square', DOMAIN, scan_forward=10.0, scan_right=0.2)
```

The robot also takes `initial_x`, `initial_y`, `initial_theta`, `control_period`,
`scan_period`, `arrival_tolerance`, `linear_k`, `angular_k`, `linear_limit`,
`angular_limit`, `progress_epsilon`, `no_progress_limit`, `odom_frame`, `base_frame` and
`publish_scan`.

### Five things which will bite you

Each of these cost a debugging session; none is obvious from the outside.

**The modules publish once, the robot reports continuously.** A module publishes a
velocity only when the diagram calls it, so a command given while the controller was
starting is gone before a test can see it. Watch `/odom`, which never stops, rather than
`/cmd_vel`, which speaks once.

**The cases of a file run in alphabetical order**, not in the order they are written, and
they share one running framework. Either make every case independent of the others, or
write the scenario as a single case.

**The framework runs freely** while the test watches. Do not assume the first message you
see is the first the framework sent - the probe starts after the controller. Assert on
what follows what, and count events before an action to compare after it.

**The tolerances have to differ.** If the robot stops as far from a point as the module
needs to call it reached, whether an odometry sample lands inside the tolerance is a race.
Give the module the larger one: `goal_tolerance=0.3, arrival_tolerance=0.05`.

**Do not use `proc_output` or `proc_info`.** `colcon` runs the tests in a worker thread,
where the process supervision of launch reports nothing. If a case is about how one
process starts, start it yourself with `subprocess` and read its output -
`test_missing_module.py` does exactly that and needs no launch description at all.

Worked examples: **`test_navigation_loop.py`** for the ordinary case, and
`test_missing_module.py` for the one that starts its own process.

## 6. A system test (L4)

A test in `hsm_tests/test/system/` is the same file with `system_framework` in the place of
`framework`: turtlesim and the turtle driver run instead of the fake robot, and the marker comes
from the directory the file is in.

```python
actions, directory = system_framework('turtle-square', DOMAIN)
```

Three things are different, and all three follow from the simulator being real.

**Do not shorten anything.** `system_framework` shortens the startup handshake and nothing else,
on purpose. The fake robot has a clock made of parameters; the simulator moves the turtle at the
speed the driver asks for, so the only thing a parameter changes is that speed - the behaviour
the tier is checking. A square takes twenty seconds and that is the price.

**Assert on `/turtle1/pose`, not on `/odom`.** The pose is what the simulator computed, and the
odometry is the driver's copy of it. Watching the copy tests the driver twice and the simulator
never.

```python
from turtlesim.msg import Pose
cls.poses = cls.probe.record('/turtle1/pose', Pose)
```

**Mind the walls.** The world is 11.089 units across and the turtle is held at the edge, so a
diagram can look correct while its trajectory is nonsense. Driving faster than the framework
ships does exactly this: the turn rate is capped, so the turtle leaves the square as an arc.
Assert the turtle stayed inside, and do not retune the speed to make a test quicker.

**Two diagrams in one scenario** do not fit `system_framework`, which generates one. Build the
description from the pieces it is made of - `generate_controller`, `api_nodes`, `turtlesim`,
`driver` and `controller` - and start the second controller with a `TimerAction` of its own,
after the first one has done its half of the job. `test_save_and_replay.py` is the worked
example: one diagram records the path of the turtle and the other drives it again, and what
joins them is a file on the disk.

To build the package the way a user does rather than running it from a directory, use
`build_controller`, `installed_executables` and `installed_controller` -
`test_documented_workflow.py` is the worked example.

## 7. Running the tests

Everything, the way the continuous integration runs it:

```
colcon build && colcon test && colcon test-result --verbose
```

One tier at a time, by its marker:

```
colcon test --pytest-args -m linter                                         # L0
colcon test --packages-select hsm_generator --pytest-args -m unit           # L1
colcon test --packages-select hsm_robot turtle_driver --pytest-args -m node # L2
colcon test --packages-select hsm_tests --pytest-args -m integration        # L3
colcon test --packages-select hsm_tests --pytest-args -m system             # L4
```

While writing, run the file directly for the output of a failure - `colcon` hides it:

```
cd hsm_tests && python3 -m pytest test/integration/test_something.py -q
```

A case which passes alone and fails under `colcon test` is almost always one of the five
things in §5 - most often the shared framework or the alphabetical order.
