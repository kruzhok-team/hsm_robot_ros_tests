# How to Write an Integration Test

An integration test drives a whole diagram: the controller generated from it, the module
nodes of the API and the fake robot run as separate processes, and the test watches them
from the outside. Use it to check that something reaches the robot and comes back - an API
call, an event, a state change. To check one node on its own, write a node test instead
(`hsm_robot_ros_api/test`, `turtle_driver/test`).

## 1. The file

One file per scenario in `hsm_tests/test/integration/`, named `test_<what it checks>.py`.
The launch description belongs to the file, not to the case, so a scenario which needs the
framework in a particular state gets a file of its own.

## 2. The skeleton

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
# a domain of this file's own; 91, 93, 94 and 96 are taken
DOMAIN = 95


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
are worth driving: **turtle-square** and **turtle-square-wheels**.

## 3. What the probe gives you

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

## 4. Driving the robot

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

## 5. Five things which will bite you

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

## 6. Running it

```
colcon build && colcon test --packages-select hsm_tests
colcon test-result --verbose
```

While writing, run the file directly for the output of a failure:

```
cd hsm_tests && python3 -m pytest test/integration/test_something.py -q
```

A case which passes alone and fails under `colcon test` is almost always one of the five
above - most often the shared framework or the alphabetical order.
