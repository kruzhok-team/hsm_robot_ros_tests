# The Integration Test Tier (L3)

The tests of this directory run the generated HSM controller together with the HSM Robot API
nodes against the `fake_robot` node. No graphical simulator is started and no display is needed.

The tier runs in real time, with the periods of the nodes shortened by their own parameters -
the fake robot has a clock made of parameters, so a diagram waiting for the one minute tick
costs a tenth of a second. There is no simulated clock: `/clock` was measured for this and
rejected, for the reasons given in `docs/hsm_robot_testing.md`.

This is the only tier able to check the laser scan driven navigation events
(`COLLISION_WARNING`, `COLLISION_DETECTED`, `RIGHT_OPEN_SPACE`), because the turtlesim
simulator does not publish the `/scan` topic.

| | |
| --- | --- |
| `test_navigation_loop.py` | the diagram sends the robot around four corners and moves on after every arrival |
| `test_scan_events.py` | the obstacle events, placed at the thresholds the navigation module compares against |
| `test_timer_loop.py` | the wheels and the timer driving the diagram instead of a goal |
| `test_missing_module.py` | a controller started without its module nodes reports and gives up |

The tests are marked with the `integration` pytest marker and are described in
`docs/hsm_robot_testing.md`. The recipe for writing one is §5 of `docs/howto_write_a_test.md`.

```
colcon test --packages-select hsm_tests --pytest-args -m integration
```
