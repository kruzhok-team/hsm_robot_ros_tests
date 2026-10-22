# The System Test Tier (L4)

The tests of this directory run the whole HSM-to-ROS2 pipeline: a CyberiadaML diagram is
converted by the code generator and the generated controller is started together with the HSM
Robot API nodes, the turtle driver and the turtlesim simulator. The simulator is started in the
offscreen mode, so the tier requires no display.

Nothing written for the tests stands between the diagram and the turtle, which is what separates
this tier from the integration one: the position asserted on is the one the simulator computed,
and the world it is asserted in has walls.

| | |
| --- | --- |
| `test_turtle_square.py` | the navigation loop: the diagram sends the turtle around four corners, and the turtle stays inside the world |
| `test_turtle_square_wheels.py` | the wheels loop: a speed and a timer instead of a goal, and the stop which only comes back because the driver copies the speed of the turtle |
| `test_documented_workflow.py` | the workflow a user follows: generate into a workspace, build it, run the installed executable, and start the simulator through the launch file which ships with the driver |
| `test_save_and_replay.py` | the two storage diagrams as two halves of one job: one saves the path of the turtle, the other drives it again |
| `test_turtle_run.py` | the turtle runner of the hsm-turtle editor plugin: the check, a run around the square traced, and a cancelled run |

The tier runs in real time and is therefore small and slow - about three minutes for the five. It
shortens nothing but the startup handshake: the clock of the tier is the simulator, and the only
thing a parameter could change is the speed of the turtle, which is the behaviour being checked.

The tests are marked with the `system` pytest marker and are described in
`docs/hsm_robot_testing.md`. The recipe for writing one is in `docs/howto_write_a_test.md`.

```
colcon test --packages-select hsm_tests --pytest-args -m system
```
