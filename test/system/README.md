# The System Test Tier (L4)

The tests of this directory run the whole HSM-to-ROS2 pipeline: a CyberiadaML diagram is
converted by the code generator, the resulting package is built, and the generated controller is
started together with the HSM Robot API nodes, the turtle driver and the turtlesim simulator.
The simulator is started in the offscreen mode, so the tier requires no display.

The tier runs in real time and is therefore small and slow. It checks that the workflow
documented for the framework users actually works from end to end.

The tests are marked with the `system` pytest marker and are described in
`docs/hsm_robot_testing.md`.

The directory is a placeholder: the tests are implemented in a separate step.
