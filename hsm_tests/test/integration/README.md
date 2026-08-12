# The Integration Test Tier (L3)

The tests of this directory run the generated HSM controller together with the HSM Robot API
nodes against the `fake_robot` node. No graphical simulator is started and no wall clock is
used: the tier runs on the simulated time driven by the test harness, so the tests are fast and
deterministic.

This is the only tier able to check the laser scan driven navigation events
(`COLLISION_WARNING`, `COLLISION_DETECTED`, `RIGHT_OPEN_SPACE`), because the turtlesim
simulator does not publish the `/scan` topic.

The tests are marked with the `integration` pytest marker and are described in
`docs/hsm_robot_testing.md`.

The directory is a placeholder: the tests are implemented in a separate step.
