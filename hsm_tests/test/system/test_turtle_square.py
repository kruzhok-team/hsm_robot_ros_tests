# -----------------------------------------------------------------------------
# The Cyberiada HSM-to-ROS2 testing environment
# -----------------------------------------------------------------------------
#
# A diagram drives the turtle of the simulator around the square
#
# Copyright (C) 2026 Alexey Fedoseev <aleksey@fedoseev.net>
#
# This program is free software; you can redistribute it and/or
# modify it under the terms of the GNU General Public
# License as published by the Free Software Foundation; either
# version 3 of the License, or (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
# General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program. If not, see https://www.gnu.org/licenses/
#
# -----------------------------------------------------------------------------

# The same diagram as the integration tier drives, with the shipped simulator in the place
# of the fake robot: turtlesim, the turtle driver which adapts it to the API, and nothing
# written for the tests between the diagram and the turtle.
#
# What only this tier can see is the pose the simulator itself reports. The integration
# tier believes the odometry, and the odometry is published by the same fake robot which
# decides where the robot is; here the position is the one the simulator computed, and the
# odometry of the driver is only a copy of it. So the arrival is asserted on
# /turtle1/pose and not on /odom.
#
# Nothing is shortened and no tolerance is retuned. The integration tier drives the fake
# robot faster than the framework ships to keep the suite quick; the same speed here takes
# the turtle out of the world of the simulator, which is eleven units across - it is asked
# to turn no faster than it turns, so a fast enough turtle leaves the square as an arc and
# is stopped by the wall. The tier runs the system at the speed the system ships at, and a
# square takes the twenty seconds a square takes.

import shutil
import unittest

import launch_testing
import pytest
import rclpy
from launch import LaunchDescription

from geometry_msgs.msg import PoseStamped

from hsm_interfaces.msg import SimpleMessage
from hsm_tests.launch_support import system_framework, use_domain
from hsm_test_utils import Probe

from turtlesim.msg import Pose

MOVE_COMPLETED = SimpleMessage.MSG_NAVIGATION_MOVE_COMPLETED

# the corners of the diagram, in the order it visits them
CORNERS = ((2.0, 7.0), (7.0, 7.0), (7.0, 2.0), (2.0, 2.0))

# the world of the simulator is 11.088889 units across, and the turtle is stopped at the
# wall; a turtle which stays away from the edge never met one
WORLD = (0.1, 11.0)

# a square is four legs of five and a third seconds, after five of starting up
STARTUP = 120.0
# the domain of this description, so the descriptions of the tier do not meet. The
# simulator owns the turtle1 names of the whole graph, so a turtlesim of the developer
# would join a test which did not isolate itself
DOMAIN = 101


@pytest.mark.launch_test
def generate_test_description():
    use_domain(DOMAIN)
    actions, directory = system_framework('turtle-square', DOMAIN)
    generate_test_description.directory = directory
    return LaunchDescription(actions + [launch_testing.actions.ReadyToTest()])


def corner_of(pose):
    point = (round(pose.pose.position.x, 1), round(pose.pose.position.y, 1))
    assert point in CORNERS, 'the diagram asked for {}, which is not a corner'.format(point)
    return CORNERS.index(point)


class TestTurtleSquare(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.probe = Probe(name='hsm_turtle_square_probe')
        # the pose of the simulator is the truth about where the turtle is; the goals say
        # where the diagram asked it to go
        cls.poses = cls.probe.record('/turtle1/pose', Pose)
        cls.goals = cls.probe.record('/goal_pose', PoseStamped)

    @classmethod
    def tearDownClass(cls):
        cls.probe.destroy_node()
        rclpy.shutdown()
        shutil.rmtree(getattr(generate_test_description, 'directory', ''), ignore_errors=True)

    def test_the_simulator_is_running_and_the_driver_reaches_it(self):
        # the driver subscribes to the pose of the simulator and the modules of the API
        # see the robot through it, so a silent pose is a system which cannot work at all
        assert self.probe.wait_for(lambda: len(self.poses) > 0, timeout=STARTUP), \
            'the simulator published no pose'
        before = len(self.poses)
        self.probe.spin(1.0)
        published = len(self.poses) - before
        assert published > 40, \
            'the simulator published {} poses in a second'.format(published)

    def test_the_diagram_asks_the_navigation_module_for_a_corner(self):
        assert self.probe.wait_for(lambda: len(self.goals) >= 1, timeout=STARTUP), \
            'the diagram never asked the navigation module for a point'
        corner_of(self.goals[0])

    def test_the_turtle_reaches_the_corner_it_was_sent_to(self):
        assert self.probe.wait_for(lambda: len(self.goals) >= 1, timeout=STARTUP)
        target = self.goals[0].pose.position
        # the module reports the arrival as soon as the turtle is within its own tolerance
        # and the diagram gives the next corner at once, so the turtle passes through the
        # point instead of stopping on it: what is asserted is that it got there
        assert self.probe.wait_for(
            lambda: any(abs(p.x - target.x) < 0.1 and abs(p.y - target.y) < 0.1
                        for p in self.poses), timeout=STARTUP), \
            'the turtle never reached the corner the diagram sent it to'

    def test_the_arrival_comes_back_as_an_event(self):
        self.probe.wait_for_event(MOVE_COMPLETED, timeout=STARTUP)

    def test_the_diagram_moves_on_after_every_arrival(self):
        # a second corner is asked for only if the event reached the state machine, so the
        # sequence of the corners is the proof that the loop is closed through the driver
        # and the simulator
        assert self.probe.wait_for(lambda: len(self.goals) >= 5, timeout=STARTUP), \
            'the diagram asked for {} corners of the five expected'.format(len(self.goals))
        visited = [corner_of(goal) for goal in self.goals[:5]]
        expected = [(visited[0] + step) % len(CORNERS) for step in range(5)]
        assert visited == expected, \
            'the corners were visited as {} and not in the order of the diagram'.format(visited)

    def test_the_turtle_stays_inside_the_world(self):
        # The simulator stops the turtle at the wall and says so, and a turtle held against
        # a wall still reports a pose, so a diagram can look as if it works while the
        # trajectory is nonsense. This is what says the speeds of the framework suit the
        # world it is driving in. The output of the simulator cannot be read from here -
        # colcon runs the tests in a worker thread where launch reports no output at all -
        # so the wall is checked by where the turtle went.
        assert self.probe.wait_for(lambda: len(self.goals) >= 5, timeout=STARTUP)
        outside = [(round(p.x, 3), round(p.y, 3)) for p in self.poses
                   if not (WORLD[0] < p.x < WORLD[1] and WORLD[0] < p.y < WORLD[1])]
        assert not outside, \
            'the turtle went to the wall at {}'.format(outside[:3])
