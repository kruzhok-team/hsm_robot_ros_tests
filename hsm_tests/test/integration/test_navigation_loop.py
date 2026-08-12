# -----------------------------------------------------------------------------
# The Cyberiada HSM-to-ROS2 testing environment
# -----------------------------------------------------------------------------
#
# A diagram drives the robot: the whole loop from the state machine to the robot
# and back
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

# The turtle-square diagram drives the robot around four corners. Every corner is one
# turn of the whole loop: the diagram calls Navigation.move_to_point, the caller reaches
# the navigation module through a service, the module publishes the target point, the
# robot drives there, the module sees the arrival in the odometry and reports
# MOVE_COMPLETED, and the state machine moves to the next corner.
#
# No other tier can check this: the unit tier has no robot and the node tier runs one
# node at a time.
#
# The framework runs freely while the test watches it, so nothing is asserted about which
# corner is the first one seen: the probe starts recording after the controller has
# already begun. What is asserted is that the corners follow one another in the order of
# the diagram, which is what proves the events are closing the loop.

import shutil
import unittest

import launch_testing
import pytest
import rclpy
from launch import LaunchDescription

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry

from hsm_interfaces.msg import SimpleMessage
from hsm_tests.launch_support import framework, use_domain
from hsm_test_utils import Probe

MOVE_COMPLETED = SimpleMessage.MSG_NAVIGATION_MOVE_COMPLETED

# the corners of the diagram, in the order it visits them
CORNERS = ((2.0, 7.0), (7.0, 7.0), (7.0, 2.0), (2.0, 2.0))
STARTUP = 60.0
# the domain of this description, so the descriptions of the tier do not meet
DOMAIN = 91


@pytest.mark.launch_test
def generate_test_description():
    use_domain(DOMAIN)
    # the robot stops closer to the point than the module needs to call it reached:
    # with the two tolerances equal the robot stops exactly on the boundary and whether
    # an odometry sample lands inside it is a race
    actions, directory = framework('turtle-square', DOMAIN, linear_limit=8.0,
                                   goal_tolerance=0.3, arrival_tolerance=0.05)
    generate_test_description.directory = directory
    return LaunchDescription(actions + [launch_testing.actions.ReadyToTest()])


def corner_of(pose):
    point = (round(pose.pose.position.x, 1), round(pose.pose.position.y, 1))
    assert point in CORNERS, 'the diagram asked for {}, which is not a corner'.format(point)
    return CORNERS.index(point)


class TestNavigationLoop(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.probe = Probe(name='hsm_navigation_loop_probe')
        # the recording starts as early as possible, before the first assertion waits
        cls.goals = cls.probe.record('/goal_pose', PoseStamped)
        cls.odometry = cls.probe.record('/odom', Odometry)

    @classmethod
    def tearDownClass(cls):
        cls.probe.destroy_node()
        rclpy.shutdown()
        shutil.rmtree(getattr(generate_test_description, 'directory', ''), ignore_errors=True)

    def test_the_diagram_asks_the_navigation_module_for_a_corner(self):
        # the call of the diagram reached the module through its service, and the module
        # published the point for the robot
        assert self.probe.wait_for(lambda: len(self.goals) >= 1, timeout=STARTUP), \
            'the diagram never asked the navigation module for a point'
        corner_of(self.goals[0])

    def test_the_robot_reaches_the_corner_it_was_sent_to(self):
        assert self.probe.wait_for(lambda: len(self.goals) >= 1, timeout=STARTUP)
        target = self.goals[0].pose.position
        assert self.probe.wait_for(
            lambda: any(abs(o.pose.pose.position.x - target.x) < 0.3 and
                        abs(o.pose.pose.position.y - target.y) < 0.3
                        for o in self.odometry), timeout=STARTUP), \
            'the robot never reached the corner it was sent to'

    def test_the_arrival_comes_back_as_an_event(self):
        self.probe.wait_for_event(MOVE_COMPLETED, timeout=STARTUP)

    def test_the_diagram_moves_on_after_every_arrival(self):
        # a second corner is asked for only if the event reached the state machine, so
        # the sequence of the corners is the proof that the loop is closed
        assert self.probe.wait_for(lambda: len(self.goals) >= 5, timeout=STARTUP), \
            'the diagram asked for {} corners of the five expected'.format(len(self.goals))
        visited = [corner_of(goal) for goal in self.goals[:5]]
        expected = [(visited[0] + step) % len(CORNERS) for step in range(5)]
        assert visited == expected, \
            'the corners were visited as {} and not in the order of the diagram'.format(visited)

    def test_the_arrivals_are_reported_once_each(self):
        assert self.probe.wait_for(lambda: len(self.goals) >= 5, timeout=STARTUP)
        # one arrival per corner: the module drops the goal before it reports, so a
        # standing robot does not report the same arrival again
        assert self.probe.count_events(MOVE_COMPLETED) <= len(self.goals)
