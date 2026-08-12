# -----------------------------------------------------------------------------
# The Cyberiada HSM-to-ROS2 testing environment
# -----------------------------------------------------------------------------
#
# The obstacle events of the navigation module, reported from a real scan
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

# The turtlesim simulator publishes no laser scan at all, so these events are unreachable
# in the system tier, and the node tier has no robot to produce a scan from a distance.
# The events follow the scan whatever the diagram is doing, so the cases here are
# independent of one another.

import shutil
import unittest

import launch_testing
import pytest
import rclpy
from launch import LaunchDescription

from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray

from hsm_interfaces.msg import SimpleMessage
from hsm_tests.launch_support import framework, use_domain
from hsm_test_utils import Probe

COLLISION_WARNING = SimpleMessage.MSG_NAVIGATION_COLLISION_WARNING
COLLISION_DETECTED = SimpleMessage.MSG_NAVIGATION_COLLISION_DETECTED
RIGHT_OPEN_SPACE = SimpleMessage.MSG_NAVIGATION_RIGHT_OPEN_SPACE
STOP_COMPLETED = SimpleMessage.MSG_WHEELS_STOP_COMPLETED

OBSTACLES_TOPIC = '/fake_robot/obstacles'
STARTUP = 90.0
# the domain of this description, so the descriptions of the tier do not meet
DOMAIN = 93

# the thresholds the navigation module compares the scan against
COLLISION_RANGE = 0.15
OBSTACLE_RANGE = 0.45
OPEN_SPACE_RANGE = 0.5


@pytest.mark.launch_test
def generate_test_description():
    use_domain(DOMAIN)
    actions, directory = framework('right_hand_maze_solver', DOMAIN)
    generate_test_description.directory = directory
    return LaunchDescription(actions + [launch_testing.actions.ReadyToTest()])


def obstacles(forward, right, elsewhere=10.0):
    msg = Float64MultiArray()
    msg.data = [float(forward), float(right), float(elsewhere)]
    return msg


class TestScanEvents(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.probe = Probe(name='hsm_scan_events_probe')
        cls.twists = cls.probe.record('/cmd_vel', Twist)

    @classmethod
    def tearDownClass(cls):
        cls.probe.destroy_node()
        rclpy.shutdown()
        shutil.rmtree(getattr(generate_test_description, 'directory', ''), ignore_errors=True)

    def place(self, forward, right, elsewhere=10.0):
        self.probe.publish(OBSTACLES_TOPIC, obstacles(forward, right, elsewhere))

    def test_a_near_obstacle_is_reported_as_a_warning(self):
        self.place(forward=(COLLISION_RANGE + OBSTACLE_RANGE) / 2.0, right=10.0)
        self.probe.wait_for_event(COLLISION_WARNING, timeout=STARTUP)

    def test_a_close_obstacle_is_reported_as_a_collision(self):
        self.place(forward=COLLISION_RANGE / 2.0, right=10.0)
        self.probe.wait_for_event(COLLISION_DETECTED, timeout=STARTUP)

    def test_free_space_on_the_right_is_reported(self):
        self.place(forward=10.0, right=OPEN_SPACE_RANGE * 4.0)
        self.probe.wait_for_event(RIGHT_OPEN_SPACE, timeout=STARTUP)

    def test_the_obstacle_events_keep_coming(self):
        # the events report the state of the scan and are not latched, so an obstacle
        # which does not move is reported by every scan message
        self.place(forward=COLLISION_RANGE / 2.0, right=10.0)
        before = self.probe.count_events(COLLISION_DETECTED)
        assert self.probe.wait_for(
            lambda: self.probe.count_events(COLLISION_DETECTED) > before + 2,
            timeout=STARTUP), 'the obstacle was reported once and then forgotten'
