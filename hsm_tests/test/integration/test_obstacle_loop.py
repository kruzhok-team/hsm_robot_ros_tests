# -----------------------------------------------------------------------------
# The Cyberiada HSM-to-ROS2 testing environment
# -----------------------------------------------------------------------------
#
# The laser scan drives a diagram: the whole chain, on a framework of its own
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
DOMAIN = 92

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


class TestObstacleLoop(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.probe = Probe(name='hsm_obstacle_loop_probe')
        cls.twists = cls.probe.record('/cmd_vel', Twist)

    @classmethod
    def tearDownClass(cls):
        cls.probe.destroy_node()
        rclpy.shutdown()
        shutil.rmtree(getattr(generate_test_description, 'directory', ''), ignore_errors=True)

    def place(self, forward, right, elsewhere=10.0):
        self.probe.publish(OBSTACLES_TOPIC, obstacles(forward, right, elsewhere))

    def test_the_scan_drives_the_diagram(self):
        # the whole chain in order: the robot reports a free way, the diagram drives, an
        # obstacle appears, the module turns the distance into an event, the state
        # machine leaves the driving state and its exit stops the wheels, and the wheels
        # module reports the stopping back to the diagram.
        # This is the only case of its launch description, so the diagram is untouched
        self.place(forward=10.0, right=10.0)
        assert self.probe.wait_for(lambda: any(t.linear.x > 0.0 for t in self.twists),
                                   timeout=STARTUP), \
            'the diagram never commanded the wheels to drive'
        # what happened before the obstacle appeared is counted, so the events which
        # follow are the answer to it and not something the diagram did earlier
        warnings = self.probe.count_events(COLLISION_WARNING)
        stops = self.probe.count_events(STOP_COMPLETED)
        self.place(forward=(COLLISION_RANGE + OBSTACLE_RANGE) / 2.0, right=10.0)
        assert self.probe.wait_for(
            lambda: self.probe.count_events(COLLISION_WARNING) > warnings,
            timeout=STARTUP), 'the obstacle was not reported to the diagram'
        assert self.probe.wait_for(
            lambda: self.probe.count_events(STOP_COMPLETED) > stops, timeout=STARTUP), \
            'the diagram did not stop the wheels at the obstacle'
