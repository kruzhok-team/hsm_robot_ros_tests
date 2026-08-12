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

from nav_msgs.msg import Odometry
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
# closer than the open space range, so the right hand rule sees a wall and not a turn
WALL_ON_THE_RIGHT = 0.2


@pytest.mark.launch_test
def generate_test_description():
    use_domain(DOMAIN)
    # the robot starts with the wall on its right already: the scan of the robot is
    # free on every side by default, so the diagram of the right hand rule would be
    # turning from the moment it starts and would not be driving when the test places
    # the obstacle
    actions, directory = framework('right_hand_maze_solver', DOMAIN,
                                   scan_forward=10.0, scan_right=WALL_ON_THE_RIGHT)
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
        # the wheels module publishes a velocity only when the diagram calls it, so a
        # command given while the controller was starting is gone before a test can
        # see it. The odometry of the robot is continuous and says what it is doing
        cls.odometry = cls.probe.record('/odom', Odometry)

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
        # the way ahead is free and the wall on the right is close: the diagram of the
        # right hand rule then has nothing to react to and drives straight on. With a
        # free space on the right as well it keeps turning, and the obstacle would arrive
        # while the diagram is in a state which does not answer it
        assert self.probe.wait_for(
            lambda: any(o.twist.twist.linear.x > 0.0 for o in self.odometry[-10:]),
            timeout=STARTUP), 'the diagram never set the robot moving'
        # what happened before the obstacle appeared is counted, so the events which
        # follow are the answer to it and not something the diagram did earlier
        warnings = self.probe.count_events(COLLISION_WARNING)
        stops = self.probe.count_events(STOP_COMPLETED)
        self.place(forward=(COLLISION_RANGE + OBSTACLE_RANGE) / 2.0,
                   right=WALL_ON_THE_RIGHT)
        assert self.probe.wait_for(
            lambda: self.probe.count_events(COLLISION_WARNING) > warnings,
            timeout=STARTUP), 'the obstacle was not reported to the diagram'
        assert self.probe.wait_for(
            lambda: self.probe.count_events(STOP_COMPLETED) > stops, timeout=STARTUP), \
            'the diagram did not stop the wheels at the obstacle'
