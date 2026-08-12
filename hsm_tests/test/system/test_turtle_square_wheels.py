# -----------------------------------------------------------------------------
# The Cyberiada HSM-to-ROS2 testing environment
# -----------------------------------------------------------------------------
#
# A diagram drives the turtle of the simulator with the wheels and a timer
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

# The other way a diagram moves a robot: no goal and no navigation, only a speed given to
# the wheels and a timer to say when to stop. The turtle drives the same square blindly,
# and every leg is the diagram asking the wheels to move, the timer reporting the time,
# and the diagram asking the wheels to stop.
#
# The stop is the part which is worth running against the real simulator. The wheels module
# reports STOP_COMPLETED when it sees the speed of the robot fall to zero in the odometry,
# so the event exists only if the driver copies the speed of the turtle and not only its
# position. A driver which published a position alone would look correct in every other
# test and deadlock a diagram which waits for the stop.
#
# The timer runs at the speed the framework ships at, because a diagram which drives for
# five seconds drives the turtle five seconds worth of the world. Shortening the second
# would keep the events and lose the square.

import shutil
import unittest

import launch_testing
import pytest
import rclpy
from launch import LaunchDescription

from hsm_interfaces.msg import SimpleMessage, StringArgMessage
from hsm_tests.launch_support import system_framework, use_domain
from hsm_test_utils import Probe

from turtlesim.msg import Pose

STOP_COMPLETED = SimpleMessage.MSG_WHEELS_STOP_COMPLETED
TIMER_ELAPSED = StringArgMessage.MSG_TIMER_ELAPSED

# the world of the simulator; the blind square keeps well over a unit away from every wall
WORLD = (0.1, 11.0)

# a leg is five seconds of driving and four of turning, after five of starting up
STARTUP = 120.0
DOMAIN = 102


@pytest.mark.launch_test
def generate_test_description():
    use_domain(DOMAIN)
    actions, directory = system_framework('turtle-square-wheels', DOMAIN)
    generate_test_description.directory = directory
    return LaunchDescription(actions + [launch_testing.actions.ReadyToTest()])


class TestTurtleSquareWheels(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.probe = Probe(name='hsm_turtle_wheels_probe')
        cls.poses = cls.probe.record('/turtle1/pose', Pose)

    @classmethod
    def tearDownClass(cls):
        cls.probe.destroy_node()
        rclpy.shutdown()
        shutil.rmtree(getattr(generate_test_description, 'directory', ''), ignore_errors=True)

    def elapsed(self):
        return [m for m in self.probe.str_events if m.code == TIMER_ELAPSED]

    def test_the_speed_of_the_diagram_reaches_the_turtle(self):
        # the wheels module publishes a velocity, the driver passes it to the simulator,
        # and the simulator reports the turtle moving at it
        assert self.probe.wait_for(
            lambda: any(p.linear_velocity > 0.0 for p in self.poses), timeout=STARTUP), \
            'the turtle never drove'
        assert self.probe.wait_for(
            lambda: any(p.angular_velocity != 0.0 for p in self.poses), timeout=STARTUP), \
            'the turtle never turned'

    def test_the_timer_keeps_driving_the_diagram(self):
        # the diagram leaves every state on the timer, so a diagram which is still moving
        # is a timer which keeps reporting
        assert self.probe.wait_for(lambda: len(self.elapsed()) >= 3, timeout=STARTUP), \
            'the timer reported {} times and the diagram stopped'.format(len(self.elapsed()))

    def test_the_stop_comes_back_through_the_odometry_of_the_driver(self):
        # the event the driver has to earn: it exists only because the driver copies the
        # speed of the turtle into the odometry the wheels module watches
        self.probe.wait_for_event(STOP_COMPLETED, timeout=STARTUP)

    def test_the_turtle_stays_inside_the_world(self):
        # the diagram drives blindly, so nothing but the lengths it was written with keeps
        # it off the wall
        assert self.probe.wait_for(lambda: len(self.elapsed()) >= 3, timeout=STARTUP)
        outside = [(round(p.x, 3), round(p.y, 3)) for p in self.poses
                   if not (WORLD[0] < p.x < WORLD[1] and WORLD[0] < p.y < WORLD[1])]
        assert not outside, \
            'the turtle went to the wall at {}'.format(outside[:3])
