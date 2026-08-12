# -----------------------------------------------------------------------------
# The Cyberiada HSM-to-ROS2 testing environment
# -----------------------------------------------------------------------------
#
# A diagram waiting for a timer: the module drives the state machine
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

# The turtle-square-wheels diagram drives the square with the wheels and a timer instead
# of the navigation module: it turns for a while, drives ahead for a while, and the timer
# is what tells it when to stop doing either. The whole loop is the diagram asking the
# timer module to start, the module reporting the elapsed time with the name of the timer,
# and the state machine leaving the state on that event.

import shutil
import unittest

import launch_testing
import pytest
import rclpy
from launch import LaunchDescription

from nav_msgs.msg import Odometry

from hsm_interfaces.msg import StringArgMessage
from hsm_tests.launch_support import framework, use_domain
from hsm_test_utils import Probe

TIMER_ELAPSED = StringArgMessage.MSG_TIMER_ELAPSED

STARTUP = 60.0
DOMAIN = 94


@pytest.mark.launch_test
def generate_test_description():
    use_domain(DOMAIN)
    actions, directory = framework('turtle-square-wheels', DOMAIN)
    generate_test_description.directory = directory
    return LaunchDescription(actions + [launch_testing.actions.ReadyToTest()])


class TestTimerLoop(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.probe = Probe(name='hsm_timer_loop_probe')
        cls.odometry = cls.probe.record('/odom', Odometry)

    @classmethod
    def tearDownClass(cls):
        cls.probe.destroy_node()
        rclpy.shutdown()
        shutil.rmtree(getattr(generate_test_description, 'directory', ''), ignore_errors=True)

    def elapsed(self):
        return [m for m in self.probe.str_events if m.code == TIMER_ELAPSED]

    def test_the_timer_reports_back_to_the_diagram(self):
        assert self.probe.wait_for(lambda: len(self.elapsed()) >= 1, timeout=STARTUP), \
            'the timer the diagram started never reported'

    def test_the_timer_carries_its_name(self):
        assert self.probe.wait_for(lambda: len(self.elapsed()) >= 1, timeout=STARTUP)
        # the name is what tells a diagram which of its timers has elapsed
        assert isinstance(self.elapsed()[0].arg, str)

    def test_the_diagram_keeps_being_driven_by_the_timer(self):
        # the diagram is a loop of turning and driving, so the timer reports again and
        # again: one report only would mean the state machine stopped moving
        assert self.probe.wait_for(lambda: len(self.elapsed()) >= 3, timeout=STARTUP), \
            'the timer reported {} times and the diagram stopped'.format(len(self.elapsed()))

    def test_the_robot_turns_and_drives(self):
        # the two states of the diagram command the wheels differently, and the robot
        # reports both through the odometry
        assert self.probe.wait_for(
            lambda: any(o.twist.twist.angular.z != 0.0 for o in self.odometry),
            timeout=STARTUP), 'the robot never turned'
        assert self.probe.wait_for(
            lambda: any(o.twist.twist.linear.x > 0.0 for o in self.odometry),
            timeout=STARTUP), 'the robot never drove ahead'
