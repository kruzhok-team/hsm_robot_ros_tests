# -----------------------------------------------------------------------------
# The Cyberiada HSM-to-ROS2 testing environment
# -----------------------------------------------------------------------------
#
# A diagram reads its route from the storage and drives it
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

# The move-along-traj diagram loads a route from the storage module and drives to the
# points of it one after another, deciding with a guard whether there is a point left.
# The route is written to the storage of the test before the framework starts, so the
# diagram has something to read; nothing is written into the storage of the developer.

import json
import os
import shutil
import tempfile
import unittest

import launch_testing
import pytest
import rclpy
from launch import LaunchDescription

from geometry_msgs.msg import PoseStamped

from hsm_interfaces.msg import SimpleMessage
from hsm_tests.launch_support import framework, use_domain
from hsm_test_utils import Probe

MOVE_COMPLETED = SimpleMessage.MSG_NAVIGATION_MOVE_COMPLETED

STARTUP = 60.0
DOMAIN = 97
# the route the diagram is given, in the order it has to drive it
ROUTE = [[1.0, 1.0], [2.0, 1.0], [2.0, 2.0]]


@pytest.mark.launch_test
def generate_test_description():
    use_domain(DOMAIN)
    storage = tempfile.mkdtemp(prefix='hsm_storage_')
    with open(os.path.join(storage, 'path.json'), 'w', encoding='utf-8') as f:
        json.dump(ROUTE, f)
    actions, directory = framework('move-along-traj', DOMAIN, storage_path=storage,
                                   goal_tolerance=0.3, arrival_tolerance=0.05,
                                   linear_limit=8.0)
    generate_test_description.directory = directory
    generate_test_description.storage = storage
    return LaunchDescription(actions + [launch_testing.actions.ReadyToTest()])


class TestStorageLoop(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.probe = Probe(name='hsm_storage_loop_probe')
        cls.goals = cls.probe.record('/goal_pose', PoseStamped)

    @classmethod
    def tearDownClass(cls):
        cls.probe.destroy_node()
        rclpy.shutdown()
        for path in ('directory', 'storage'):
            shutil.rmtree(getattr(generate_test_description, path, ''), ignore_errors=True)

    def points(self):
        return [(round(g.pose.position.x, 1), round(g.pose.position.y, 1))
                for g in self.goals]

    def test_the_diagram_reads_a_point_from_the_storage(self):
        # the diagram asked the storage module to load the route and took a point from
        # it: the point it drives to is the one which was written to the disk
        assert self.probe.wait_for(lambda: len(self.goals) >= 1, timeout=STARTUP), \
            'the diagram never asked for a point of the route'
        assert self.points()[0] == tuple(ROUTE[0])

    def test_the_diagram_drives_the_whole_route(self):
        assert self.probe.wait_for(lambda: len(self.goals) >= len(ROUTE), timeout=STARTUP), \
            'the diagram drove {} points of the {}'.format(len(self.goals), len(ROUTE))
        assert self.points()[:len(ROUTE)] == [tuple(point) for point in ROUTE]

    def test_every_point_is_reached(self):
        assert self.probe.wait_for(
            lambda: self.probe.count_events(MOVE_COMPLETED) >= len(ROUTE), timeout=STARTUP), \
            'the robot reached {} points of the {}'.format(
                self.probe.count_events(MOVE_COMPLETED), len(ROUTE))

    def test_the_route_ends(self):
        # the guard of the diagram asks the storage whether a point is left, so the
        # diagram stops asking for points when the route is driven
        assert self.probe.wait_for(
            lambda: self.probe.count_events(MOVE_COMPLETED) >= len(ROUTE), timeout=STARTUP)
        self.probe.spin(2.0)
        assert len(self.goals) == len(ROUTE), \
            'the diagram asked for {} points and the route has {}'.format(
                len(self.goals), len(ROUTE))
