# -----------------------------------------------------------------------------
# The Cyberiada HSM-to-ROS2 testing environment
# -----------------------------------------------------------------------------
#
# A diagram drives the robot along a stored trajectory
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

# The move-along-traj diagram hands a whole route to the navigation module with one call:
# Storage.points reads the saved trajectory, Navigation.move_along_traj sends it, and the
# module travels it point by point on its own. Every point it passes comes back as
# POINT_PASSED and the last one as MOVE_COMPLETED.
#
# This is the tier which can see the difference between the two: the module publishes the
# next point only after the robot has reached the current one, which no lower tier can
# check - the node tier has no robot to reach anything, and the unit tier has no module.
#
# The trajectory is written into the storage of the test rather than recorded by a diagram:
# the recording is what the system tier checks, and a timer running for its real ten
# seconds does not belong in this tier. The storage directory is a parameter, so the
# storage of the developer is never touched.

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
from nav_msgs.msg import Odometry

from hsm_interfaces.msg import SimpleMessage
from hsm_tests.launch_support import framework, use_domain
from hsm_test_utils import Probe

POINT_PASSED = SimpleMessage.MSG_NAVIGATION_POINT_PASSED
MOVE_COMPLETED = SimpleMessage.MSG_NAVIGATION_MOVE_COMPLETED

# the trajectory the diagram reads from the storage, in the order it has to travel it
ROUTE = [[2.0, 7.0], [7.0, 7.0], [7.0, 2.0]]
# how close to a point of the route the robot counts as having reached it
REACHED = 0.5
STARTUP = 90.0
# a domain of this file's own; 91, 93, 94 and 96 are taken, and 101-103 by the system tier
DOMAIN = 95


def seed_storage(route):
    # the storage the diagram loads, in the format of the storage module: one JSON file
    # per storage, holding the array of points
    directory = tempfile.mkdtemp(prefix='hsm_trajectory_storage_')
    with open(os.path.join(directory, 'path.json'), 'w') as f:
        json.dump(route, f)
    return directory


@pytest.mark.launch_test
def generate_test_description():
    use_domain(DOMAIN)
    # the robot stops closer to the point than the module needs to call it reached: with
    # the two tolerances equal whether an odometry sample lands inside the boundary is a
    # race
    storage = seed_storage(ROUTE)
    actions, directory = framework('move-along-traj', DOMAIN, linear_limit=8.0,
                                   goal_tolerance=0.3, arrival_tolerance=0.05,
                                   storage_path=storage)
    generate_test_description.directory = directory
    generate_test_description.storage = storage
    return LaunchDescription(actions + [launch_testing.actions.ReadyToTest()])


def point_of(pose):
    return [round(pose.pose.position.x, 1), round(pose.pose.position.y, 1)]


class TestTrajectoryLoop(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.probe = Probe(name='hsm_trajectory_loop_probe')
        # the recording starts as early as possible, before the first assertion waits
        cls.goals = cls.probe.record('/goal_pose', PoseStamped)
        cls.odometry = cls.probe.record('/odom', Odometry)

    @classmethod
    def tearDownClass(cls):
        cls.probe.destroy_node()
        rclpy.shutdown()
        shutil.rmtree(getattr(generate_test_description, 'directory', ''), ignore_errors=True)
        shutil.rmtree(getattr(generate_test_description, 'storage', ''), ignore_errors=True)

    def route_travelled(self):
        return self.probe.wait_for(
            lambda: self.probe.count_events(MOVE_COMPLETED) >= 1, timeout=STARTUP)

    def test_the_module_asks_for_the_points_one_by_one(self):
        # one call of the diagram, and the module publishes the points itself: the driver
        # of the platform always sees a single goal
        assert self.probe.wait_for(lambda: len(self.goals) >= len(ROUTE), timeout=STARTUP), \
            'the module published {} of the {} points'.format(len(self.goals), len(ROUTE))
        assert [point_of(goal) for goal in self.goals[:len(ROUTE)]] == ROUTE

    def test_the_robot_travels_every_point_of_the_route(self):
        assert self.route_travelled(), 'the route never completed'
        for point in ROUTE:
            assert any(abs(o.pose.pose.position.x - point[0]) < REACHED and
                       abs(o.pose.pose.position.y - point[1]) < REACHED
                       for o in self.odometry), \
                'the robot never reached {}'.format(point)

    def test_every_passed_point_comes_back_as_an_event(self):
        assert self.route_travelled(), 'the route never completed'
        # the events are what the diagram sees: one per point of the route
        assert self.probe.count_events(POINT_PASSED) == len(ROUTE), \
            'the route reported {} passed points of the {} expected'.format(
                self.probe.count_events(POINT_PASSED), len(ROUTE))

    def test_the_route_is_completed_once(self):
        assert self.route_travelled(), 'the route never completed'
        # the module drops the point before it reports, so a standing robot does not
        # report the end of the route again
        self.probe.spin(0.5)
        assert self.probe.count_events(MOVE_COMPLETED) == 1
        # and the route is over: nothing is published after the last point
        assert len(self.goals) == len(ROUTE)
