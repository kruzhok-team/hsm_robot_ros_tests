# -----------------------------------------------------------------------------
# The Cyberiada HSM-to-ROS2 testing environment
# -----------------------------------------------------------------------------
#
# One diagram records the path of the turtle and another one drives it again
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

# The two storage diagrams are two halves of one job, and this is the test of the whole of
# it: save-path writes down where the robot has been, move-along-traj sends it there again.
# Nothing but this tier can check that the halves fit, because what joins them is a file on
# the disk written by one run and read by another.
#
# The scenario, in one launch description:
#
#   1. the turtle is driven by hand - the test publishes a velocity the way the wheels
#      module would - while the gps_tracker diagram saves the position every ten seconds;
#   2. the turtle is stopped and carried back to the beginning of the path it has just
#      drawn, which is where a route is replayed from;
#   3. the move_along_trajectory diagram starts, loads the very same storage and hands the
#      whole route to the navigation module;
#   4. the module drives the turtle through the recorded points, reporting each of them.
#
# The turtle is carried back with the teleport service of the simulator, the way an operator
# carries a robot back to the start. Without it the route is replayed from its far end and
# every point of it lies behind the turtle: the driver of this platform steers towards the
# goal while it drives forward, so a point behind means driving away from it first, and its
# own watchdog then calls the goal unreachable. Which is honest - it is the driver saying
# what it can do, and not something for this test to retune away.
#
# The tracker keeps recording while the route is replayed, so the storage file grows under
# the test. Nothing is asserted about its final length: the route the diagram received is
# the file as it was when it loaded it, and the assertions compare what was published
# against the beginning of the file, which the recording can only extend.
#
# It is one test case and not four. The cases of a file share one running framework and run
# in alphabetical order, and these steps have to happen in the order they are written.

import json
import math
import os
import shutil
import tempfile
import unittest

import launch_testing
import pytest
import rclpy
from launch import LaunchDescription
from launch.actions import TimerAction

from geometry_msgs.msg import PoseStamped, Twist
from turtlesim.msg import Pose
from turtlesim.srv import TeleportAbsolute

from hsm_interfaces.msg import SimpleMessage
from hsm_tests.launch_support import (CONTROLLER_DELAY, STARTUP_PARAMETERS, api_nodes,
                                      controller, diagram, driver, generate_controller,
                                      turtlesim, use_domain)
from hsm_test_utils import Probe

POINT_PASSED = SimpleMessage.MSG_NAVIGATION_POINT_PASSED
MOVE_COMPLETED = SimpleMessage.MSG_NAVIGATION_MOVE_COMPLETED

VELOCITY_TOPIC = '/cmd_vel'
POSE_TOPIC = '/turtle1/pose'
TELEPORT_SERVICE = '/turtle1/teleport_absolute'
GOAL_TOPIC = '/goal_pose'
STORAGE = 'path'
# how far before the first point of the path the turtle is put back, along the heading it
# had when the point was saved
APPROACH = 0.5

# the turtle is driven slowly on purpose: the recording takes half a minute and the world
# is 11.089 units across, so a faster turtle spends it held against the wall with every
# recorded point in the same place
RECORD_SPEED = 0.1
# the timer of the diagram elapses every ten seconds, and the route needs points to be a
# route; nothing here is shortened, because the clock of this tier is the simulator
POINTS = 3
RECORDING = 60.0
# when the replaying controller is started - after the recording above has finished
REPLAY_DELAY = 45.0
REPLAY = 90.0
# how close to a recorded point the turtle has to come for the point to count as visited
VISITED = 0.6
# the turtlesim world; the simulator holds the turtle at the edge, so a diagram can look
# correct while its route is nonsense
WORLD = 11.089

# a domain of this file's own; 101-103 are taken by the other files of the tier
DOMAIN = 104


@pytest.mark.launch_test
def generate_test_description():
    use_domain(DOMAIN)
    # the storage of the test, so the two diagrams share a file and the storage of the
    # developer is never touched
    storage = tempfile.mkdtemp(prefix='hsm_save_replay_storage_')
    recorder, recorder_module = generate_controller(diagram('save-path'))
    replay, replay_module = generate_controller(diagram('move-along-traj'))
    actions = api_nodes(DOMAIN, defaults=STARTUP_PARAMETERS, storage_path=storage)
    actions.append(turtlesim(DOMAIN))
    actions.append(driver(DOMAIN))
    actions.append(TimerAction(
        period=CONTROLLER_DELAY,
        actions=[controller(recorder, recorder_module, DOMAIN,
                            defaults=STARTUP_PARAMETERS, storage_path=storage)]))
    actions.append(TimerAction(
        period=REPLAY_DELAY,
        actions=[controller(replay, replay_module, DOMAIN,
                            defaults=STARTUP_PARAMETERS, storage_path=storage)]))
    generate_test_description.directories = (recorder, replay, storage)
    generate_test_description.storage = os.path.join(storage, STORAGE + '.json')
    return LaunchDescription(actions + [launch_testing.actions.ReadyToTest()])


def drive(probe, speed):
    # the velocity the wheels module publishes; the driver repeats it to the simulator
    # until it is told otherwise, so one message keeps the turtle going
    msg = Twist()
    msg.linear.x = float(speed)
    probe.publish(VELOCITY_TOPIC, msg)


def carry_back(probe, point):
    # put the turtle just before the first saved point, facing the way it faced when the
    # point was saved: a point is the (x, y, theta) triple the navigation module reports
    x, y, theta = point[0], point[1], point[2] if len(point) > 2 else 0.0
    request = TeleportAbsolute.Request()
    request.x = float(x - APPROACH * math.cos(theta))
    request.y = float(y - APPROACH * math.sin(theta))
    request.theta = float(theta)
    return probe.call(TELEPORT_SERVICE, TeleportAbsolute, request)


def recorded():
    # the storage as the module has written it so far; the file is rewritten on every
    # saved point, so a read landing in the middle of a write is answered as "nothing yet"
    try:
        with open(generate_test_description.storage) as f:
            return json.load(f)
    except (OSError, ValueError):
        return []


class TestSaveAndReplay(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.probe = Probe(name='hsm_save_replay_probe')
        # the pose is what the simulator computed; the odometry is the driver's copy of it
        cls.poses = cls.probe.record(POSE_TOPIC, Pose)
        cls.goals = cls.probe.record(GOAL_TOPIC, PoseStamped)

    @classmethod
    def tearDownClass(cls):
        cls.probe.destroy_node()
        rclpy.shutdown()
        for directory in getattr(generate_test_description, 'directories', ()):
            shutil.rmtree(directory, ignore_errors=True)

    def visited(self, point, since):
        # only the poses of the replay count: the turtle stood on every point of the route
        # while the route was being recorded, so the whole history would answer yes to
        # anything
        return any(math.hypot(pose.x - point[0], pose.y - point[1]) < VISITED
                   for pose in self.poses[since:])

    def test_the_recorded_path_is_driven_again(self):
        # 1. the turtle is driven while the tracker writes down where it has been
        assert self.probe.wait_for(lambda: len(self.poses) > 0, timeout=RECORDING), \
            'the simulator never reported the turtle'
        drive(self.probe, RECORD_SPEED)
        assert self.probe.wait_for(lambda: len(recorded()) >= POINTS, timeout=RECORDING), \
            'the tracker saved {} of the {} points'.format(len(recorded()), POINTS)
        drive(self.probe, 0.0)
        route = recorded()
        # the diagram saved the position and not the same place over and over, which is
        # what makes the recording a route
        assert len({(round(p[0], 1), round(p[1], 1)) for p in route}) > 1, route

        # 2. the turtle is carried back to where the path begins
        assert carry_back(self.probe, route[0]) is not None, 'the turtle was not carried back'
        replay_from = len(self.poses)

        # 3. the replaying diagram loads that storage and hands it to the module, which
        # publishes the points one at a time
        assert self.probe.wait_for(lambda: len(self.goals) > 0, timeout=REPLAY), \
            'the replaying diagram never asked for a point'
        published = [(g.pose.position.x, g.pose.position.y) for g in self.goals]
        assert math.hypot(published[0][0] - route[0][0],
                          published[0][1] - route[0][1]) < 0.01, \
            'the route started at {} and not at the first saved point {}'.format(
                published[0], route[0])

        # 4. the turtle is driven through the recorded points and each of them is reported
        assert self.probe.wait_for(
            lambda: self.probe.count_events(MOVE_COMPLETED) >= 1, timeout=REPLAY), \
            'the replayed route never completed'
        published = [(g.pose.position.x, g.pose.position.y) for g in self.goals]
        saved = recorded()
        for number, point in enumerate(published):
            # the tracker is still recording, so the file has grown; what the diagram
            # received is the beginning of it
            assert math.hypot(point[0] - saved[number][0],
                              point[1] - saved[number][1]) < 0.01, \
                'the point {} was published as {} and saved as {}'.format(
                    number, point, saved[number])
            assert self.visited(point, replay_from), \
                'the turtle never came to {} while replaying'.format(point)
        assert self.probe.count_events(POINT_PASSED) == len(published), \
            'the route reported {} passed points of the {} published'.format(
                self.probe.count_events(POINT_PASSED), len(published))
        assert self.probe.count_events(MOVE_COMPLETED) == 1

        # the turtle stayed in the world: a route driven into a wall looks complete too
        assert all(0.0 < pose.x < WORLD and 0.0 < pose.y < WORLD for pose in self.poses)
