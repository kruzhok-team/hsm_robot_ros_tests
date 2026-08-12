# -----------------------------------------------------------------------------
# The Cyberiada HSM-to-ROS2 testing environment
# -----------------------------------------------------------------------------
#
# The workflow the documentation gives a user, run as it is written
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

# Every other test of the framework runs the generated controller out of the directory it
# was generated into, because a generated package is always named hsm_controller and a
# workspace holds one of those at a time. That is convenient and it is not what a user
# does. A user generates the diagram into a workspace, builds it, and runs the executable
# the build installed - and the half of the workflow between the generator and the running
# controller, the packaging, is then never exercised at all.
#
# This is the one test which pays for it: it builds, which costs a second and a half, and
# it starts the simulator through the launch file which ships with the driver rather than
# through a description written for the tests. So the two things a user touches first, the
# package and the launch file, are covered where they break.
#
# The launch file is included rather than rebuilt here, and an included description cannot
# be given an environment of its own - the nodes it starts inherit the environment of the
# process which builds the description. That is why use_domain() is called first: it puts
# the domain into that environment, and the simulator of this test stays out of the way of
# the others.

import os
import shutil
import unittest

import launch_testing
import pytest
import rclpy
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource

from geometry_msgs.msg import PoseStamped

from hsm_tests.launch_support import (CONTROLLER_DELAY, STARTUP_PARAMETERS, api_nodes,
                                      build_controller, diagram, installed_controller,
                                      installed_executables, use_domain)
from hsm_test_utils import Probe

from turtlesim.msg import Pose

STARTUP = 120.0
DOMAIN = 103


@pytest.mark.launch_test
def generate_test_description():
    use_domain(DOMAIN)
    workspace, module = build_controller(diagram('turtle-square'))
    executables = installed_executables(workspace)
    generate_test_description.workspace = workspace
    generate_test_description.module = module
    generate_test_description.executables = executables

    simulator = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('turtle_driver'), 'launch',
                         'start.launch.py')),
        launch_arguments={'headless': 'true'}.items())

    actions = api_nodes(DOMAIN, defaults=STARTUP_PARAMETERS)
    actions.append(simulator)
    actions.append(TimerAction(
        period=CONTROLLER_DELAY,
        actions=[installed_controller(workspace, executables[0], DOMAIN)]))
    return LaunchDescription(actions + [launch_testing.actions.ReadyToTest()])


class TestDocumentedWorkflow(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.probe = Probe(name='hsm_documented_workflow_probe')
        cls.poses = cls.probe.record('/turtle1/pose', Pose)
        cls.goals = cls.probe.record('/goal_pose', PoseStamped)

    @classmethod
    def tearDownClass(cls):
        cls.probe.destroy_node()
        rclpy.shutdown()
        shutil.rmtree(getattr(generate_test_description, 'workspace', ''), ignore_errors=True)

    def test_the_generated_package_builds(self):
        # the build is done while the description is made, so reaching this point at all
        # means it succeeded; what is checked here is that it installed something to run
        assert generate_test_description.executables, \
            'the build installed no executable to run'

    def test_the_executable_is_named_after_the_state_machine(self):
        # the name a user types after ros2 run: it comes from the state machine of the
        # diagram and not from the directory the package was generated into
        name = os.path.basename(generate_test_description.executables[0])
        assert name == '{}_controller'.format(generate_test_description.module), \
            'the build installed {} for the module {}'.format(
                name, generate_test_description.module)

    def test_the_package_is_named_hsm_controller_whatever_the_directory_is(self):
        # this is the constraint which makes every other tier run the controller out of a
        # temporary directory: the name is in the generated manifest, so two diagrams
        # cannot be built into one workspace
        manifest = os.path.join(generate_test_description.workspace, 'src', 'hsm_controller',
                                'package.xml')
        with open(manifest) as source:
            assert '<name>hsm_controller</name>' in source.read(), \
                'the generated package is no longer named hsm_controller'

    def test_the_shipped_launch_file_starts_the_simulator_without_a_display(self):
        # the launch file of the driver is the one a user runs, and it starts the
        # simulator, which is a Qt program and refuses to start without a display unless
        # it is asked for the offscreen platform. There is no display here
        assert self.probe.wait_for(lambda: len(self.poses) > 0, timeout=STARTUP), \
            'the launch file did not bring the simulator up'

    def test_the_installed_controller_drives_the_turtle(self):
        # the whole point: the package which was built a moment ago drives the simulator
        assert self.probe.wait_for(lambda: len(self.goals) >= 1, timeout=STARTUP), \
            'the installed controller asked for no point'
        target = self.goals[0].pose.position
        assert self.probe.wait_for(
            lambda: any(abs(p.x - target.x) < 0.1 and abs(p.y - target.y) < 0.1
                        for p in self.poses), timeout=STARTUP), \
            'the turtle never reached the point the installed controller sent it to'
