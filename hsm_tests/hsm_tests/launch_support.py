# -----------------------------------------------------------------------------
# The Cyberiada HSM-to-ROS2 testing environment
# -----------------------------------------------------------------------------
#
# The launch descriptions of the integration tests
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

# The integration tests run the framework the way it is run for real: the module nodes of
# the API, the robot and the controller generated from a diagram are separate processes,
# and the test watches them from the outside. The controller cannot be built into the
# workspace, because a generated package is always named hsm_controller and only one of
# them could exist at a time; it is generated into a temporary directory instead and run
# from there, which also lets a tier use several diagrams.
#
# The timers run in real time, with the periods shortened by the parameters of the nodes -
# the same model the node tests use.

import os
import shutil
import subprocess
import sys
import tempfile

from ament_index_python.packages import get_package_share_directory
from launch.actions import ExecuteProcess, TimerAction
from launch_ros.actions import Node

# the module nodes of the API and the executables which start them
API_NODES = ('debug_node', 'timer_node', 'wheels_node', 'navigation_node',
             'pump_node', 'storage_node')

# the periods which make a test finish in a moment instead of a minute
FAST_PARAMETERS = {
    'tick_period': 0.02,
    'second_period': 0.05,
    'minute_period': 0.1,
    'control_period': 0.02,
    'scan_period': 0.05,
    # the controller checks for a module service often, but waits long enough for the
    # whole framework to come up: a cold start brings eight processes up at once
    'service_startup_timeout': 0.2,
    'service_startup_limit': 60.0,
}


def diagram(name):
    # the example diagrams are installed with the generator
    return os.path.join(get_package_share_directory('hsm_generator'), 'examples',
                        '{}.graphml'.format(name))


def generate_controller(diagram_path):
    # convert a diagram into a package in a temporary directory and return the directory
    # and the name of the module which holds the controller
    directory = tempfile.mkdtemp(prefix='hsm_integration_')
    result = subprocess.run(
        [sys.executable, '-m', 'hsm_generator', diagram_path, '-o', directory, '-q'],
        capture_output=True, text=True)
    if result.returncode != 0:
        shutil.rmtree(directory, ignore_errors=True)
        raise RuntimeError('generating {} failed: {}'.format(diagram_path, result.stderr))
    library = os.listdir(os.path.join(get_package_share_directory('hsm_generator'),
                                      'hsm_controller'))
    modules = [f[:-3] for f in os.listdir(os.path.join(directory, 'hsm_controller'))
               if f.endswith('.py') and f not in library]
    assert len(modules) == 1, 'expected one generated module, got {}'.format(modules)
    return directory, modules[0]


def isolate(domain):
    # every launch description gets a domain of its own: the descriptions of a tier run
    # one after another in the same session, and the nodes of the previous one are not
    # always gone when the next one starts - two robots publishing the odometry make the
    # events of the modules meaningless. It also keeps a graph the developer is running
    # out of the tests
    environment = dict(os.environ)
    environment['ROS_DOMAIN_ID'] = str(domain)
    environment['ROS_AUTOMATIC_DISCOVERY_RANGE'] = 'LOCALHOST'
    return environment


def use_domain(domain):
    # the test process joins the domain of the nodes it watches; it has to be called
    # before the ROS2 context of the test is created
    os.environ['ROS_DOMAIN_ID'] = str(domain)
    os.environ['ROS_AUTOMATIC_DISCOVERY_RANGE'] = 'LOCALHOST'


def api_nodes(domain, **parameters):
    # the six module nodes of the API, with the periods a test needs
    values = dict(FAST_PARAMETERS)
    values.update(parameters)
    return [Node(package='hsm_robot', executable=executable, output='screen',
                 parameters=[values], additional_env=isolate(domain))
            for executable in API_NODES]


def fake_robot(domain, **parameters):
    values = dict(FAST_PARAMETERS)
    values.update(parameters)
    return Node(package='hsm_tests', executable='fake_robot', output='screen',
                parameters=[values], additional_env=isolate(domain))


def controller(directory, module, domain, **parameters):
    # the generated package is not installed, so it is run from where it was generated
    environment = isolate(domain)
    environment['PYTHONPATH'] = directory + os.pathsep + environment.get('PYTHONPATH', '')
    arguments = ['--ros-args']
    values = dict(FAST_PARAMETERS)
    values.update(parameters)
    for name, value in values.items():
        arguments += ['-p', '{}:={}'.format(name, repr(float(value))
                                            if isinstance(value, float) else value)]
    return ExecuteProcess(
        cmd=[sys.executable, '-m', 'hsm_controller.{}'.format(module)] + arguments,
        additional_env=environment, output='screen')


# the controller is started after the rest of the framework, the way a robot is started
# before the program which drives it. The modules publish a velocity only when the
# diagram calls them, so a command given before the robot is listening is lost and the
# robot never moves - a diagram waiting for the movement to end then waits forever
CONTROLLER_DELAY = 3.0


def framework(diagram_name, domain, **parameters):
    # everything a diagram needs to run: the API, the robot and the controller, in a
    # domain of their own
    directory, module = generate_controller(diagram(diagram_name))
    actions = api_nodes(domain, **parameters)
    actions.append(fake_robot(domain, **parameters))
    actions.append(TimerAction(period=CONTROLLER_DELAY,
                               actions=[controller(directory, module, domain, **parameters)]))
    return actions, directory
