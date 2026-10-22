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

# The integration and system tests run the framework the way it is run for real: the module
# nodes of the API, the robot and the controller generated from a diagram are separate
# processes, and the test watches them from the outside. The controller cannot be built into
# the workspace, because a generated package is always named hsm_controller and only one of
# them could exist at a time; it is generated into a temporary directory instead and run
# from there, which also lets a tier use several diagrams.
#
# The timers run in real time. The integration tier shortens the periods with the parameters
# of the nodes - the same model the node tests use - and the system tier does not, because
# its clock is the simulator and a parameter cannot shorten that.

import glob
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

# The controller checks for a module service often, but waits long enough for the whole
# framework to come up: a cold start brings eight processes up at once. This is the
# handshake before anything runs, so every tier shortens it.
STARTUP_PARAMETERS = {
    'service_startup_timeout': 0.2,
    'service_startup_limit': 60.0,
}

# the periods which make a test finish in a moment instead of a minute
FAST_PARAMETERS = dict(
    STARTUP_PARAMETERS,
    tick_period=0.02,
    second_period=0.05,
    minute_period=0.1,
    control_period=0.02,
    scan_period=0.05,
)


def diagram(name):
    # the example diagrams are installed with the generator
    return os.path.join(get_package_share_directory('hsm_generator'), 'examples',
                        '{}.graphml'.format(name))


def generate_controller(diagram_path, directory=None, force=False):
    # convert a diagram into a package in a temporary directory (or the given one, replaced
    # with force) and return the directory and the name of the module of the controller
    temporary = directory is None
    if temporary:
        directory = tempfile.mkdtemp(prefix='hsm_integration_')
    result = subprocess.run(
        [sys.executable, '-m', 'hsm_generator', diagram_path, '-o', directory, '-q']
        + (['-f'] if force else []),
        capture_output=True, text=True)
    if result.returncode != 0:
        if temporary:
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


def api_nodes(domain, defaults=None, **parameters):
    # the six module nodes of the API, with the periods a test needs
    values = dict(FAST_PARAMETERS if defaults is None else defaults)
    values.update(parameters)
    return [Node(package='hsm_robot', executable=executable, output='screen',
                 parameters=[values], additional_env=isolate(domain))
            for executable in API_NODES]


def fake_robot(domain, **parameters):
    values = dict(FAST_PARAMETERS)
    values.update(parameters)
    return Node(package='hsm_tests', executable='fake_robot', output='screen',
                parameters=[values], additional_env=isolate(domain))


def parameter_arguments(values):
    arguments = ['--ros-args']
    for name, value in values.items():
        arguments += ['-p', '{}:={}'.format(name, repr(float(value))
                                            if isinstance(value, float) else value)]
    return arguments


def controller(directory, module, domain, defaults=None, **parameters):
    # the generated package is not installed, so it is run from where it was generated
    environment = isolate(domain)
    environment['PYTHONPATH'] = directory + os.pathsep + environment.get('PYTHONPATH', '')
    values = dict(FAST_PARAMETERS if defaults is None else defaults)
    values.update(parameters)
    return ExecuteProcess(
        cmd=[sys.executable, '-m', 'hsm_controller.{}'.format(module)]
        + parameter_arguments(values),
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


# ---------------------------------------------------------------------------------------
# The system tier: the shipped simulator instead of the fake robot
#
# Nothing here shortens the clock. The fake robot has one because its clock is a parameter;
# the simulator draws the turtle at its own rate and moves it at the speed the driver asks
# for, and the only thing a parameter could change is the speed - which is the behaviour
# the tier exists to check. A square takes the time a square takes.
# ---------------------------------------------------------------------------------------


def turtlesim(domain):
    # the simulator draws with Qt and refuses to start without a display, so the tier asks
    # for the offscreen platform itself rather than expecting it from the shell
    environment = isolate(domain)
    environment['QT_QPA_PLATFORM'] = 'offscreen'
    return Node(package='turtlesim', executable='turtlesim_node', output='screen',
                additional_env=environment)


def driver(domain, **parameters):
    # the platform adapter between the API and the simulator; its executable is 'driver'
    return Node(package='turtle_driver', executable='driver', output='screen',
                parameters=[dict(parameters)] if parameters else [],
                additional_env=isolate(domain))


def system_framework(diagram_name, domain, **parameters):
    # the same shape as framework(), with the simulator and its driver in the place of the
    # fake robot, and only the startup handshake shortened
    directory, module = generate_controller(diagram(diagram_name))
    actions = api_nodes(domain, defaults=STARTUP_PARAMETERS, **parameters)
    actions.append(turtlesim(domain))
    actions.append(driver(domain))
    actions.append(TimerAction(
        period=CONTROLLER_DELAY,
        actions=[controller(directory, module, domain, defaults=STARTUP_PARAMETERS,
                            **parameters)]))
    return actions, directory


def colcon_command(workspace):
    # colcon is told every path it uses, so it works from any directory: without the log
    # base it leaves a log directory wherever the tests happen to run, which under colcon
    # is the build directory of this package. The log base belongs to colcon itself and
    # comes before the verb
    return ['colcon', '--log-base', os.path.join(workspace, 'log'), 'build',
            '--base-paths', os.path.join(workspace, 'src'),
            '--build-base', os.path.join(workspace, 'build'),
            '--install-base', os.path.join(workspace, 'install'),
            '--packages-select', 'hsm_controller']


def installed_site_packages(workspace):
    # where the build put the modules of the package
    return glob.glob(os.path.join(workspace, 'install', 'hsm_controller', 'lib',
                                  'python3.*', 'site-packages'))


def build_controller(diagram_path):
    # The workflow the documentation gives a user: generate the diagram into a workspace
    # and build it. The workspace is its own, because a generated package is always named
    # hsm_controller whatever the output directory is called - two diagrams cannot share
    # one workspace, and building into the workspace of the developer would overwrite what
    # is already there.
    workspace = tempfile.mkdtemp(prefix='hsm_system_ws_')
    directory, module = generate_controller(diagram_path,
                                            os.path.join(workspace, 'src', 'hsm_controller'))
    result = subprocess.run(colcon_command(workspace), capture_output=True, text=True)
    if result.returncode != 0:
        raise RuntimeError('building the generated package failed: {}{}'.format(
            result.stdout, result.stderr))
    return workspace, module


def installed_executables(workspace):
    # what the build put where ros2 run would look for it
    return sorted(glob.glob(os.path.join(workspace, 'install', 'hsm_controller', 'lib',
                                         'hsm_controller', '*')))


def installed_controller(workspace, executable, domain, **parameters):
    # The built package is started by its path and not with a Node action: a Node resolves
    # the package through the index of the process which builds the description, and a
    # package built while the test runs is not in it. The modules of the package are found
    # through the path of the interpreter, so nothing has to be sourced.
    environment = isolate(domain)
    environment['PYTHONPATH'] = os.pathsep.join(
        installed_site_packages(workspace) + [environment.get('PYTHONPATH', '')])
    values = dict(STARTUP_PARAMETERS)
    values.update(parameters)
    return ExecuteProcess(cmd=[executable] + parameter_arguments(values),
                          additional_env=environment, output='screen')
