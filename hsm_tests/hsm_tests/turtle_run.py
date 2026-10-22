# -----------------------------------------------------------------------------
# The Cyberiada HSM-to-ROS2 testing environment
# -----------------------------------------------------------------------------
#
# The turtle runner: one diagram drives the turtle of the simulator
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

# The runner of the hsm-turtle editor plugin (turtle_driver/docs/editor_plugin.md): the
# diagram is generated and built into a workspace, then drives turtlesim through the API
# nodes and the turtle driver, in a domain of its own, until the time is up, the controller
# ends or the run is cancelled. The pose of the turtle is traced and written as CSV and SVG
# in any case. With --check the diagram is only checked, for the turtle as well.

import argparse
import math
import os
import random
import signal
import subprocess
import sys

from ament_index_python.packages import (get_package_prefix, get_package_share_directory,
                                         PackageNotFoundError)
import gencode
import hsm_generator
import hsmplugin
from hsm_test_utils.probes import Probe
import rclpy
from std_srvs.srv import Empty
from turtlesim.msg import Pose
from turtlesim.srv import TeleportAbsolute

from hsm_tests import turtle_rules, turtle_trace
from hsm_tests.diagram_test import (Cancelled, cancel, DOMAINS, EXIT_FAILED, EXIT_FRAMEWORK,
                                    EXIT_PASSED, EXIT_SIGNAL, Expectations, Framework, Output,
                                    watch, write_findings)
from hsm_tests.launch_support import (colcon_command, CONTROLLER_DELAY, generate_controller,
                                      installed_site_packages, STARTUP_PARAMETERS, use_domain)

PLUGIN_ID = 'hsm-turtle'
TURTLE_ROBOT = (('turtlesim', 'turtlesim_node'), ('turtle_driver', 'driver'))
POSE_TOPIC = '/turtle1/pose'
TELEPORT_SERVICE = '/turtle1/teleport_absolute'
CLEAR_SERVICE = '/clear'
SERVICE_TIMEOUT = 10.0
# turtlesim puts its turtle into the middle of the world
SPAWN = (turtle_rules.WORLD_SIZE / 2, turtle_rules.WORLD_SIZE / 2, 0.0)
DEFAULT_DURATION = 60.0
TRACE_NAME = 'trace'


def arguments():
    parser = argparse.ArgumentParser(
        description='Drive the turtle of turtlesim with a diagram, or check it for the turtle.')
    parser.add_argument('diagram', nargs='?', help='the CyberiadaML diagram')
    parser.add_argument('--check', action='store_true',
                        help='check the diagram for the turtle and run nothing')
    parser.add_argument('--json', metavar='FILE',
                        help='write the findings as the diagnostics file of the editor')
    parser.add_argument('--workspace', metavar='DIR',
                        help='the colcon workspace the package is generated and built in')
    parser.add_argument('--duration', type=float, default=DEFAULT_DURATION,
                        help='the run limit, s, from the controller start; 0: until cancelled')
    parser.add_argument('--pose', type=float, nargs=3, metavar=('X', 'Y', 'THETA'),
                        default=SPAWN, help='the start pose of the turtle')
    parser.add_argument('--degrees', action='store_true',
                        help='the THETA of the pose is in degrees, not radians')
    parser.add_argument('--headless', action='store_true',
                        help='run the simulator without its window')
    parser.add_argument('--trace', metavar='DIR', help='write the trace of the turtle here')
    parser.add_argument('--name', help='the base name of the trace files')
    parser.add_argument('--image', choices=('true', 'false'), default='true',
                        help='write the SVG picture of the trace besides the CSV')
    parser.add_argument('--domain', type=int, help='the ROS domain (a random one by default)')
    parser.add_argument('--verbose', action='store_true',
                        help='show the output of the API nodes, the simulator and the driver')
    parser.add_argument('--install-editor-plugin', metavar='DIR', nargs='?', const='',
                        help='install the hsm-turtle plugin of CyberiadaPro into DIR '
                        '(the user plugin folder by default) and exit')
    args = parser.parse_args()
    if args.install_editor_plugin is not None:
        return args
    if args.diagram is None:
        parser.error('the diagram is required')
    if not args.check and args.workspace is None:
        parser.error('the workspace is required to run')
    if args.duration < 0:
        parser.error('the duration cannot be negative')
    if args.degrees:
        args.pose = (args.pose[0], args.pose[1], math.radians(args.pose[2]))
    if not turtle_rules.inside(*args.pose[:2]):
        parser.error('the start pose is outside the turtle world 0..{:.2f}'.format(
            turtle_rules.WORLD_SIZE))
    return args


def check(args, generator, output):
    findings = turtle_rules.findings(generator)
    for f in findings:
        output.line('{}: {} [{}]'.format(f['severity'], f['message'], f['rule']))
    output.line('{} finding(s)'.format(len(findings)))
    write_findings(args.json, findings)
    return EXIT_PASSED


def build(args, output):
    # generate the package into the workspace, replacing the earlier one, and build it
    workspace = os.path.abspath(args.workspace)
    package = os.path.join(workspace, 'src', 'hsm_controller')
    output.line('generating {}'.format(package))
    _, module = generate_controller(args.diagram, package, force=True)
    output.line('building {}'.format(workspace))
    result = subprocess.run(colcon_command(workspace) + ['--event-handlers', 'console_cohesion+'],
                            stdin=subprocess.DEVNULL)
    if result.returncode != 0:
        raise RuntimeError('colcon build ended with {}'.format(result.returncode))
    site_packages = installed_site_packages(workspace)
    if not site_packages:
        raise RuntimeError('the build installed no hsm_controller into {}'.format(workspace))
    return site_packages[0], module


def place(probe, pose):
    # put the turtle at the start pose and wipe the line the teleport drew
    try:
        request = TeleportAbsolute.Request(x=pose[0], y=pose[1], theta=pose[2])
        probe.call(TELEPORT_SERVICE, TeleportAbsolute, request, SERVICE_TIMEOUT)
        probe.call(CLEAR_SERVICE, Empty, None, SERVICE_TIMEOUT)
    except AssertionError as e:
        raise RuntimeError('the simulator does not answer: {}'.format(e)) from e


def write_trace(args, trace, output):
    if args.trace is None or not trace.points:
        return
    name = args.name or os.path.splitext(os.path.basename(args.diagram))[0] or TRACE_NAME
    os.makedirs(args.trace, exist_ok=True)
    files = [('csv', turtle_trace.csv(trace.points))]
    if args.image == 'true':
        files.append(('svg', turtle_trace.svg(trace.points, name)))
    for extension, text in files:
        path = os.path.join(args.trace, '{}.{}'.format(name, extension))
        with open(path, 'w', encoding='utf-8') as out:
            out.write(text)
        output.line('the trace: {}'.format(path))


def drive(args, directory, module, output):
    domain = args.domain if args.domain is not None else random.randint(*DOMAINS)
    duration = args.duration or math.inf
    framework = Framework(domain, output, args.verbose)
    trace = turtle_trace.Trace()
    use_domain(domain)
    rclpy.init()
    probe = None
    try:
        framework.check_dispatcher(directory)
        output.line('domain {}, {}'.format(domain, 'until cancelled' if duration == math.inf
                                           else 'at most {:g} s'.format(duration)))
        environment = framework.environment()
        if args.headless:
            environment['QT_QPA_PLATFORM'] = 'offscreen'
        framework.start_nodes(STARTUP_PARAMETERS, {}, TURTLE_ROBOT, environment)
        probe = Probe(name='hsm_turtle_run')
        probe.create_subscription(Pose, POSE_TOPIC, trace.add, 100)
        probe.spin(CONTROLLER_DELAY)
        failed = framework.failed_node()
        if failed is not None:
            raise RuntimeError('the node {} ended with {}'.format(*failed))
        place(probe, args.pose)
        # the events before the controller cannot come from the diagram
        probe.clear()
        trace.start()
        framework.start_controller(directory, module, STARTUP_PARAMETERS)
        code = watch(framework, probe, Expectations([], duration), output, duration)
        output.mute()
        if code not in (None, 0):
            reason = output.errors.get('controller')
            output.line('FAILED: the controller ended with {}{}'.format(
                code, ': ' + reason if reason else ''))
            return EXIT_FAILED
        return EXIT_PASSED
    finally:
        output.mute()
        framework.stop()
        if probe is not None:
            probe.destroy_node()
        rclpy.try_shutdown()
        if trace.points:
            _, x, y, theta = trace.points[-1]
            output.line('the turtle ended at ({:.2f}, {:.2f}, {:.2f})'.format(x, y, theta))
        write_trace(args, trace, output)


def run(args, output):
    try:
        generator = gencode.CodeGenerator(args.diagram)
    except gencode.ConvertorError as e:
        output.line('the diagram is rejected: {}'.format(str(e).strip()))
        if args.json is not None:
            hsm_generator.write_diagnostics(args.json, args.diagram, e)
        return EXIT_FAILED
    if args.check:
        return check(args, generator, output)
    directory, module = build(args, output)
    return drive(args, directory, module, output)


def install_editor_plugin(target):
    # the plugin is shipped with the turtle driver; it runs this runner and the generator
    try:
        origin = os.path.join(get_package_share_directory('turtle_driver'), 'editor-plugin')
        generator = os.path.join(get_package_prefix('hsm_generator'), 'lib', 'hsm_generator',
                                 'hsm_generator')
        folder = hsmplugin.install(origin, generator if os.path.isfile(generator) else None,
                                   target or None, plugin_id=PLUGIN_ID)
    except (PackageNotFoundError, hsmplugin.PluginError) as e:
        sys.stderr.write('Editor plugin error: {}\n'.format(e))
        return 2
    print('Installed the editor plugin into {}'.format(folder))
    return 0


def main():
    args = arguments()
    if args.install_editor_plugin is not None:
        sys.exit(install_editor_plugin(args.install_editor_plugin))
    output = Output()
    signal.signal(signal.SIGTERM, cancel)
    signal.signal(signal.SIGINT, cancel)
    try:
        sys.exit(run(args, output))
    except Cancelled as e:
        output.line('cancelled')
        sys.exit(EXIT_SIGNAL + e.args[0])
    except RuntimeError as e:
        sys.stderr.write('hsm_turtle_run: {}\n'.format(e))
        sys.exit(EXIT_FRAMEWORK)


if __name__ == '__main__':
    main()
