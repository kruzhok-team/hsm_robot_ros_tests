# -----------------------------------------------------------------------------
# The Cyberiada HSM-to-ROS2 testing environment
# -----------------------------------------------------------------------------
#
# The diagram test runner: one diagram against the fake robot
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

# The integration framework of launch_support without pytest and launch: the processes are
# started here, one session each, so a cancelled run - the Cancel of the editor plugin is a
# SIGTERM - stops every node and leaves nothing behind in the domain. The expectations are
# the rules of the diagram's 'ros2 hsm test' comment (docs/hsm_robot_testing.md, section 10).

import argparse
import json
import os
import random
import shutil
import signal
import subprocess
import sys
import threading
import time

from ament_index_python.packages import get_package_prefix
import gencode
import hsm_generator
from hsm_test_utils.probes import Probe
from nav_msgs.msg import Odometry
import rclpy

from hsm_tests.launch_support import (API_NODES, CONTROLLER_DELAY, FAST_PARAMETERS,
                                      STARTUP_PARAMETERS, generate_controller, isolate,
                                      parameter_arguments, use_domain)

EXIT_PASSED = 0
EXIT_FAILED = 1
EXIT_FRAMEWORK = 4
EXIT_SIGNAL = 128

# the domains of the test tiers are below, so a run never meets a test
DOMAINS = (150, 199)
DEFAULT_DURATION = 30.0
FREE_SPACE = 10.0
STOP_GRACE = 3.0
SPIN_STEP = 0.05
ODOM_TOPIC = '/odom'
# the scan events and the ticks come with every period: a repeat this close to the
# previous one is counted but not shown
REPEAT_GAP = 1.0
# the rclpy log line of an error: [ERROR] [time] [node]: message
ERROR_MARK = '[ERROR]'
ERROR_SEPARATOR = ']: '
EVENT_NAMES = {code: name for events in gencode.HSM_EVENTS.values()
               for code, name in events.items()}


class Cancelled(Exception):
    pass


def cancel(signum, frame):
    raise Cancelled(signum)


class Output:
    # the lines of the processes and of the runner, one at a time

    def __init__(self):
        self.__lock = threading.Lock()
        self.__muted = False
        # the last error a process logged, by its prefix
        self.errors = {}

    def line(self, text):
        with self.__lock:
            print(text, flush=True)

    def mute(self):
        # what the processes print while they are being stopped is not a part of the run
        self.__muted = True

    def follow(self, process, prefix, shown):
        def read():
            for line in process.stdout:
                if self.__muted:
                    continue
                if ERROR_MARK in line:
                    self.errors[prefix] = line.rstrip().split(ERROR_SEPARATOR, 1)[-1]
                if shown:
                    self.line('{}: {}'.format(prefix, line.rstrip()))
        threading.Thread(target=read, daemon=True).start()


class Framework:
    # the API nodes, the fake robot and the controller, each in a session of its own

    def __init__(self, domain, output, verbose):
        self.__environment = isolate(domain)
        self.__output = output
        self.__verbose = verbose
        self.nodes = []
        self.controller = None

    def __start(self, command, name, shown, environment=None):
        process = subprocess.Popen(command, env=environment or self.__environment,
                                   stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                                   stdin=subprocess.DEVNULL, text=True, start_new_session=True)
        self.__output.follow(process, name, shown)
        return process

    def start_nodes(self, values, robot_values):
        for package, executables, extra in (('hsm_robot', API_NODES, {}),
                                            ('hsm_tests', ('fake_robot',), robot_values)):
            for executable in executables:
                program = os.path.join(get_package_prefix(package), 'lib', package, executable)
                parameters = dict(values, **extra)
                self.nodes.append((executable, self.__start(
                    [program] + parameter_arguments(parameters), executable, self.__verbose)))

    def controller_environment(self, directory):
        environment = dict(self.__environment)
        environment['PYTHONPATH'] = directory + os.pathsep + environment.get('PYTHONPATH', '')
        return environment

    def check_dispatcher(self, directory):
        # the controller runs on hsmd: without it the controller dies on its first line,
        # which would be reported as a failed diagram
        result = subprocess.run([sys.executable, '-c', 'import hsmd'],
                                env=self.controller_environment(directory),
                                capture_output=True, text=True)
        if result.returncode != 0:
            raise RuntimeError('the HSM dispatcher hsmd (hsm-dispatcher-py) is not installed: {}'
                               .format((result.stderr.strip().splitlines() or [''])[-1]))

    def start_controller(self, directory, module, values):
        self.controller = self.__start(
            [sys.executable, '-m', 'hsm_controller.{}'.format(module)]
            + parameter_arguments(values), 'controller', True,
            self.controller_environment(directory))

    def failed_node(self):
        for name, process in self.nodes:
            if process.poll() is not None:
                return name, process.returncode
        return None

    def stop(self):
        processes = [p for _, p in self.nodes] + ([self.controller] if self.controller else [])
        for signum in (signal.SIGTERM, signal.SIGKILL):
            for process in processes:
                try:
                    os.killpg(process.pid, signum)
                except (ProcessLookupError, PermissionError):
                    pass
            deadline = time.monotonic() + STOP_GRACE
            for process in processes:
                try:
                    process.wait(timeout=max(0.0, deadline - time.monotonic()))
                except subprocess.TimeoutExpired:
                    pass
            if all(p.poll() is not None for p in processes):
                break


class Expectations:
    # the rules of the diagram against the events seen, by the time since the controller start

    def __init__(self, rules, duration):
        self.rules = rules
        self.duration = duration
        self.seen = {}

    def record(self, name, moment):
        self.seen.setdefault(name, []).append(moment)

    def __count(self, rule):
        limit = self.duration if rule.within is None else rule.within
        return len([t for t in self.seen.get(rule.event, ()) if t <= limit])

    def holds(self, rule):
        if rule.never:
            return rule.event not in self.seen
        return self.__count(rule) >= rule.count

    def settled(self, now):
        # every positive rule holds, or a rule can no longer hold
        positive = [r for r in self.rules if not r.never]
        if any(r.never and not self.holds(r) for r in self.rules):
            return True
        if any(not self.holds(r) and r.within is not None and now > r.within for r in positive):
            return True
        return bool(positive) and all(self.holds(r) for r in positive)

    def failed(self):
        return [r for r in self.rules if not self.holds(r)]

    def report(self, rule):
        return '{} - seen {} time(s)'.format(rule.line, len(self.seen.get(rule.event, ())))


def arguments():
    parser = argparse.ArgumentParser(
        description='Run a diagram against the fake robot and check its ros2 hsm test rules.')
    parser.add_argument('diagram', help='the CyberiadaML diagram')
    parser.add_argument('--duration', type=float, default=DEFAULT_DURATION,
                        help='the run limit, s, from the controller start')
    parser.add_argument('--pose', type=float, nargs=3, metavar=('X', 'Y', 'THETA'),
                        default=(0.0, 0.0, 0.0), help='the start pose of the robot')
    parser.add_argument('--scan', type=float, nargs=3, metavar=('FORWARD', 'RIGHT', 'ELSEWHERE'),
                        default=(FREE_SPACE,) * 3, help='the distances the scan reports, m')
    parser.add_argument('--real-time', action='store_true',
                        help='run the timers at their real periods')
    parser.add_argument('--domain', type=int, help='the ROS domain (a random one by default)')
    parser.add_argument('--json', metavar='FILE',
                        help='write the failures as the diagnostics file of the editor')
    parser.add_argument('--verbose', action='store_true',
                        help='show the output of the API nodes and the fake robot')
    args = parser.parse_args()
    if args.duration <= 0:
        parser.error('the duration has to be positive')
    return args


def write_findings(path, findings):
    if path is not None:
        with open(path, 'w', encoding='utf-8') as out:
            json.dump(findings, out, indent=1, ensure_ascii=False)


def watch(framework, probe, expectations, output, duration):
    # spin the probe until the rules settle, the time is up or the controller ends;
    # return the exit code of the controller, or None while it runs
    start = time.monotonic()
    events, str_events = 0, 0
    while True:
        probe.spin(SPIN_STEP)
        now = time.monotonic() - start
        new = [(m, None) for m in probe.events[events:]] + \
            [(m, m.arg) for m in probe.str_events[str_events:]]
        events, str_events = len(probe.events), len(probe.str_events)
        for message, argument in new:
            name = EVENT_NAMES.get(message.code, 'code {}'.format(message.code))
            previous = expectations.seen.get(name, [None])[-1]
            expectations.record(name, now)
            if previous is not None and now - previous < REPEAT_GAP:
                continue
            output.line('{:8.2f} s  {}{}'.format(now, name,
                                                 ' ({})'.format(argument) if argument else ''))
        failed = framework.failed_node()
        if failed is not None:
            raise RuntimeError('the node {} ended with {}'.format(*failed))
        code = framework.controller.poll()
        if code is not None or now >= duration or expectations.settled(now):
            counts = ', '.join('{} x{}'.format(n, len(t)) for n, t in sorted(expectations.seen.items()))
            output.line('{:8.2f} s  stop; events: {}'.format(now, counts or 'none'))
            return code


def run(args, output):
    try:
        rules, comment = gencode.CodeGenerator(args.diagram).test_rules()
    except gencode.ConvertorError as e:
        output.line('the diagram is rejected: {}'.format(str(e).strip()))
        if args.json is not None:
            hsm_generator.write_diagnostics(args.json, args.diagram, e)
        return EXIT_FAILED
    directory, module = generate_controller(args.diagram)
    domain = args.domain if args.domain is not None else random.randint(*DOMAINS)
    values = dict(STARTUP_PARAMETERS if args.real_time else FAST_PARAMETERS)
    robot = {'initial_x': args.pose[0], 'initial_y': args.pose[1], 'initial_theta': args.pose[2],
             'scan_forward': args.scan[0], 'scan_right': args.scan[1],
             'scan_elsewhere': args.scan[2]}
    framework = Framework(domain, output, args.verbose)
    expectations = Expectations(rules, args.duration)
    use_domain(domain)
    rclpy.init()
    probe = None
    try:
        framework.check_dispatcher(directory)
        output.line('domain {}, {} rule(s), at most {:g} s'.format(domain, len(rules),
                                                                   args.duration))
        framework.start_nodes(values, robot)
        probe = Probe(name='hsm_robot_test')
        odometry = probe.record(ODOM_TOPIC, Odometry)
        probe.spin(CONTROLLER_DELAY)
        failed = framework.failed_node()
        if failed is not None:
            raise RuntimeError('the node {} ended with {}'.format(*failed))
        # the events before the controller cannot come from the diagram
        probe.clear()
        framework.start_controller(directory, module, values)
        code = watch(framework, probe, expectations, output, args.duration)
        output.mute()
        framework.stop()
        findings = []
        if code not in (None, 0):
            # the dispatcher logs why it stopped the machine before the controller exits
            reason = output.errors.get('controller')
            message = 'The controller ended with {}{}'.format(code, ': ' + reason if reason else '')
            output.line('FAILED: ' + message)
            findings.append({'severity': 'error', 'rule': 'test-controller', 'message': message})
        for rule in rules:
            holds = expectations.holds(rule)
            output.line('{}: {}'.format('ok' if holds else 'FAILED', expectations.report(rule)))
            if not holds:
                finding = {'severity': 'error', 'rule': 'test-failed',
                           'message': 'Not met: {}'.format(expectations.report(rule))}
                if comment:
                    finding['element'] = comment
                findings.append(finding)
        if odometry:
            pose = odometry[-1].pose.pose.position
            output.line('the robot ended at ({:.2f}, {:.2f})'.format(pose.x, pose.y))
        output.line('PASSED' if not findings else 'FAILED')
        write_findings(args.json, findings)
        return EXIT_PASSED if not findings else EXIT_FAILED
    finally:
        framework.stop()
        if probe is not None:
            probe.destroy_node()
        rclpy.try_shutdown()
        shutil.rmtree(directory, ignore_errors=True)


def main():
    args = arguments()
    output = Output()
    signal.signal(signal.SIGTERM, cancel)
    signal.signal(signal.SIGINT, cancel)
    try:
        sys.exit(run(args, output))
    except Cancelled as e:
        output.line('cancelled')
        sys.exit(EXIT_SIGNAL + e.args[0])
    except RuntimeError as e:
        sys.stderr.write('hsm_robot_test: {}\n'.format(e))
        sys.exit(EXIT_FRAMEWORK)


if __name__ == '__main__':
    main()
