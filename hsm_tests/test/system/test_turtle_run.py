# -----------------------------------------------------------------------------
# The Cyberiada HSM-to-ROS2 testing environment
# -----------------------------------------------------------------------------
#
# The turtle runner of the hsm-turtle editor plugin against the simulator
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

# The runner is what the run and check actions of the hsm-turtle plugin start, so it is run
# here the way the plugin runs it: the installed executable, a workspace to build in, and
# the trace folder it fills. The trace is the turtle's path as the simulator reports it.

import csv
import json
import math
import os
import signal
import subprocess
import time

from ament_index_python.packages import get_package_prefix
import pytest

from hsm_tests.launch_support import diagram

RUN_DOMAIN = '104'
CANCELLED_DOMAIN = '105'
RUN_LIMIT = 150
STOP_LIMIT = 15
DURATION = '40'
CORNERS = ((2.0, 7.0), (7.0, 7.0), (7.0, 2.0))
NEAR = 0.3
EXIT_CANCELLED = 128 + signal.SIGTERM


def runner(*arguments):
    program = os.path.join(get_package_prefix('hsm_tests'), 'lib', 'hsm_tests', 'hsm_turtle_run')
    return [program] + list(arguments)


def trace_points(folder):
    with open(os.path.join(str(folder), 'turtle-square.csv'), encoding='utf-8') as source:
        return [(float(r['x']), float(r['y'])) for r in csv.DictReader(source)]


def test_check_reports_what_the_turtle_cannot_do(tmp_path):
    findings = tmp_path / 'diagnostics.json'
    result = subprocess.run(runner(diagram('right_hand_maze_solver'), '--check',
                                   '--json', str(findings)),
                            capture_output=True, text=True, timeout=RUN_LIMIT)
    assert result.returncode == 0, result.stderr
    assert {f['rule'] for f in json.loads(findings.read_text())} == {'turtle-scan'}


def test_run_drives_the_turtle_and_traces_it(tmp_path):
    result = subprocess.run(
        runner(diagram('turtle-square'), '--workspace', str(tmp_path / 'ws'),
               '--trace', str(tmp_path / 'trace'), '--duration', DURATION,
               '--pose', '2', '2', '0', '--headless', '--domain', RUN_DOMAIN),
        capture_output=True, text=True, timeout=RUN_LIMIT)
    assert result.returncode == 0, result.stdout + result.stderr
    assert 'MOVE_COMPLETED' in result.stdout
    points = trace_points(tmp_path / 'trace')
    assert points[0] == pytest.approx((2.0, 2.0), abs=0.05)
    for corner in CORNERS:
        assert any(math.dist(corner, p) < NEAR for p in points), \
            'the turtle never came to {}'.format(corner)
    assert (tmp_path / 'trace' / 'turtle-square.svg').read_text().startswith('<svg')


def test_cancel_stops_the_run_and_keeps_the_trace(tmp_path):
    # a run without a limit, as the plugin starts it with the duration 0, ends on SIGTERM
    process = subprocess.Popen(
        runner(diagram('turtle-square'), '--workspace', str(tmp_path / 'ws'),
               '--trace', str(tmp_path / 'trace'), '--duration', '0', '--headless',
               '--domain', CANCELLED_DOMAIN),
        stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
    deadline = time.monotonic() + RUN_LIMIT
    for line in process.stdout:
        if 'MOVE_COMPLETED' in line or time.monotonic() > deadline:
            break
    time.sleep(1.0)
    process.send_signal(signal.SIGTERM)
    output = process.communicate(timeout=STOP_LIMIT)[0]
    assert process.returncode == EXIT_CANCELLED, output
    assert 'cancelled' in output
    assert len(trace_points(tmp_path / 'trace')) > 1
