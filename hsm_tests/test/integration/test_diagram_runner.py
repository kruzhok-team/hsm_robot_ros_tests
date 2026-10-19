# -----------------------------------------------------------------------------
# The Cyberiada HSM-to-ROS2 testing environment
# -----------------------------------------------------------------------------
#
# The diagram test runner checks the expectations of a diagram and stops cleanly
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

# The runner is what the test action of the editor plugin starts, so it is run here the way
# the plugin runs it: the installed executable, a diagram in the Cyberiada format the editor
# hands over, and the diagnostics file it writes.

import json
import os
import signal
import subprocess
import time

from ament_index_python.packages import get_package_prefix
import CyberiadaML
import pytest

from hsm_tests.launch_support import diagram

PASSING_DOMAIN = '97'
CANCELLED_DOMAIN = '98'
RUN_LIMIT = 120
CANCEL_AFTER = 8.0
STOP_LIMIT = 15
TEST_COMMENT = 'nTestRules'

pytestmark = pytest.mark.integration


def runner():
    return os.path.join(get_package_prefix('hsm_tests'), 'lib', 'hsm_tests', 'hsm_robot_test')


def diagram_with_rules(directory, rules):
    # turtle-square in the editor's format with a 'ros2 hsm test' comment
    document = CyberiadaML.LocalDocument()
    document.open(diagram('turtle-square'), CyberiadaML.formatDetect,
                  CyberiadaML.geometryFormatNone, False, False, True)
    converted = str(directory / 'converted.graphml')
    document.save_as(converted, CyberiadaML.formatCyberiada10)
    with open(converted, encoding='utf-8') as f:
        text = f.read()
    position = text.index('<node ', text.index('<graph '))
    comment = ('<node id="{}">\n<data key="dNote">informal</data>\n'
               '<data key="dData">ROS2 HSM Test\n\n{}</data>\n</node>\n').format(TEST_COMMENT, rules)
    tested = directory / 'tested.graphml'
    tested.write_text(text[:position] + comment + text[position:], encoding='utf-8')
    return str(tested)


def run(path, domain, findings):
    return subprocess.run([runner(), path, '--domain', domain, '--json', str(findings)],
                          capture_output=True, text=True, timeout=RUN_LIMIT)


def processes_in_domain(domain):
    # the processes of this user which run in the domain
    found = []
    for pid in filter(str.isdigit, os.listdir('/proc')):
        try:
            with open('/proc/{}/environ'.format(pid), 'rb') as f:
                environment = f.read().split(b'\0')
        except OSError:
            continue
        if 'ROS_DOMAIN_ID={}'.format(domain).encode() in environment and int(pid) != os.getpid():
            found.append(pid)
    return found


def test_met_expectations_pass(tmp_path):
    path = diagram_with_rules(tmp_path, 'MOVE_COMPLETED x2 within 60\nnever COLLISION_DETECTED')
    result = run(path, PASSING_DOMAIN, tmp_path / 'findings.json')
    assert result.returncode == 0, result.stdout[-2000:] + result.stderr[-2000:]
    assert 'PASSED' in result.stdout
    assert json.loads((tmp_path / 'findings.json').read_text()) == []


def test_unmet_expectation_points_at_the_comment(tmp_path):
    path = diagram_with_rules(tmp_path, 'never MOVE_COMPLETED')
    result = run(path, PASSING_DOMAIN, tmp_path / 'findings.json')
    assert result.returncode == 1, result.stdout[-2000:] + result.stderr[-2000:]
    findings = json.loads((tmp_path / 'findings.json').read_text())
    assert [(f['rule'], f.get('element')) for f in findings] == [('test-failed', TEST_COMMENT)]
    # the failed rule stops the run at once instead of waiting for the duration
    assert 'stop; events:' in result.stdout


def test_cancel_leaves_nothing_behind(tmp_path):
    path = diagram_with_rules(tmp_path, 'MOVE_COMPLETED x100')
    process = subprocess.Popen([runner(), path, '--domain', CANCELLED_DOMAIN],
                               stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    time.sleep(CANCEL_AFTER)
    assert processes_in_domain(CANCELLED_DOMAIN), 'the framework did not start'
    process.send_signal(signal.SIGTERM)
    assert process.wait(timeout=STOP_LIMIT) == 128 + signal.SIGTERM
    assert processes_in_domain(CANCELLED_DOMAIN) == []
