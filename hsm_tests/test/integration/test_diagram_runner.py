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
EXIT_FAILED = 1
EXIT_FRAMEWORK = 4

# the second state divides by zero on the first tick: the dispatcher stops the machine
RAISING_DIAGRAM = """<?xml version="1.0" encoding="UTF-8"?>
<graphml xmlns="http://graphml.graphdrawing.org/xmlns">
  <data key="gFormat">Cyberiada-GraphML-1.0</data>
  <key id="gFormat" for="graphml" attr.name="format" attr.type="string"/>
  <key id="dName" for="graph" attr.name="name" attr.type="string"/>
  <key id="dName" for="node" attr.name="name" attr.type="string"/>
  <key id="dStateMachine" for="graph" attr.name="stateMachine" attr.type="string"/>
  <key id="dNote" for="node" attr.name="note" attr.type="string"/>
  <key id="dVertex" for="node" attr.name="vertex" attr.type="string"/>
  <key id="dData" for="node" attr.name="data" attr.type="string"/>
  <key id="dData" for="edge" attr.name="data" attr.type="string"/>
  <graph id="G" edgedefault="directed">
    <data key="dStateMachine"/>
    <data key="dName">raising</data>
    <node id="nMeta">
      <data key="dNote">formal</data>
      <data key="dName">CGML_META</data>
      <data key="dData">standardVersion/ 1.0

name/ raising
</data>
    </node>
    <node id="nModules">
      <data key="dNote">informal</data>
      <data key="dData">ROS2 HSM Modules

Debug
Timer</data>
    </node>
    <node id="nInit">
      <data key="dVertex">initial</data>
    </node>
    <node id="nA">
      <data key="dName">first</data>
    </node>
    <node id="nB">
      <data key="dName">second</data>
      <data key="dData">entry/
Debug.println(1 / 0)</data>
    </node>
    <edge id="e0" source="nInit" target="nA"/>
    <edge id="e1" source="nA" target="nB">
      <data key="dData">TIMER_TICK/</data>
    </edge>
  </graph>
</graphml>
"""

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


def run(path, domain, findings, env=None):
    return subprocess.run([runner(), path, '--domain', domain, '--json', str(findings)],
                          capture_output=True, text=True, timeout=RUN_LIMIT, env=env)


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


def test_error_of_the_diagram_code_fails_the_run(tmp_path):
    # the dispatcher stops the machine and the controller exits with an error, which a
    # smoke run - a diagram without rules - has to report
    path = tmp_path / 'raising.graphml'
    path.write_text(RAISING_DIAGRAM, encoding='utf-8')
    result = run(str(path), PASSING_DOMAIN, tmp_path / 'findings.json')
    assert result.returncode == EXIT_FAILED, result.stdout[-2000:] + result.stderr[-2000:]
    findings = json.loads((tmp_path / 'findings.json').read_text())
    assert [f['rule'] for f in findings] == ['test-controller']
    assert 'The state machine is stopped' in findings[0]['message']


def test_missing_dispatcher_is_named(tmp_path):
    # an hsmd which does not import stands for a system without the dispatcher
    shadow = tmp_path / 'shadow'
    shadow.mkdir()
    (shadow / 'hsmd.py').write_text('raise ImportError("no hsmd here")\n')
    env = dict(os.environ)
    env['PYTHONPATH'] = str(shadow) + os.pathsep + env.get('PYTHONPATH', '')
    path = diagram_with_rules(tmp_path, 'MOVE_COMPLETED')
    result = run(path, PASSING_DOMAIN, tmp_path / 'findings.json', env)
    assert result.returncode == EXIT_FRAMEWORK, result.stdout[-2000:] + result.stderr[-2000:]
    assert 'hsmd' in result.stderr and 'not installed' in result.stderr
    assert processes_in_domain(PASSING_DOMAIN) == []
