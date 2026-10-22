# -----------------------------------------------------------------------------
# The Cyberiada HSM-to-ROS2 testing environment
# -----------------------------------------------------------------------------
#
# The turtle checks and the trace of the turtle runner
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

import math
import os
import types

import gencode
import pytest

from hsm_tests import turtle_rules, turtle_trace
from hsm_tests.launch_support import diagram


class Diagram:
    # what the rules read of a parsed diagram

    def __init__(self, triggers=(), code=(), rules=(), comment=None):
        self.__usage = (['Navigation', 'Wheels'], list(triggers), list(code))
        self.__rules = (list(rules), comment)

    def usage(self):
        return self.__usage

    def test_rules(self):
        return self.__rules


def rules_of(findings):
    return [(f['rule'], f.get('element')) for f in findings]


def test_scan_events_are_reported():
    findings = turtle_rules.findings(Diagram(triggers=[('e1', 'RIGHT_OPEN_SPACE'),
                                                       ('e2', 'MOVE_COMPLETED'),
                                                       ('nA', 'COLLISION_WARNING')]))
    assert rules_of(findings) == [('turtle-scan', 'e1'), ('turtle-scan', 'nA')]


def test_collision_detected_is_not_reported():
    # the driver reports it when the turtle makes no progress
    assert turtle_rules.findings(Diagram(triggers=[('e1', 'COLLISION_DETECTED')])) == []


def test_pump_is_reported_once_per_element():
    findings = turtle_rules.findings(Diagram(code=[('nA', 'Pump.turn_on()'),
                                                   ('nA', 'Pump.turn_off()'),
                                                   ('nB', "Debug.print('Pump.turn_on()')")]))
    assert rules_of(findings) == [('turtle-pump', 'nA')]


@pytest.mark.parametrize('code, outside', [
    ('Navigation.move_to_point(2, 7)', []),
    ('Navigation.move_to_point(12, 1.5, 0.3)', [(12.0, 1.5)]),
    ('Navigation.move_to_point(-.5, 3)', [(-0.5, 3.0)]),
    ('Navigation.move_along_traj([(1, 1), (2.5, 11.5), [20, 0, 1]])', [(2.5, 11.5), (20.0, 0.0)]),
    ('Navigation.move_along_traj(Storage.points("path"))\nx = (30, 30)', []),
    ('Navigation.move_to_point(x, 30)', []),
])
def test_points_outside_the_world_are_reported(code, outside):
    findings = turtle_rules.findings(Diagram(code=[('nA', code)]))
    assert [f['rule'] for f in findings] == ['turtle-world'] * len(outside)
    for f, (x, y) in zip(findings, outside):
        assert '({:g}, {:g})'.format(x, y) in f['message']


def test_test_rules_on_scan_events_are_reported():
    rules = gencode.parse_test_rules('RIGHT_OPEN_SPACE\nnever COLLISION_WARNING\nMOVE_COMPLETED')
    findings = turtle_rules.findings(Diagram(rules=rules, comment='nTest'))
    assert [(f['severity'], f['rule'], f['element']) for f in findings] == [
        ('info', 'turtle-test', 'nTest')]


def test_shipped_diagrams():
    findings = turtle_rules.findings(gencode.CodeGenerator(diagram('right_hand_maze_solver')))
    assert {f['rule'] for f in findings} == {'turtle-scan'}
    assert turtle_rules.findings(gencode.CodeGenerator(diagram('turtle-square'))) == []


def pose(x, y, theta=0.0):
    return types.SimpleNamespace(x=x, y=y, theta=theta)


def test_trace_keeps_the_moves_after_start():
    trace = turtle_trace.Trace()
    trace.add(pose(1.0, 1.0))
    trace.start()
    for p in (pose(1.0, 1.0), pose(1.001, 1.0), pose(1.1, 1.0), pose(1.1, 1.0, 0.5)):
        trace.add(p)
    assert [(x, y, theta) for _, x, y, theta in trace.points] == [
        (1.0, 1.0, 0.0), (1.1, 1.0, 0.0), (1.1, 1.0, 0.5)]


def test_csv_and_svg():
    points = [(0.0, 2.0, 2.0, 0.0), (1.5, 7.0, 2.0, math.pi / 2)]
    assert turtle_trace.csv(points).splitlines() == [
        't,x,y,theta', '0.000,2.0000,2.0000,0.0000', '1.500,7.0000,2.0000,1.5708']
    picture = turtle_trace.svg(points, 'a<b')
    # the y axis of the world points up, the one of the picture down
    assert 'points="90.0,409.0 315.0,409.0"' in picture
    assert '<title>a&lt;b</title>' in picture
    assert picture.count('<polygon') == 1
    assert '<polyline' not in turtle_trace.svg([], 'empty')


def parsed(monkeypatch, *arguments):
    from hsm_tests import turtle_run
    monkeypatch.setattr('sys.argv', ['hsm_turtle_run', 'd.graphml', '--workspace', 'ws']
                        + list(arguments))
    return turtle_run.arguments()


def test_heading_in_degrees(monkeypatch):
    assert parsed(monkeypatch, '--pose', '2', '3', '90', '--degrees').pose == \
        pytest.approx((2.0, 3.0, math.pi / 2))
    assert parsed(monkeypatch, '--pose', '2', '3', '1.5').pose[2] == 1.5


@pytest.mark.parametrize('image, files', [('true', ['d.csv', 'd.svg']), ('false', ['d.csv'])])
def test_trace_image_is_optional(monkeypatch, tmp_path, image, files):
    from hsm_tests import turtle_run
    args = parsed(monkeypatch, '--trace', str(tmp_path), '--image', image)
    trace = turtle_trace.Trace()
    trace.points = [(0.0, 1.0, 1.0, 0.0)]
    turtle_run.write_trace(args, trace, types.SimpleNamespace(line=lambda text: None))
    assert sorted(os.listdir(str(tmp_path))) == files
