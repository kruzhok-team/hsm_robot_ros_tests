# -----------------------------------------------------------------------------
# The Cyberiada HSM-to-ROS2 testing environment
# -----------------------------------------------------------------------------
#
# The checks of a diagram for the turtle of the simulator
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

# What the generator accepts but the turtle cannot do: the findings are warnings, the
# diagram still runs. The turtle driver publishes no laser scan, so the events of the scan
# never come, and nothing of the turtle reacts to the pump.

import re

import gencode

# the world of turtlesim, m: its window is 500 px at 45 px/m
WORLD_SIZE = 11.088889
# the events only the scan of the navigation node raises
SCAN_EVENTS = ('COLLISION_WARNING', 'RIGHT_OPEN_SPACE')
PUMP_RE = re.compile(r'\bPump\s*\.')
NUMBER = r'-?(?:\d+(?:\.\d*)?|\.\d+)'
MOVE_TO_RE = re.compile(r'\bNavigation\s*\.\s*move_to_point\s*\(\s*({0})\s*,\s*({0})\s*[,)]'
                        .format(NUMBER))
TRAJ_RE = re.compile(r'\bNavigation\s*\.\s*move_along_traj\s*\(')
POINT_RE = re.compile(r'[(\[]\s*({0})\s*,\s*({0})\s*[,)\]]'.format(NUMBER))


def finding(severity, rule, message, element):
    entry = {'severity': severity, 'rule': rule, 'message': message}
    if element:
        entry['element'] = element
    return entry


def argument(text, start):
    # the text of the call argument opened just before start, up to its closing bracket
    depth = 1
    for position in range(start, len(text)):
        if text[position] in '([':
            depth += 1
        elif text[position] in ')]':
            depth -= 1
            if depth == 0:
                return text[start:position]
    return text[start:]


def literal_points(text):
    # the (x, y) numbers written into the movement calls
    points = [m.groups() for m in MOVE_TO_RE.finditer(text)]
    for match in TRAJ_RE.finditer(text):
        points += POINT_RE.findall(argument(text, match.end()))
    return [(float(x), float(y)) for x, y in points]


def inside(x, y):
    return 0.0 <= x <= WORLD_SIZE and 0.0 <= y <= WORLD_SIZE


def findings(generator):
    # the findings of a parsed diagram (gencode.CodeGenerator), in the editor's format
    _, triggers, code = generator.usage()
    rules, comment = generator.test_rules()
    result = []
    for element, event in triggers:
        if event in SCAN_EVENTS:
            result.append(finding('warning', 'turtle-scan',
                                  'The turtle has no laser scan: {} never comes'.format(event),
                                  element))
    pumped = set()
    for element, text in code:
        text = gencode.STRING_LITERAL_RE.sub('', text)
        if PUMP_RE.search(text) and element not in pumped:
            pumped.add(element)
            result.append(finding('warning', 'turtle-pump',
                                  'The turtle has no pump: the Pump calls change nothing',
                                  element))
        for x, y in literal_points(text):
            if not inside(x, y):
                result.append(finding('warning', 'turtle-world',
                                      'The point ({:g}, {:g}) is outside the turtle world '
                                      '0..{:.2f}'.format(x, y, WORLD_SIZE), element))
    for rule in rules:
        if rule.event in SCAN_EVENTS and not rule.never:
            result.append(finding('info', 'turtle-test',
                                  'The test rule "{}" cannot hold on the turtle: {} never comes'
                                  .format(rule.line, rule.event), comment))
    return result
