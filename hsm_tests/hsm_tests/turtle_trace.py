# -----------------------------------------------------------------------------
# The Cyberiada HSM-to-ROS2 testing environment
# -----------------------------------------------------------------------------
#
# The trace of the turtle: the poses of a run as CSV and SVG
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

# The picture of a run: turtlesim has no service to capture its window, so the path is drawn
# here over the world the way the simulator draws it - the same scale, background and pen.

import math
import time

from hsm_tests.turtle_rules import WORLD_SIZE

SCALE = 45.0
BACKGROUND = '#4556ff'
PEN = '#b3b8ff'
GRID = '#ffffff'
START = '#33cc33'
TURTLE = '#ffcc00'
# the smallest move and turn kept, m and rad: the simulator publishes its pose at 62 Hz
STEP = 0.02
TURN = 0.05


class Trace:
    # the poses of the turtle since start(), each with its time

    def __init__(self):
        self.points = []
        self.__start = None

    def start(self):
        self.points = []
        self.__start = time.monotonic()

    def add(self, pose):
        if self.__start is None:
            return
        if self.points:
            _, x, y, theta = self.points[-1]
            if math.hypot(pose.x - x, pose.y - y) < STEP and abs(pose.theta - theta) < TURN:
                return
        self.points.append((time.monotonic() - self.__start, pose.x, pose.y, pose.theta))


def csv(points):
    lines = ['t,x,y,theta']
    lines += ['{:.3f},{:.4f},{:.4f},{:.4f}'.format(*p) for p in points]
    return '\n'.join(lines) + '\n'


def pixel(x, y):
    return '{:.1f},{:.1f}'.format(x * SCALE, (WORLD_SIZE - y) * SCALE)


def svg(points, title):
    size = WORLD_SIZE * SCALE
    parts = ['<svg xmlns="http://www.w3.org/2000/svg" width="{0:.0f}" height="{0:.0f}" '
             'viewBox="0 0 {0:.1f} {0:.1f}">'.format(size),
             '<title>{}</title>'.format(escape(title)),
             '<rect width="100%" height="100%" fill="{}"/>'.format(BACKGROUND)]
    for i in range(1, int(WORLD_SIZE) + 1):
        parts.append('<path d="M{0:.1f},0V{1:.1f}M0,{0:.1f}H{1:.1f}" stroke="{2}" '
                     'stroke-opacity="0.15"/>'.format(i * SCALE, size, GRID))
    if points:
        path = ' '.join(pixel(x, y) for _, x, y, _ in points)
        parts.append('<polyline points="{}" fill="none" stroke="{}" stroke-width="3" '
                     'stroke-linejoin="round"/>'.format(path, PEN))
        _, x, y, _ = points[0]
        parts.append('<circle cx="{:.1f}" cy="{:.1f}" r="6" fill="{}"/>'.format(
            x * SCALE, (WORLD_SIZE - y) * SCALE, START))
        _, x, y, theta = points[-1]
        # the turtle: a triangle heading theta, counterclockwise from the x axis
        corners = [(x + 0.3 * math.cos(theta + a), y + 0.3 * math.sin(theta + a))
                   for a in (0.0, 2.5, -2.5)]
        parts.append('<polygon points="{}" fill="{}"/>'.format(
            ' '.join(pixel(*c) for c in corners), TURTLE))
    caption = '{}, {:.1f} s'.format(title, points[-1][0] if points else 0.0)
    parts.append('<text x="8" y="20" font-family="sans-serif" font-size="14" fill="{}">{}'
                 '</text>'.format(GRID, escape(caption)))
    parts.append('</svg>')
    return '\n'.join(parts) + '\n'


def escape(text):
    return text.replace('&', '&amp;').replace('<', '&lt;').replace('>', '&gt;')
