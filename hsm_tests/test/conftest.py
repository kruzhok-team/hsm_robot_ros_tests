# -----------------------------------------------------------------------------
# The Cyberiada HSM-to-ROS2 testing environment
# -----------------------------------------------------------------------------
#
# Every test of a tier carries the marker of its tier
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

# A tier is selected by its marker, and the tier a test belongs to is the directory it is
# in. Marking the files by hand does not work for the launch tests: the case a launch test
# produces is built by the plugin of launch_testing and not by the module, so a marker
# written in the module does not reach it and the test is silently deselected. The marker
# is added here instead, after the cases have been collected, so the whole tier answers to
# its name.

import os

import pytest

TIERS = ('unit', 'integration', 'system')


def pytest_collection_modifyitems(items):
    for item in items:
        directory = os.path.basename(os.path.dirname(str(item.fspath)))
        if directory in TIERS:
            item.add_marker(getattr(pytest.mark, directory))
