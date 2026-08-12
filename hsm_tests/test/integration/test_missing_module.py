# -----------------------------------------------------------------------------
# The Cyberiada HSM-to-ROS2 testing environment
# -----------------------------------------------------------------------------
#
# A controller started without the modules it needs reports it and gives up
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

# A controller cannot do anything without the module nodes of the API, and it used to wait
# for them without end, with one line of log per second to show for it. It reports the
# module it is waiting for and gives up after the limit, which is why the limit is a
# parameter: a test asks for a short one.
#
# The case is about how one process starts, so it starts that process itself instead of a
# launch description. A launch description would add nothing, and the tests of a package
# are run by colcon in a worker thread, where the process supervision of launch does not
# report the output of a process at all.

import os
import shutil
import subprocess
import sys

import pytest

from hsm_tests.launch_support import FAST_PARAMETERS, diagram, generate_controller

# a domain of its own, so no module node of the developer or of another case answers
DOMAIN = '96'
STARTUP_LIMIT = 3.0

pytestmark = pytest.mark.integration


@pytest.fixture
def lonely_controller():
    # the controller of a diagram, started with nothing else running
    directory, module = generate_controller(diagram('turtle-square'))
    environment = dict(os.environ)
    environment['ROS_DOMAIN_ID'] = DOMAIN
    environment['ROS_AUTOMATIC_DISCOVERY_RANGE'] = 'LOCALHOST'
    environment['PYTHONPATH'] = directory + os.pathsep + environment.get('PYTHONPATH', '')
    arguments = ['--ros-args']
    values = dict(FAST_PARAMETERS)
    values.update({'service_startup_timeout': 0.2, 'service_startup_limit': STARTUP_LIMIT})
    for name, value in values.items():
        arguments += ['-p', '{}:={}'.format(name, repr(float(value)))]
    process = subprocess.Popen(
        [sys.executable, '-m', 'hsm_controller.{}'.format(module)] + arguments,
        stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, env=environment)
    yield process
    if process.poll() is None:
        process.kill()
        process.wait(timeout=10)
    shutil.rmtree(directory, ignore_errors=True)


def test_the_controller_gives_up_instead_of_waiting(lonely_controller):
    # it ends by itself, and well within the time a test is willing to wait
    output = lonely_controller.communicate(timeout=STARTUP_LIMIT * 10)[0]
    assert lonely_controller.returncode != 0, \
        'the controller reported success without its modules'
    # the report names the service, so the one who started the system knows which node
    # is missing
    assert 'service not available' in output, output[-500:]
    assert 'is not available after' in output, output[-500:]


def test_the_limit_is_the_one_it_was_given(lonely_controller):
    output = lonely_controller.communicate(timeout=STARTUP_LIMIT * 10)[0]
    assert '{}'.format(STARTUP_LIMIT) in output, \
        'the controller did not wait the time it was given:\n{}'.format(output[-500:])
