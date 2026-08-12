# -----------------------------------------------------------------------------
# The Cyberiada HSM-to-ROS2 testing environment
# -----------------------------------------------------------------------------
#
# The pytest fixtures which start one node under test with a probe
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

import os

import pytest
import rclpy
from rclpy.executors import SingleThreadedExecutor

from hsm_tests.test_utils.probes import Probe


def parameter_arguments(parameters):
    # the module nodes take no constructor arguments, so a test cannot pass parameter
    # overrides to them directly. Every node reads the arguments of the context it is
    # built in, so the overrides are given to rclpy.init instead.
    # The values are written the way the command line writes them, and the type comes
    # from the literal: a parameter declared as a double needs 1.0 and not 1
    if not parameters:
        return None
    arguments = ['--ros-args']
    for name, value in parameters.items():
        if isinstance(value, bool):
            text = 'true' if value else 'false'
        elif isinstance(value, float):
            text = repr(float(value))
        else:
            text = str(value)
        arguments += ['-p', '{}:={}'.format(name, text)]
    return arguments


class NodeUnderTest:
    # a node under test, its probe and the executor running both of them

    def __init__(self, node, probe, executor):
        self.node = node
        self.probe = probe
        self.executor = executor


@pytest.fixture
def isolated_domain(monkeypatch):
    # the tests must not join the ROS2 graph of the developer, and must not be joined by
    # it: a node started by hand on the same machine would answer the service calls of
    # the test and record its events
    monkeypatch.setenv('ROS_DOMAIN_ID', os.environ.get('HSM_TEST_DOMAIN_ID', '73'))
    monkeypatch.setenv('ROS_AUTOMATIC_DISCOVERY_RANGE', 'LOCALHOST')
    monkeypatch.setenv('ROS_LOCALHOST_ONLY', '1')


@pytest.fixture
def node_factory(isolated_domain):
    # build one node of the framework with the given parameters, together with the probe
    # which drives it. The node names are the constants of the classes, so the context is
    # created and destroyed per test - a node left behind would clash with the next one
    started = []

    def _start(node_class, **parameters):
        assert not started, 'a node under test is already running'
        # the node classes call rclpy.node.Node.__init__ without a context, so they are
        # built in the default one and read its arguments: the default context is where
        # the parameter overrides have to go
        rclpy.init(args=parameter_arguments(parameters))
        executor = SingleThreadedExecutor()
        node = node_class()
        probe = Probe(executor)
        executor.add_node(node)
        executor.add_node(probe)
        started.append((node, probe, executor))
        return NodeUnderTest(node, probe, executor)

    yield _start
    for node, probe, executor in started:
        executor.remove_node(node)
        executor.remove_node(probe)
        node.destroy_node()
        probe.destroy_node()
        rclpy.shutdown()


@pytest.fixture
def storage_directory(tmp_path):
    # the long-term storage of the Storage module is written to the disk, so a test points
    # it at its own directory and never touches the storage of the developer
    directory = tmp_path / 'storage'
    return str(directory)
