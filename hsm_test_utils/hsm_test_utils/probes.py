# -----------------------------------------------------------------------------
# The Cyberiada HSM-to-ROS2 testing environment
# -----------------------------------------------------------------------------
#
# The probe node which records what a node under test publishes and calls its
# services
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

import rclpy.node
from rclpy.executors import SingleThreadedExecutor

from hsm_interfaces.msg import SimpleMessage, StringArgMessage

# the topics the API modules report their events on
MESSAGES_TOPIC = '/hsm_ros_msg'
STR_MESSAGES_TOPIC = '/hsm_ros_str_msg'
MSG_QUEUE_LEN = 10

# the node under test and the probe share one executor, so waiting is a loop of short
# spins and not a blocking call: a service call which never answers, or an event which
# never arrives, has to fail the test rather than stop it
SPIN_STEP = 0.02
DEFAULT_TIMEOUT = 5.0


class Probe(rclpy.node.Node):
    # the other side of the node under test: it records the messages the node publishes,
    # publishes what the node listens to, and calls the services the node advertises

    def __init__(self, executor=None, name='hsm_test_probe'):
        rclpy.node.Node.__init__(self, name)
        # the node tests put the node under test and the probe in one executor, and that
        # executor is what has to be spun. The integration tests run the framework as
        # separate processes, so the probe is alone and spins itself
        if executor is None:
            executor = SingleThreadedExecutor()
            executor.add_node(self)
        self.__executor = executor
        self.__recorders = {}
        self.__publishers = {}
        self.__clients = {}
        self.events = self.record(MESSAGES_TOPIC, SimpleMessage)
        self.str_events = self.record(STR_MESSAGES_TOPIC, StringArgMessage)

    # -- recording -----------------------------------------------------------

    def record(self, topic, message_type):
        # start recording a topic and return the list the messages are appended to
        if topic not in self.__recorders:
            messages = []
            self.create_subscription(message_type, topic, messages.append, MSG_QUEUE_LEN)
            self.__recorders[topic] = messages
        return self.__recorders[topic]

    def recorded(self, topic):
        return self.__recorders[topic]

    # -- driving -------------------------------------------------------------

    def publisher(self, topic, message_type):
        if topic not in self.__publishers:
            self.__publishers[topic] = self.create_publisher(message_type, topic, MSG_QUEUE_LEN)
        return self.__publishers[topic]

    def publish(self, topic, message, wait_for_listener=True):
        publisher = self.publisher(topic, type(message))
        # a publisher which has just been created is not connected to the subscriptions
        # yet, and its first message is dropped. The nodes of the integration tests are
        # separate processes, so the discovery takes long enough for this to matter
        if wait_for_listener:
            self.wait_for(lambda: publisher.get_subscription_count() > 0, DEFAULT_TIMEOUT)
        publisher.publish(message)
        # let the node under test process the message before the test asserts on it
        self.spin(SPIN_STEP * 3)

    def call(self, service, service_type, request=None, timeout=DEFAULT_TIMEOUT):
        # call a service of the node under test and return the response. Both nodes run on
        # the same executor, so the executor has to be spun and not the probe alone -
        # rclpy.spin_until_future_complete would spin the probe and deadlock
        if service not in self.__clients:
            self.__clients[service] = self.create_client(service_type, service)
        client = self.__clients[service]
        assert client.wait_for_service(timeout_sec=timeout), \
            'the {} service did not appear'.format(service)
        future = client.call_async(request if request is not None else service_type.Request())
        self.__executor.spin_until_future_complete(future, timeout_sec=timeout)
        assert future.done(), 'the {} service did not answer'.format(service)
        return future.result()

    # -- waiting -------------------------------------------------------------

    def spin(self, duration):
        # spin both nodes for the given time
        deadline = self.get_clock().now().nanoseconds + duration * 1e9
        while self.get_clock().now().nanoseconds < deadline:
            self.__executor.spin_once(timeout_sec=SPIN_STEP)

    def wait_for(self, condition, timeout=DEFAULT_TIMEOUT):
        # spin until the condition holds; returns whether it did
        deadline = self.get_clock().now().nanoseconds + timeout * 1e9
        while self.get_clock().now().nanoseconds < deadline:
            if condition():
                return True
            self.__executor.spin_once(timeout_sec=SPIN_STEP)
        return condition()

    def wait_for_event(self, code, timeout=DEFAULT_TIMEOUT):
        # wait for an HSM event and return it, or fail the test naming the code
        found = self.wait_for(lambda: any(m.code == code for m in self.events), timeout)
        assert found, 'the event {} was not reported, got {}'.format(code, self.event_codes())
        return [m for m in self.events if m.code == code][0]

    def wait_for_str_event(self, code, timeout=DEFAULT_TIMEOUT):
        found = self.wait_for(lambda: any(m.code == code for m in self.str_events), timeout)
        assert found, 'the string event {} was not reported'.format(code)
        return [m for m in self.str_events if m.code == code][0]

    def expect_no_event(self, code, duration=0.3):
        # the absence of an event is asserted by spinning for a while and looking
        self.spin(duration)
        assert not any(m.code == code for m in self.events), \
            'the event {} was reported and should not have been'.format(code)

    # -- reading -------------------------------------------------------------

    def event_codes(self):
        return [m.code for m in self.events]

    def count_events(self, code):
        return len([m for m in self.events if m.code == code])

    def clear(self):
        for messages in self.__recorders.values():
            del messages[:]
