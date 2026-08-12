# -----------------------------------------------------------------------------
# The Cyberiada HSM-to-ROS2 testing environment
# -----------------------------------------------------------------------------
#
# The fake robot node: the deterministic simulation backend of the integration
# tests
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

# The node implements the same ROS2 contract as the turtle driver - the velocity, the
# target point and the odometry - and adds the laser scan the turtlesim simulator cannot
# provide. It is not a better simulator than turtlesim, it is a simpler one: it is driven
# by a test instead of being watched by a human.
#
# Two topics exist for the tests alone and have no counterpart in a real robot: the
# obstacles the scan reports, and the teleport which moves the robot without driving it.

import math

import rclpy
import rclpy.node
from rclpy.executors import ExternalShutdownException

from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float64MultiArray

from hsm_interfaces.msg import SimpleMessage

ODOM_TOPIC = '/odom'
SCAN_TOPIC = '/scan'
TWIST_TOPIC = '/cmd_vel'
GOAL_TOPIC = '/goal_pose'
MESSAGES_TOPIC = '/hsm_ros_msg'
OBSTACLES_TOPIC = '/fake_robot/obstacles'
TELEPORT_TOPIC = '/fake_robot/teleport'
STOP_MESSAGE_FRAME_ID = '__CANCEL_NAV__'
MSG_QUEUE_LEN = 10

# the scan is a full circle one degree apart, the way a scanner reports it: the
# navigation module derives the beam indices from the header of the message
SCAN_BEAMS = 360
SCAN_RANGE_MIN = 0.05
SCAN_RANGE_MAX = 20.0


class FakeRobot(rclpy.node.Node):

    OBJECT_NAME = 'fake_robot'

    def __init__(self):
        rclpy.node.Node.__init__(self, self.OBJECT_NAME)
        self.__control_period = self.__declare('control_period', 0.05,
                                               'the period (s) of the model')
        self.__scan_period = self.__declare('scan_period', 0.1,
                                            'the period (s) of the laser scan')
        self.__arrival_tolerance = self.__declare(
            'arrival_tolerance', 0.05,
            'the distance to the goal at which the robot stops')
        self.__linear_k = self.__declare('linear_k', 1.0, 'the linear gain')
        self.__angular_k = self.__declare('angular_k', 4.0, 'the angular gain')
        self.__linear_limit = self.__declare('linear_limit', 2.0, 'the largest linear speed')
        self.__angular_limit = self.__declare('angular_limit', 2.0, 'the largest angular speed')
        self.__progress_epsilon = self.__declare(
            'progress_epsilon', 0.001,
            'the decrease of the distance to the goal which counts as progress')
        self.__no_progress_limit = self.__declare(
            'no_progress_limit', 5.0,
            'the time (s) without progress after which a collision is reported')
        self.__odom_frame = self.__declare('odom_frame', 'world', 'the odometry frame')
        self.__base_frame = self.__declare('base_frame', 'fake_robot', 'the robot frame')
        self.__publish_scan = self.__declare('publish_scan', True,
                                             'whether the laser scan is published')
        # the distances the scan reports ahead, to the right and everywhere else. A test
        # changes them to place an obstacle exactly at the threshold of the navigation
        # module, which is what the turtlesim simulator cannot do at all
        self.__forward = self.__declare('scan_forward', 10.0,
                                        'the distance (m) the scan reports ahead')
        self.__right = self.__declare('scan_right', 10.0,
                                      'the distance (m) the scan reports to the right')
        self.__elsewhere = self.__declare('scan_elsewhere', 10.0,
                                          'the distance (m) the scan reports elsewhere')

        self.__x = self.__declare('initial_x', 0.0, 'the initial position of the robot')
        self.__y = self.__declare('initial_y', 0.0, 'the initial position of the robot')
        self.__theta = self.__declare('initial_theta', 0.0, 'the initial heading of the robot')
        self.__v = 0.0
        self.__w = 0.0
        self.__commanded = Twist()
        self.__goal = None
        self.__last_distance = float('inf')
        self.__last_progress = self.get_clock().now().nanoseconds

        self.__odom_publisher = self.create_publisher(Odometry, ODOM_TOPIC, MSG_QUEUE_LEN)
        self.__scan_publisher = self.create_publisher(LaserScan, SCAN_TOPIC, MSG_QUEUE_LEN)
        self.__msg_publisher = self.create_publisher(SimpleMessage, MESSAGES_TOPIC,
                                                     MSG_QUEUE_LEN)
        self.create_subscription(Twist, TWIST_TOPIC, self.__twist_callback, MSG_QUEUE_LEN)
        self.create_subscription(PoseStamped, GOAL_TOPIC, self.__goal_callback, MSG_QUEUE_LEN)
        self.create_subscription(Float64MultiArray, OBSTACLES_TOPIC,
                                 self.__obstacles_callback, MSG_QUEUE_LEN)
        self.create_subscription(Float64MultiArray, TELEPORT_TOPIC,
                                 self.__teleport_callback, MSG_QUEUE_LEN)
        self.create_timer(self.__control_period, self.__step)
        if self.__publish_scan:
            self.create_timer(self.__scan_period, self.__publish_laser_scan)
        self.get_logger().info('Fake robot started at ({}, {})'.format(self.__x, self.__y))

    def __declare(self, name, default, description):
        from rcl_interfaces.msg import ParameterDescriptor
        self.declare_parameter(name, default, ParameterDescriptor(description=description))
        return self.get_parameter(name).value

    # -- the robot API side --------------------------------------------------

    def __twist_callback(self, msg):
        # the velocity of the wheels module is kept until it is changed, the way a robot
        # keeps driving until it is told otherwise
        self.__commanded = msg

    def __goal_callback(self, msg):
        if msg.header.frame_id == STOP_MESSAGE_FRAME_ID:
            self.__set_goal(None)
            return
        self.__set_goal((msg.pose.position.x, msg.pose.position.y))

    def __set_goal(self, goal):
        self.__goal = goal
        self.__last_distance = float('inf')
        self.__last_progress = self.get_clock().now().nanoseconds
        if goal is None:
            self.__commanded = Twist()

    # -- the test side -------------------------------------------------------

    def __obstacles_callback(self, msg):
        # [forward, right, elsewhere]
        values = list(msg.data) + [self.__elsewhere] * 3
        self.__forward, self.__right, self.__elsewhere = values[0], values[1], values[2]

    def __teleport_callback(self, msg):
        # [x, y, theta]; the robot is moved without being driven, so a test can start a
        # case from a known place without restarting the node
        values = list(msg.data) + [0.0] * 3
        self.__x, self.__y, self.__theta = values[0], values[1], values[2]
        self.__set_goal(None)

    # -- the model -----------------------------------------------------------

    def __step(self):
        if self.__goal is not None:
            self.__drive_to_goal()
        else:
            self.__v = self.__commanded.linear.x
            self.__w = self.__commanded.angular.z
        # a unicycle: the robot turns and then moves along its heading
        self.__theta += self.__w * self.__control_period
        self.__x += self.__v * math.cos(self.__theta) * self.__control_period
        self.__y += self.__v * math.sin(self.__theta) * self.__control_period
        self.__publish_odometry()

    def __drive_to_goal(self):
        dx = self.__goal[0] - self.__x
        dy = self.__goal[1] - self.__y
        distance = math.sqrt(dx * dx + dy * dy)
        if distance < self.__arrival_tolerance:
            # the robot stops and forgets the goal, but reports nothing: the arrival is
            # reported by the navigation module of the API, which watches the odometry
            self.__set_goal(None)
            self.__v = self.__w = 0.0
            return
        now = self.get_clock().now().nanoseconds
        if distance < self.__last_distance - self.__progress_epsilon:
            self.__last_distance = distance
            self.__last_progress = now
        elif (now - self.__last_progress) / 1e9 > self.__no_progress_limit:
            self.__set_goal(None)
            self.__v = self.__w = 0.0
            event = SimpleMessage()
            event.code = SimpleMessage.MSG_NAVIGATION_COLLISION_DETECTED
            self.__msg_publisher.publish(event)
            self.get_logger().info('The goal cannot be reached')
            return
        angle = math.atan2(dy, dx) - self.__theta
        angle = math.atan2(math.sin(angle), math.cos(angle))
        self.__v = min(self.__linear_k * distance, self.__linear_limit)
        self.__w = max(-self.__angular_limit,
                       min(self.__angular_k * angle, self.__angular_limit))

    def __publish_odometry(self):
        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = self.__odom_frame
        odom.child_frame_id = self.__base_frame
        odom.pose.pose.position.x = float(self.__x)
        odom.pose.pose.position.y = float(self.__y)
        odom.pose.pose.orientation.z = math.sin(self.__theta / 2.0)
        odom.pose.pose.orientation.w = math.cos(self.__theta / 2.0)
        # the speed has to be reported as well: the wheels module reports the stopping of
        # the robot when the speed falls below its threshold, so a robot which never
        # reports a speed never reports a stop either
        odom.twist.twist.linear.x = float(self.__v)
        odom.twist.twist.angular.z = float(self.__w)
        self.__odom_publisher.publish(odom)

    def __publish_laser_scan(self):
        scan = LaserScan()
        scan.header.stamp = self.get_clock().now().to_msg()
        scan.header.frame_id = self.__base_frame
        scan.angle_min = -math.pi
        scan.angle_max = math.pi
        scan.angle_increment = 2.0 * math.pi / SCAN_BEAMS
        scan.range_min = SCAN_RANGE_MIN
        scan.range_max = SCAN_RANGE_MAX
        ranges = [float(self.__elsewhere)] * SCAN_BEAMS
        self.__fill_sector(ranges, scan, 0.0, self.__forward)
        self.__fill_sector(ranges, scan, -math.pi / 2.0, self.__right)
        scan.ranges = ranges
        self.__scan_publisher.publish(scan)

    @classmethod
    def __fill_sector(cls, ranges, scan, angle, distance):
        # a sector wide enough to be found whatever the scan window of the module is
        center = int(round((angle - scan.angle_min) / scan.angle_increment))
        for index in range(center - 10, center + 11):
            ranges[index % len(ranges)] = float(distance)


def main(args=None):
    rclpy.init(args=args)
    node = FakeRobot()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
