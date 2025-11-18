#!/usr/bin/env python3
import time

import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node

from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from move_robot_action.action import MoveRobot
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist


class MoveRobotActionServer(Node):

    def __init__(self):
        super().__init__('move_robot_action_server')

        self.sub_group = MutuallyExclusiveCallbackGroup()
        self.action_group = MutuallyExclusiveCallbackGroup()

        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        self.subscription = self.create_subscription(
            LaserScan,
            'laser_scan',
            self.listener_callback,
            10,
            callback_group=self.sub_group)
        self.subscription  # prevent unused variable warning
        self._action_server = ActionServer(
            self,
            MoveRobot,
            'move_robot',
            self.execute_callback,
            callback_group=self.action_group)
        self.get_logger().info('move_robot Action Server is running...')
        self.last_time_published = 0.0
        self.start = 0.0

    def listener_callback(self, msg):
        self.distance = msg.ranges[180]
        self.range_max = msg.range_max
        self.time = float(msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9)

    def execute_callback(self, goal_handle):
        feedback_msg = MoveRobot.Feedback()
        publisher_msg = Twist()
        result = MoveRobot.Result()
        velocity = 1.0

        if (goal_handle.request.goal_distance < 0 or goal_handle.request.goal_distance > 8.0) or \
            goal_handle.request.timeout < 1.0 or goal_handle.request.precision < 0.01:
            self.get_logger().error("Make sure goal_distance is between 0.0 and 8.0, timeout is longer than 1.0 second and precision value is not less than 0.01.")
            goal_handle.abort() 
            result.succeeded = False
            result.final_precision = 0.0
            result.total_time = 0.0
            return result

        self.get_logger().info('Executing goal...')
        self.start = self.time
        while abs(self.distance - goal_handle.request.goal_distance) > abs(goal_handle.request.precision):
            publisher_msg.linear.x = velocity * (self.distance - goal_handle.request.goal_distance)/abs(self.distance - goal_handle.request.goal_distance)
            self.publisher_.publish(publisher_msg)
            if self.time - self.last_time_published >= 1.0:
                feedback_msg.current_distance = self.distance
                feedback_msg.estimated_time = abs(self.distance - goal_handle.request.goal_distance) / velocity
                goal_handle.publish_feedback(feedback_msg)
                self.last_time_published = self.time
            if self.time - self.start > goal_handle.request.timeout:
                break

        publisher_msg.linear.x = 0.0
        self.publisher_.publish(publisher_msg)

        if self.time - self.start > goal_handle.request.timeout or abs(self.distance - goal_handle.request.goal_distance) > goal_handle.request.precision:
            result.succeeded = False
            result.final_precision = abs(self.distance - goal_handle.request.goal_distance)
            result.total_time = self.time - self.start
        else:
            result.succeeded = True
            result.final_precision = abs(self.distance - goal_handle.request.goal_distance)
            result.total_time = self.time - self.start
        
        goal_handle.succeed()
        return result


def main(args=None):
    rclpy.init(args=args)

    move_robot_action_server = MoveRobotActionServer()

    # --- MultiThreadedExecutor required for concurrency ---
    executor = MultiThreadedExecutor()
    executor.add_node(move_robot_action_server)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        move_robot_action_server.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()