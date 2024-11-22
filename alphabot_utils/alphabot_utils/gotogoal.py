#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf_transformations import euler_from_quaternion
import sys
import math

class GoToGoal(Node):
    def __init__(self):
        super().__init__("Go_to_Goal_Node")
        self.cmd_vel_pub = self.create_publisher(Twist, '/alphabot_controller/cmd_vel_unstamped', 10)
        self.odom_sub = self.create_subscription(Odometry, '/alphabot_controller/odom', self.odom_callback, 10)
        self.timer = self.create_timer(0.1, self.go_to_goal)
        self.position = None
        self.orientation_theta = None

    def odom_callback(self, data):
        # Extract position (x, y) and orientation (quaternion -> theta)
        self.position = data.pose.pose.position
        orientation_q = data.pose.pose.orientation
        _, _, self.orientation_theta = euler_from_quaternion(
            [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        )

    def go_to_goal(self):
        if self.position is None or self.orientation_theta is None:
            self.get_logger().info("Waiting for Odometry data...")
            return

        goal_x = float(sys.argv[1])
        goal_y = float(sys.argv[2])
        goal_theta = float(sys.argv[3])

        new_vel = Twist()

        # Euclidean Distance
        distance_to_goal = math.sqrt((goal_x - self.position.x) ** 2 + (goal_y - self.position.y) ** 2)
        # Angle to Goal
        angle_to_goal = math.atan2(goal_y - self.position.y, goal_x - self.position.x)

        distance_tolerance = 0.1
        angle_tolerance = 0.01

        angle_error = angle_to_goal - self.orientation_theta
        kp = 1.2

        if abs(angle_error) > angle_tolerance:
            new_vel.angular.z = kp * angle_error
        else:
            if distance_to_goal >= distance_tolerance:
                new_vel.linear.x = kp * distance_to_goal
            else:
                new_vel.linear.x = 0.0
                self.get_logger().info("Goal Reached")
                quit()

        self.cmd_vel_pub.publish(new_vel)

def main(args=None):
    rclpy.init(args=args)
    turtle_gtg = GoToGoal()
    rclpy.spin(turtle_gtg)
    turtle_gtg.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
