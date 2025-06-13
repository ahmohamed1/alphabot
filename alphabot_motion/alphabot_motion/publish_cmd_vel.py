#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from rclpy.qos import QoSProfile, QoSReliabilityPolicy

class CMDVELPublisher(Node):
    def __init__(self):
        super().__init__("simple_publisher_cmd_vel")

        qos = QoSProfile(
            depth=10,
            reliability=QoSReliabilityPolicy.BEST_EFFORT
        )

        # Subscribers and publishers
        self.cmd_pub = self.create_publisher(Twist, "/joy_vel", qos)

        # Control loop
        self.timer = self.create_timer(0.1, self.control_loop)

    def control_loop(self):
        cmd_vel = Twist()
        cmd_vel.angular.z = 0.0
        cmd_vel.linear.x = 0.5

        self.cmd_pub.publish(cmd_vel)



def main(args=None):
    rclpy.init(args=args)
    simple_publisher_cmd_vel = CMDVELPublisher()
    rclpy.spin(simple_publisher_cmd_vel)
    simple_publisher_cmd_vel.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
