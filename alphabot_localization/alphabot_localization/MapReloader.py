import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid

class MapRepublisher(Node):
    def __init__(self):
        super().__init__('map_republisher')
        self.sub = self.create_subscription(
            OccupancyGrid,
            '/map',
            self.map_callback,
            qos_profile=rclpy.qos.QoSProfile(depth=1)
        )
        self.pub = self.create_publisher(
            OccupancyGrid,
            '/map_republished',
            qos_profile=rclpy.qos.QoSProfile(depth=1)
        )

    def map_callback(self, msg):
        self.pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = MapRepublisher()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
