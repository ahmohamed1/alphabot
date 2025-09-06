import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist, Pose
from tf2_ros import Buffer, TransformListener, LookupException
from nav_msgs.msg import OccupancyGrid, Path
from rclpy.qos import QoSProfile, DurabilityPolicy
from PyQt5.QtCore import QObject, pyqtSignal


class ROS2Interface(Node, QObject):
    map_received = pyqtSignal(object)

    def __init__(self):
        Node.__init__(self, 'alphabot_gui_node')
        QObject.__init__(self)
        self.cmd_vel_pub_ = self.create_publisher(Twist,'/cmd_vel', 10)
        self.battery_level_sub_ = self.create_subscription(Float32, '/battery_level', self.battery_callback, 10)
        self.battery_level_ = 0.0

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        map_qos = QoSProfile(depth=10)
        map_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL

        self.map_sub = self.create_subscription(
            OccupancyGrid, "/map", self.map_callback, map_qos
        )
        self.map_ = None

    
    def map_callback(self, map_msg: OccupancyGrid):
        self.map_ = map_msg
        self.map_received.emit(map_msg)

    def battery_callback(self, msg):
        self.battery_level_ = msg.data
    
    def publish_cmd(self, direction, speed=0.1):
        msg = Twist()
        # Basic mapping: you decide how each direction sets linear/angular
        if direction == 'FORWARD':
            msg.linear.x = speed
        elif direction == 'BACKWARD':
            msg.linear.x = -speed
        elif direction == 'LEFT':
            msg.angular.z = speed
        elif direction == 'RIGHT':
            msg.angular.z = -speed
        elif direction == 'STOP':
            # Zero velocities
            msg.linear.x = 0.0
            msg.angular.z = 0.0
        self.cmd_vel_pub_.publish(msg)
        # self.get_logger().info(f'Published Twist: {msg}')
        # self.get_logger().info(f'Published Twist: {direction}')
    
    def publish_cmd_joystick(self, velocity, rotation):
        msg = Twist()
        msg.linear.x = velocity
        msg.angular.z = rotation
        self.cmd_vel_pub_.publish(msg)

    def get_battery_level(self):
        return self.battery_level_
    
    def getRobotPositionInMap(self):
        try:
            map_to_base_tf = self.tf_buffer.lookup_transform(
                self.map_.header.frame_id, "base_footprint", rclpy.time.Time()
            )
        except LookupException:
            self.get_logger().error("Could not transform from map to base_footprint")
            return

        map_to_base_pose = Pose()
        map_to_base_pose.position.x = map_to_base_tf.transform.translation.x
        map_to_base_pose.position.y = map_to_base_tf.transform.translation.y
        map_to_base_pose.orientation = map_to_base_tf.transform.rotation

        return map_to_base_pose