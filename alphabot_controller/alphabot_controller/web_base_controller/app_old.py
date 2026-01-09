from flask import Flask, request, jsonify, render_template
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import threading
import math

app = Flask(__name__)

# Shared robot state (feedback)
robot_state = {
    "linear_x": 0.0,
    "angular_z": 0.0,
    "x": 0.0,
    "y": 0.0,
    "yaw": 0.0
}

# ---------------- ROS 2 NODE ----------------
class WebJoystickNode(Node):
    def __init__(self):
        super().__init__('web_joystick')

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # ONLY subscribe to ODOM (real robot state)
        self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )

    def publish_cmd(self, x, y):
        twist = Twist()
        twist.linear.x = y / 100.0
        twist.angular.z = -x / 100.0
        self.cmd_pub.publish(twist)

    def odom_callback(self, msg):
        # Position
        robot_state["x"] = msg.pose.pose.position.x
        robot_state["y"] = msg.pose.pose.position.y

        # Velocity (REAL velocity)
        robot_state["linear_x"] = msg.twist.twist.linear.x
        robot_state["angular_z"] = msg.twist.twist.angular.z

        # Orientation → yaw
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        robot_state["yaw"] = math.atan2(siny_cosp, cosy_cosp)

# ---------------- ROS INIT ----------------
rclpy.init()
ros_node = WebJoystickNode()

def ros_spin():
    rclpy.spin(ros_node)

threading.Thread(target=ros_spin, daemon=True).start()

# ---------------- FLASK ----------------
@app.route('/')
def index():
    return render_template('index.html')

@app.route('/joystick', methods=['POST'])
def joystick():
    data = request.get_json()
    ros_node.publish_cmd(
        data.get('x', 0),
        data.get('y', 0)
    )
    return jsonify(status='ok')

@app.route('/state')
def state():
    return jsonify(robot_state)

# ---------------- MAIN ----------------
if __name__ == '__main__':
    app.run(host='0.0.0.0', port=5000)
