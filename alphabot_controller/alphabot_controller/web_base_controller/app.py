from flask import Flask, render_template
from flask_socketio import SocketIO
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import OccupancyGrid
from tf2_ros import Buffer, TransformListener
from geometry_msgs.msg import TransformStamped
import threading
import math
import time
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy


from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped


# ---------------- FLASK ----------------
app = Flask(__name__)
socketio = SocketIO(app, cors_allowed_origins="*")  # Allow WebSocket from browser

# ---------------- SHARED STATE ----------------
robot_state = {
    "linear_x": 0.0,
    "angular_z": 0.0,
    "x": 0.0,
    "y": 0.0,
    "yaw": 0.0
}

map_state = {
    "width": 0,
    "height": 0,
    "resolution": 0.0,
    "origin_x": 0.0,
    "origin_y": 0.0,
    "data": []
}

last_cmd_time = time.time()
CMD_TIMEOUT = 0.5  # seconds (dead-man switch)

# ---------------- ROS 2 NODE ----------------
class WebJoystickNode(Node):
    def __init__(self):
        super().__init__('web_joystick')

        # Publisher for joystick commands
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        map_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL
        )

        self.create_subscription(
            OccupancyGrid,
            '/map',
            self.map_callback,
            map_qos
        )

        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        # TF buffer and listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Safety timer to stop robot if no joystick input
        self.create_timer(0.1, self.safety_check)

        # Timer to update robot state from TF
        self.create_timer(0.05, self.update_state_from_tf)  # 20 Hz

        # Previous pose for velocity computation
        self.prev_x = 0.0
        self.prev_y = 0.0
        self.prev_yaw = 0.0
        self.prev_time = time.time()

    def send_nav_goal(self, x, y, yaw=0.0):
        if not self.nav_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().error("Nav2 action server not available")
            return

        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = "map"
        goal.pose.header.stamp = self.get_clock().now().to_msg()

        goal.pose.pose.position.x = float(x)
        goal.pose.pose.position.y = float(y)

        # yaw → quaternion
        goal.pose.pose.orientation.z = math.sin(yaw / 2.0)
        goal.pose.pose.orientation.w = math.cos(yaw / 2.0)

        self.nav_client.send_goal_async(goal)
        self.get_logger().info(f"Goal sent: x={x:.2f}, y={y:.2f}")
        
    def publish_cmd(self, x, y):
        global last_cmd_time
        last_cmd_time = time.time()

        twist = Twist()
        twist.linear.x = y / 100.0
        twist.angular.z = -x / 100.0
        self.cmd_pub.publish(twist)

    def safety_check(self):
        if time.time() - last_cmd_time > CMD_TIMEOUT:
            self.cmd_pub.publish(Twist())  # Stop robot

    def update_state_from_tf(self):
        global robot_state
        try:
            t = self.tf_buffer.lookup_transform(
                'map',
                'base_link',
                rclpy.time.Time()
            )

            x = t.transform.translation.x
            y = t.transform.translation.y

            q = t.transform.rotation
            siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
            cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
            yaw = math.atan2(siny_cosp, cosy_cosp)

            # compute linear/angular velocities
            now = time.time()
            dt = now - self.prev_time if now - self.prev_time > 1e-6 else 1e-6
            linear_vel = math.sqrt((x - self.prev_x)**2 + (y - self.prev_y)**2) / dt
            angular_vel = (yaw - self.prev_yaw) / dt

            robot_state.update({
                "x": x,
                "y": y,
                "yaw": yaw,
                "linear_x": linear_vel,
                "angular_z": angular_vel
            })
            self.prev_x, self.prev_y, self.prev_yaw, self.prev_time = x, y, yaw, now

        except Exception as e:
            self.get_logger().warn(f"TF lookup failed: {e}")

    def map_callback(self, msg):
        global map_state
        map_state["width"] = msg.info.width
        map_state["height"] = msg.info.height
        map_state["resolution"] = msg.info.resolution
        map_state["origin_x"] = msg.info.origin.position.x
        map_state["origin_y"] = msg.info.origin.position.y
        map_state["data"] = list(msg.data)


# ---------------- ROS INIT ----------------
rclpy.init()
ros_node = WebJoystickNode()

def ros_spin():
    rclpy.spin(ros_node)

threading.Thread(target=ros_spin, daemon=True).start()

# ---------------- SOCKET.IO ----------------
@socketio.on("joystick")
def handle_joystick(data):
    ros_node.publish_cmd(data["x"], data["y"])

@socketio.on("goal")
def handle_goal(data):
    ros_node.send_nav_goal(data["x"], data["y"])
    
@socketio.on("connect")
def on_connect():
    print("Client connected")
    # Start state push for this client
    socketio.start_background_task(push_robot_state_for_client)

def push_robot_state_for_client():
    while True:
        socketio.emit("state", robot_state)
        # print(f"Emitted: {robot_state}")
        socketio.sleep(0.1)

@socketio.on("disconnect")
def on_disconnect():
    print("Client disconnected")

# Push robot state at 10 Hz
def push_robot_state():
    while True:
        socketio.emit("state", robot_state)
        socketio.sleep(0.1)

# Push map at 1 Hz
def push_map_state():
    while True:
        if map_state["data"]:
            socketio.emit("map", map_state)
        socketio.sleep(1.0)

socketio.start_background_task(push_robot_state)
socketio.start_background_task(push_map_state)

# ---------------- FLASK ROUTES ----------------
@app.route("/")
def index():
    return render_template("index.html")

# ---------------- MAIN ----------------
if __name__ == "__main__":
    socketio.run(app, host="0.0.0.0", port=5000)
