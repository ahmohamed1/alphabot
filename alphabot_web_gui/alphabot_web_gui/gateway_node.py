#!/usr/bin/env python3
"""Main ROS 2 Web GUI Gateway Node for AlphaBot / ServiceBot."""

import asyncio
import json
import math
import os
import signal
import socket
import threading
import time
from pathlib import Path
from typing import Any, Dict, List, Optional

os.environ["RMW_IMPLEMENTATION"] = "rmw_fastrtps_cpp"

import numpy as np
import rclpy
from action_msgs.msg import GoalStatus
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist
from nav2_msgs.action import NavigateToPose
from nav2_msgs.srv import LoadMap
from nav_msgs.msg import OccupancyGrid, Odometry, Path as NavPath
from rclpy.action import ActionClient
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool
from tf2_ros import Buffer, TransformListener

from alphabot_web_gui.http_ws_server import AsyncHttpWsServer, WebSocketConnection
from alphabot_web_gui.map_manager import (
    MapAndWaypointManager,
    quaternion_to_yaw,
    yaw_to_quaternion,
)
from alphabot_web_gui.mode_manager import ModeManager


def get_lan_ip_addresses() -> List[str]:
    ips: List[str] = []
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.connect(("10.255.255.255", 1))
        ip = s.getsockname()[0]
        s.close()
        if ip and not ip.startswith("127."):
            ips.append(ip)
    except Exception:
        pass
    if not ips:
        ips.append("127.0.0.1")
    return ips


def resolve_mapping_directories() -> List[Path]:
    dirs: List[Path] = []
    # 1. Workspace src directory for persistent map storage across builds
    ws_candidates = [
        Path("/home/abdll/workspace/alphabot_ws/src/alphabot/alphabot_mapping/maps"),
        Path.cwd() / "alphabot" / "alphabot_mapping" / "maps",
    ]
    for c in ws_candidates:
        if c.parent.exists():
            dirs.append(c)
            break

    # 2. Installed share directory for immediate runtime resolution by global_localization.launch.py
    try:
        share_dir = Path(get_package_share_directory("alphabot_mapping")) / "maps"
        if share_dir not in dirs:
            dirs.append(share_dir)
    except Exception:
        pass

    if not dirs:
        dirs.append(Path.home() / ".alphabot_web_gui" / "maps")
    return dirs


def resolve_web_static_dir() -> Path:
    local_dir = Path(__file__).resolve().parent.parent / "web"
    if (local_dir / "index.html").is_file():
        return local_dir
    try:
        share_web = Path(get_package_share_directory("alphabot_web_gui")) / "web"
        if (share_web / "index.html").is_file():
            return share_web
    except Exception:
        pass
    return local_dir


class AlphaBotWebGatewayNode(Node):
    """ROS 2 Node bridging AlphaBot topics/TF/Nav2 with the Local Network Web GUI."""

    def __init__(
        self,
        db_path: Optional[Path] = None,
        mapping_dirs: Optional[List[Path]] = None,
    ):
        super().__init__("alphabot_web_gateway")

        # Declare ROS 2 parameters
        self.declare_parameter(
            "robot_model", os.environ.get("ROBOT_MODEL", "servicebot")
        )
        self.declare_parameter("host", "0.0.0.0")
        self.declare_parameter("port", 8090)
        self.declare_parameter("cmd_vel_topic", "/key_vel")
        self.declare_parameter("safety_stop_topic", "/safety_stop")
        self.declare_parameter("default_map", "")
        self.declare_parameter("max_linear_speed", 0.40)
        self.declare_parameter("max_angular_speed", 1.50)
        self.declare_parameter("watchdog_timeout_sec", 0.30)
        self.declare_parameter("use_sim_time_for_switches", False)

        self.robot_model: str = str(self.get_parameter("robot_model").value)
        self.robot_radius: float = (
            0.22 if self.robot_model == "servicebot" else 0.15
        )
        self.host: str = self.get_parameter("host").value
        self.port: int = int(self.get_parameter("port").value)
        self.cmd_vel_topic: str = self.get_parameter("cmd_vel_topic").value
        self.safety_stop_topic: str = self.get_parameter("safety_stop_topic").value
        default_map: str = str(self.get_parameter("default_map").value).strip()
        self.max_linear_speed: float = float(
            self.get_parameter("max_linear_speed").value
        )
        self.max_angular_speed: float = float(
            self.get_parameter("max_angular_speed").value
        )
        self.watchdog_timeout_sec: float = float(
            self.get_parameter("watchdog_timeout_sec").value
        )
        sim_time_builtin = bool(self.get_parameter("use_sim_time").value)
        sim_time_switch = bool(
            self.get_parameter("use_sim_time_for_switches").value
        )
        self.use_sim_time_for_switches: bool = sim_time_builtin or sim_time_switch

        # Storage & Managers
        actual_db = db_path or (Path.home() / ".alphabot_web_gui" / "alphabot_gui.db")
        actual_map_dirs = mapping_dirs or resolve_mapping_directories()
        self.map_mgr = MapAndWaypointManager(actual_db, actual_map_dirs)

        # Start in STANDBY (First-Time Environment mode) unless a map was explicitly requested
        self.active_map_name: str = default_map if default_map else "unmapped"
        initial_mode = "NAVIGATION" if default_map else "STANDBY"
        self.mode_mgr = ModeManager(
            initial_mode=initial_mode, initial_map=self.active_map_name
        )

        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, self.cmd_vel_topic, 10)
        self.safety_stop_pub = self.create_publisher(Bool, self.safety_stop_topic, 10)
        self.init_pose_pub = self.create_publisher(
            PoseWithCovarianceStamped, "/initialpose", 10
        )
        self.sim_slam_mode_pub = self.create_publisher(
            Bool, "/sim/set_slam_mode", 10
        )
        self.odom_filtered_pub = self.create_publisher(
            Odometry, "/odometry/filtered", 10
        )

        # QoS Profiles
        map_qos = QoSProfile(
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        scan_qos = QoSProfile(
            durability=DurabilityPolicy.VOLATILE,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        # Subscribers
        self.map_sub = self.create_subscription(
            OccupancyGrid, "/map", self.map_callback, map_qos
        )
        self.scan_sub = self.create_subscription(
            LaserScan, "/scan", self.scan_callback, scan_qos
        )
        self.plan_sub = self.create_subscription(
            NavPath, "/plan", self.plan_callback, 10
        )
        self.odom_sub = self.create_subscription(
            Odometry, "/odometry/filtered", self.odom_callback, 10
        )
        self.ctrl_odom_sub = self.create_subscription(
            Odometry, "/alphabot_controller/odom", self.ctrl_odom_callback, 10
        )

        # TF2
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(
            self.tf_buffer, self, spin_thread=False
        )

        # Nav2 Action & Service Clients
        self.nav_client = ActionClient(self, NavigateToPose, "navigate_to_pose")
        self.load_map_client = self.create_client(LoadMap, "/map_server/load_map")

        # Runtime state
        self.robot_pose = self.map_mgr.get_state(
            "last_pose", {"x": 0.0, "y": 0.0, "yaw": 0.0}
        )
        self.robot_vel = {"vx": 0.0, "wz": 0.0}
        self.safety_locked = False
        self.nav_status = "IDLE"
        self.target_name = ""
        self.active_goal_pose: Optional[Dict[str, float]] = None
        self.nav_distance_remaining = 0.0
        self.current_goal_handle = None

        self.grid_width = 0
        self.grid_height = 0
        self.grid_resolution = 0.05
        self.grid_origin_x = 0.0
        self.grid_origin_y = 0.0
        self.grid_data: Optional[np.ndarray] = None
        self.latest_map_payload: Optional[Dict[str, Any]] = None
        self.map_version = 0

        self.latest_scan_points: List[List[float]] = []
        self.latest_path_points: List[List[float]] = []

        # Watchdog state for teleop
        self.last_cmd_vel_time = 0.0
        self.is_teleop_active = False

        # Preload static map from disk only if a default_map was explicitly specified
        if default_map:
            self._try_preload_static_map(self.active_map_name)

        # Use STEADY_TIME clock for internal timers so they never stall if /clock is paused
        self._steady_clock = Clock(clock_type=ClockType.STEADY_TIME)
        self.create_timer(
            0.05, self.watchdog_timer_cb, clock=self._steady_clock
        )  # 20Hz dead-man's switch
        self.create_timer(
            0.10, self.update_tf_pose_cb, clock=self._steady_clock
        )  # 10Hz TF lookup
        self.create_timer(
            2.00, self.periodic_state_sync_cb, clock=self._steady_clock
        )  # 0.5Hz state & node check

    def _try_preload_static_map(self, map_name: str) -> bool:
        loaded = self.map_mgr.load_static_map_as_grid(map_name)
        if loaded is None:
            return False
        w, h, res, ox, oy, grid_data, payload = loaded
        self.grid_width = w
        self.grid_height = h
        self.grid_resolution = res
        self.grid_origin_x = ox
        self.grid_origin_y = oy
        self.grid_data = grid_data
        self.latest_map_payload = payload
        self.map_version += 1
        return True

    # ---------------------------------------------------------
    # Teleop & Watchdog (Compatible with twist_mux priorities)
    # ---------------------------------------------------------
    def send_teleop_velocity(self, vx: float, wz: float) -> None:
        if self.safety_locked:
            return

        clamped_vx = float(
            np.clip(vx, -self.max_linear_speed, self.max_linear_speed)
        )
        clamped_wz = float(
            np.clip(wz, -self.max_angular_speed, self.max_angular_speed)
        )

        msg = Twist()
        msg.linear.x = clamped_vx
        msg.angular.z = clamped_wz
        self.cmd_vel_pub.publish(msg)

        self.last_cmd_vel_time = time.monotonic()
        self.is_teleop_active = (
            abs(clamped_vx) > 1e-4 or abs(clamped_wz) > 1e-4
        )

    def watchdog_timer_cb(self) -> None:
        if self.is_teleop_active:
            elapsed = time.monotonic() - self.last_cmd_vel_time
            if elapsed > self.watchdog_timeout_sec:
                # Publish zero Twist burst then stop publishing so twist_mux unlocks priority 90 for Nav2
                stop_msg = Twist()
                self.cmd_vel_pub.publish(stop_msg)
                self.is_teleop_active = False
                self.get_logger().info(
                    "Teleop watchdog timeout: published zero velocity and released /key_vel."
                )

    def set_safety_lock(self, locked: bool) -> None:
        self.safety_locked = bool(locked)
        msg = Bool()
        msg.data = self.safety_locked
        self.safety_stop_pub.publish(msg)
        if self.safety_locked:
            self.cmd_vel_pub.publish(Twist())
            self.is_teleop_active = False

    def emergency_stop(self) -> None:
        self.is_teleop_active = False
        stop_msg = Twist()
        if rclpy.ok():
            try:
                for _ in range(3):
                    self.cmd_vel_pub.publish(stop_msg)
            except Exception:
                pass
        self.cancel_navigation()

    # ---------------------------------------------------------
    # TF2 Pose, Odometry, Map, Scan & Plan Callbacks
    # ---------------------------------------------------------
    def update_tf_pose_cb(self) -> None:
        for parent_frame in ("map", "odom"):
            for child_frame in ("base_footprint", "base_link"):
                try:
                    t = self.tf_buffer.lookup_transform(
                        parent_frame, child_frame, rclpy.time.Time()
                    )
                    q = t.transform.rotation
                    yaw = quaternion_to_yaw(q.x, q.y, q.z, q.w)
                    self.robot_pose = {
                        "x": round(float(t.transform.translation.x), 3),
                        "y": round(float(t.transform.translation.y), 3),
                        "yaw": round(float(yaw), 3),
                    }
                    return
                except Exception:
                    continue

    def periodic_state_sync_cb(self) -> None:
        try:
            self.map_mgr.set_state("last_pose", self.robot_pose)
            self.map_mgr.set_state("active_map", self.active_map_name)
            node_names = self.get_node_names()
            self.mode_mgr.detect_active_mode_from_ros_nodes(node_names)
        except Exception:
            pass

    def odom_callback(self, msg: Odometry) -> None:
        self.robot_vel = {
            "vx": round(float(msg.twist.twist.linear.x), 3),
            "wz": round(float(msg.twist.twist.angular.z), 3),
        }

    def ctrl_odom_callback(self, msg: Odometry) -> None:
        self.odom_callback(msg)
        # In Gazebo simulated_robot.launch.py, ekf_filter_node is not launched so
        # /odometry/filtered has no external publisher; relay /alphabot_controller/odom
        # so Nav2 bt_navigator and controller_server receive odometry.
        if self.count_publishers("/odometry/filtered") <= 1:
            self.odom_filtered_pub.publish(msg)

    def map_callback(self, msg: OccupancyGrid) -> None:
        w = int(msg.info.width)
        h = int(msg.info.height)
        if w <= 0 or h <= 0:
            return
        res = float(msg.info.resolution)
        ox = float(msg.info.origin.position.x)
        oy = float(msg.info.origin.position.y)
        raw_data = np.asarray(msg.data, dtype=np.int8)

        payload = self.map_mgr.encode_occupancy_grid_to_png(
            w, h, res, ox, oy, raw_data
        )
        if payload is not None:
            self.grid_width = w
            self.grid_height = h
            self.grid_resolution = res
            self.grid_origin_x = ox
            self.grid_origin_y = oy
            self.grid_data = raw_data
            self.latest_map_payload = payload
            self.map_version += 1

    def scan_callback(self, msg: LaserScan) -> None:
        # Attempt to lookup laser frame in map frame, fallback to robot_pose
        sx = self.robot_pose["x"]
        sy = self.robot_pose["y"]
        syaw = self.robot_pose["yaw"]
        frame_id = msg.header.frame_id.lstrip("/")
        if frame_id:
            try:
                t = self.tf_buffer.lookup_transform(
                    "map", frame_id, rclpy.time.Time()
                )
                sx = float(t.transform.translation.x)
                sy = float(t.transform.translation.y)
                q = t.transform.rotation
                syaw = quaternion_to_yaw(q.x, q.y, q.z, q.w)
            except Exception:
                pass

        total = len(msg.ranges)
        if total == 0:
            self.latest_scan_points = []
            return

        step = max(1, total // 100)
        pts: List[List[float]] = []
        for i in range(0, total, step):
            r = msg.ranges[i]
            if msg.range_min < r < msg.range_max and math.isfinite(r):
                angle = msg.angle_min + i * msg.angle_increment + syaw
                px = round(sx + r * math.cos(angle), 2)
                py = round(sy + r * math.sin(angle), 2)
                pts.append([px, py])
        self.latest_scan_points = pts

    def plan_callback(self, msg: NavPath) -> None:
        total = len(msg.poses)
        if total == 0:
            self.latest_path_points = []
            return
        step = max(1, total // 50)
        self.latest_path_points = [
            [
                round(float(p.pose.position.x), 2),
                round(float(p.pose.position.y), 2),
            ]
            for p in msg.poses[::step]
        ]

    # ---------------------------------------------------------
    # Navigation, Validation & Initial Pose
    # ---------------------------------------------------------
    def validate_map_point(self, x: float, y: float):
        if self.grid_data is None or self.grid_width <= 0:
            return round(float(x), 3), round(float(y), 3), False
        return self.map_mgr.validate_and_snap_point(
            float(x),
            float(y),
            self.grid_width,
            self.grid_height,
            self.grid_resolution,
            self.grid_origin_x,
            self.grid_origin_y,
            self.grid_data,
        )

    def send_navigation_goal(
        self, x: float, y: float, yaw: float, label: str = "Goal"
    ) -> Dict[str, Any]:
        safe_x, safe_y, snapped = self.validate_map_point(x, y)

        if not self.nav_client.wait_for_server(timeout_sec=8.0):
            raise ValueError(
                "Nav2 'navigate_to_pose' server is still starting up. Please wait a few seconds and try again."
            )

        # Cancel any previous goal before sending the new goal
        self.is_teleop_active = False
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = PoseStamped()
        goal_msg.pose.header.frame_id = "map"
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = float(safe_x)
        goal_msg.pose.pose.position.y = float(safe_y)
        qx, qy, qz, qw = yaw_to_quaternion(float(yaw))
        goal_msg.pose.pose.orientation.x = qx
        goal_msg.pose.pose.orientation.y = qy
        goal_msg.pose.pose.orientation.z = qz
        goal_msg.pose.pose.orientation.w = qw

        self.nav_status = "NAVIGATING"
        self.target_name = label
        self.active_goal_pose = {
            "x": round(float(safe_x), 3),
            "y": round(float(safe_y), 3),
            "yaw": round(float(yaw), 3),
            "label": label,
        }
        self.nav_distance_remaining = round(
            math.hypot(safe_x - self.robot_pose["x"], safe_y - self.robot_pose["y"]),
            2,
        )

        self.get_logger().info(
            f"Sending NavigateToPose goal '{label}' -> (x={safe_x:.2f}, y={safe_y:.2f}, yaw={yaw:.2f})"
        )
        send_future = self.nav_client.send_goal_async(
            goal_msg, feedback_callback=self._nav_feedback_cb
        )
        send_future.add_done_callback(self._nav_goal_response_cb)

        return {
            "status": "NAVIGATING",
            "target_name": label,
            "x": safe_x,
            "y": safe_y,
            "yaw": round(float(yaw), 3),
            "snapped": snapped,
        }

    def _nav_feedback_cb(self, feedback_msg) -> None:
        try:
            self.nav_distance_remaining = round(
                float(feedback_msg.feedback.distance_remaining), 2
            )
        except Exception:
            pass

    def _nav_goal_response_cb(self, future) -> None:
        try:
            goal_handle = future.result()
            if not goal_handle.accepted:
                self.nav_status = "REJECTED"
                self.active_goal_pose = None
                self.get_logger().warn("NavigateToPose goal was rejected by server.")
                return
            self.current_goal_handle = goal_handle
            res_future = goal_handle.get_result_async()
            res_future.add_done_callback(self._nav_result_cb)
        except Exception as exc:
            self.nav_status = "FAILED"
            self.active_goal_pose = None
            self.get_logger().error(f"NavigateToPose goal response error: {exc}")

    def _nav_result_cb(self, future) -> None:
        self.latest_path_points = []
        self.nav_distance_remaining = 0.0
        self.active_goal_pose = None
        try:
            status = future.result().status
            if status == GoalStatus.STATUS_SUCCEEDED:
                self.nav_status = "SUCCEEDED"
            elif status == GoalStatus.STATUS_CANCELED:
                self.nav_status = "CANCELED"
            else:
                self.nav_status = "FAILED"
        except Exception:
            self.nav_status = "FAILED"

    def cancel_navigation(self) -> None:
        if self.current_goal_handle is not None:
            try:
                self.current_goal_handle.cancel_goal_async()
            except Exception:
                pass
            self.current_goal_handle = None
        self.nav_status = "CANCELED"
        self.target_name = ""
        self.active_goal_pose = None
        self.latest_path_points = []
        self.nav_distance_remaining = 0.0

    def publish_initial_pose(self, x: float, y: float, yaw: float) -> None:
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = "map"
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.pose.position.x = float(x)
        msg.pose.pose.position.y = float(y)
        qx, qy, qz, qw = yaw_to_quaternion(float(yaw))
        msg.pose.pose.orientation.x = qx
        msg.pose.pose.orientation.y = qy
        msg.pose.pose.orientation.z = qz
        msg.pose.pose.orientation.w = qw
        cov = [0.0] * 36
        cov[0] = 0.15
        cov[7] = 0.15
        cov[35] = 0.06
        msg.pose.covariance = cov
        self.init_pose_pub.publish(msg)
        self.robot_pose = {
            "x": round(float(x), 3),
            "y": round(float(y), 3),
            "yaw": round(float(yaw), 3),
        }

    # ---------------------------------------------------------
    # Map Saving, Loading & Mode Switching
    # ---------------------------------------------------------
    def save_current_map(
        self, map_name: str, copy_waypoints: bool = True
    ) -> Dict[str, Any]:
        if self.grid_data is None or self.grid_width <= 0:
            raise ValueError("No live or loaded map data is available to save.")

        prev_map = self.active_map_name
        clean_name = self.map_mgr.save_occupancy_grid_to_map_dirs(
            map_name,
            self.grid_width,
            self.grid_height,
            self.grid_resolution,
            self.grid_origin_x,
            self.grid_origin_y,
            self.grid_data,
        )
        if copy_waypoints and prev_map and prev_map != clean_name:
            self.map_mgr.copy_waypoints(prev_map, clean_name)

        self.active_map_name = clean_name
        self.map_mgr.set_state("active_map", clean_name)
        return {
            "status": "saved",
            "map_name": clean_name,
            "maps": self.map_mgr.list_available_maps(),
        }

    def load_saved_map(self, map_name: str) -> Dict[str, Any]:
        yaml_path = self.map_mgr.find_map_yaml(map_name)
        if yaml_path is None:
            raise FileNotFoundError(f"Map '{map_name}' was not found in alphabot_mapping/maps.")

        self.active_map_name = map_name
        self.map_mgr.set_state("active_map", map_name)
        self._try_preload_static_map(map_name)

        # If map_server is already active, call /map_server/load_map service; otherwise switch to NAVIGATION mode
        if self.load_map_client.service_is_ready() and self.mode_mgr.current_mode == "NAVIGATION":
            req = LoadMap.Request()
            req.map_url = str(yaml_path)
            self.load_map_client.call_async(req)
        else:
            self.switch_operating_mode("NAVIGATION", map_name)

        return {
            "status": "loaded",
            "mode": self.mode_mgr.current_mode,
            "active_map": self.active_map_name,
            "waypoints": self.map_mgr.list_waypoints(self.active_map_name),
        }

    def switch_operating_mode(
        self, mode: str, map_name: Optional[str] = None
    ) -> Dict[str, Any]:
        target_mode = mode.strip().upper()
        self.emergency_stop()
        is_2d_sim = "alphabot_2d_simulator" in self.get_node_names()

        if target_mode == "MAPPING":
            # Clear any previously loaded static map so live SLAM builds a fresh map
            self.grid_width = 0
            self.grid_height = 0
            self.grid_data = None
            self.latest_map_payload = None
            self.map_version += 1
            self.active_map_name = "unmapped_slam"

            if is_2d_sim:
                msg = Bool()
                msg.data = True
                self.sim_slam_mode_pub.publish(msg)
                self.mode_mgr.current_mode = "MAPPING"
            else:
                self.mode_mgr.switch_to_mapping(
                    use_sim_time=self.use_sim_time_for_switches
                )
            return {
                "status": "ok",
                "mode": "MAPPING",
                "active_map": self.active_map_name,
            }
        elif target_mode == "NAVIGATION":
            chosen_map = (map_name or self.active_map_name or "").strip()
            if not chosen_map or chosen_map in ("unmapped", "unmapped_slam"):
                avail = self.map_mgr.list_available_maps()
                if not avail:
                    raise ValueError(
                        "No saved maps exist yet! Start 'Build Map (SLAM)' and save a map first."
                    )
                chosen_map = avail[0]["name"]
            yaml_path = self.map_mgr.find_map_yaml(chosen_map)
            if yaml_path is None:
                raise FileNotFoundError(
                    f"Map '{chosen_map}' does not exist in alphabot_mapping/maps."
                )
            self.active_map_name = chosen_map
            self.map_mgr.set_state("active_map", chosen_map)
            self._try_preload_static_map(chosen_map)

            if is_2d_sim:
                msg = Bool()
                msg.data = False
                self.sim_slam_mode_pub.publish(msg)
                self.mode_mgr.current_mode = "NAVIGATION"
                if self.load_map_client.service_is_ready():
                    req = LoadMap.Request()
                    req.map_url = str(yaml_path)
                    self.load_map_client.call_async(req)
            else:
                self.mode_mgr.switch_to_navigation(
                    map_name=chosen_map,
                    use_sim_time=self.use_sim_time_for_switches,
                )
                # Auto-publish current pose after AMCL starts up
                saved_pose = dict(self.robot_pose)
                for delay_s in (3.0, 5.5):
                    threading.Timer(
                        delay_s,
                        lambda p=saved_pose: self.publish_initial_pose(
                            p["x"], p["y"], p["yaw"]
                        ),
                    ).start()
            return {
                "status": "ok",
                "mode": "NAVIGATION",
                "active_map": self.active_map_name,
            }
        else:
            raise ValueError(f"Unsupported mode '{mode}'. Choose MAPPING or NAVIGATION.")

    def get_telemetry_snapshot(self, include_map: bool = False) -> Dict[str, Any]:
        nav_ready = False
        try:
            if self.nav_client is not None:
                nav_ready = self.nav_client.server_is_ready()
        except Exception:
            pass
        if self.mode_mgr.current_mode == "STANDBY" and not nav_ready:
            effective_nav_status = "STANDBY"
        elif not nav_ready and self.nav_status == "IDLE":
            effective_nav_status = "STARTING_NAV2"
        else:
            effective_nav_status = self.nav_status

        snap: Dict[str, Any] = {
            "type": "telemetry",
            "robot_model": self.robot_model,
            "robot_radius": self.robot_radius,
            "mode": self.mode_mgr.current_mode,
            "active_map": self.active_map_name,
            "nav_ready": nav_ready,
            "nav_status": effective_nav_status,
            "target_name": self.target_name,
            "active_goal": self.active_goal_pose,
            "distance_remaining": self.nav_distance_remaining,
            "safety_locked": self.safety_locked,
            "pose": self.robot_pose,
            "velocity": self.robot_vel,
            "scan": self.latest_scan_points,
            "path": self.latest_path_points,
            "map_version": self.map_version,
            "has_map": self.latest_map_payload is not None,
        }
        if include_map and self.latest_map_payload is not None:
            snap["map"] = self.latest_map_payload
        return snap


def build_web_server(node: AlphaBotWebGatewayNode) -> AsyncHttpWsServer:
    static_dir = resolve_web_static_dir()
    server = AsyncHttpWsServer(static_dir=static_dir, host=node.host, port=node.port)

    @server.route("GET", "/api/status")
    async def api_status(body: Dict, params: Dict):
        return {
            **node.get_telemetry_snapshot(include_map=True),
            "lan_ips": get_lan_ip_addresses(),
            "port": node.port,
            "maps": node.map_mgr.list_available_maps(),
            "waypoints": node.map_mgr.list_waypoints(node.active_map_name),
        }

    @server.route("GET", "/api/maps")
    async def api_list_maps(body: Dict, params: Dict):
        return {
            "active_map": node.active_map_name,
            "maps": node.map_mgr.list_available_maps(),
        }

    @server.route("POST", "/api/maps/save")
    async def api_save_map(body: Dict, params: Dict):
        map_name = str(body.get("map_name", "")).strip()
        copy_wps = bool(body.get("copy_waypoints", True))
        return node.save_current_map(map_name, copy_waypoints=copy_wps)

    @server.route("POST", "/api/maps/load")
    async def api_load_map(body: Dict, params: Dict):
        map_name = str(body.get("map_name", "")).strip()
        return node.load_saved_map(map_name)

    @server.route("POST", "/api/mode")
    async def api_switch_mode(body: Dict, params: Dict):
        mode = str(body.get("mode", "NAVIGATION"))
        map_name = body.get("map_name")
        return node.switch_operating_mode(mode, map_name)

    @server.route("GET", "/api/waypoints")
    async def api_get_waypoints(body: Dict, params: Dict):
        return {
            "map_name": node.active_map_name,
            "waypoints": node.map_mgr.list_waypoints(node.active_map_name),
        }

    @server.route("POST", "/api/waypoints")
    async def api_save_waypoint(body: Dict, params: Dict):
        name = str(body.get("name", "")).strip()
        use_current = bool(body.get("use_current_pose", False))
        if use_current:
            x = node.robot_pose["x"]
            y = node.robot_pose["y"]
            yaw = node.robot_pose["yaw"]
            snapped = False
        else:
            if "x" not in body or "y" not in body:
                raise ValueError("Coordinates 'x' and 'y' are required.")
            x, y, snapped = node.validate_map_point(
                float(body["x"]), float(body["y"])
            )
            yaw = float(body.get("yaw", 0.0))

        wp = node.map_mgr.save_waypoint(node.active_map_name, name, x, y, yaw)
        return {
            "status": "ok",
            "snapped": snapped,
            "waypoint": wp,
            "waypoints": node.map_mgr.list_waypoints(node.active_map_name),
        }

    @server.route("DELETE", "/api/waypoints/{name}")
    async def api_delete_waypoint(body: Dict, params: Dict):
        name = params.get("name", "")
        node.map_mgr.delete_waypoint(node.active_map_name, name)
        return {
            "status": "deleted",
            "waypoints": node.map_mgr.list_waypoints(node.active_map_name),
        }

    @server.route("POST", "/api/navigate")
    async def api_navigate(body: Dict, params: Dict):
        wp_name = body.get("waypoint_name")
        if wp_name:
            wp = node.map_mgr.get_waypoint(node.active_map_name, str(wp_name))
            if wp is None:
                raise FileNotFoundError(f"Saved location '{wp_name}' not found.")
            return node.send_navigation_goal(
                wp["x"], wp["y"], wp["yaw"], label=wp["name"]
            )
        if "x" not in body or "y" not in body:
            raise ValueError("Provide either 'waypoint_name' or ('x', 'y') coordinates.")
        return node.send_navigation_goal(
            float(body["x"]),
            float(body["y"]),
            float(body.get("yaw", 0.0)),
            label=str(body.get("label", "Map Point")),
        )

    @server.route("POST", "/api/cancel_nav")
    async def api_cancel_nav(body: Dict, params: Dict):
        node.cancel_navigation()
        return {"status": "canceled"}

    @server.route("POST", "/api/initialpose")
    async def api_initialpose(body: Dict, params: Dict):
        wp_name = body.get("waypoint_name")
        if wp_name:
            wp = node.map_mgr.get_waypoint(node.active_map_name, str(wp_name))
            if wp is None:
                raise FileNotFoundError(f"Saved location '{wp_name}' not found.")
            node.publish_initial_pose(wp["x"], wp["y"], wp["yaw"])
            return {"status": "ok", "pose": wp}
        x = float(body.get("x", 0.0))
        y = float(body.get("y", 0.0))
        yaw = float(body.get("yaw", 0.0))
        node.publish_initial_pose(x, y, yaw)
        return {"status": "ok", "pose": {"x": x, "y": y, "yaw": yaw}}

    @server.route("POST", "/api/estop")
    async def api_estop(body: Dict, params: Dict):
        node.emergency_stop()
        return {"status": "stopped"}

    @server.route("POST", "/api/safety_lock")
    async def api_safety_lock(body: Dict, params: Dict):
        locked = bool(body.get("locked", False))
        node.set_safety_lock(locked)
        return {"status": "ok", "safety_locked": node.safety_locked}

    @server.websocket
    async def ws_endpoint(ws: WebSocketConnection):
        last_sent_map_ver = -1
        last_map_send_ts = 0.0

        async def receiver_loop():
            while not ws.closed:
                raw = await ws.recv_message()
                if raw is None:
                    break
                try:
                    msg = json.loads(raw)
                except Exception:
                    continue
                mtype = msg.get("type")
                if mtype == "cmd_vel":
                    node.send_teleop_velocity(
                        float(msg.get("vx", 0.0)), float(msg.get("wz", 0.0))
                    )
                elif mtype == "estop":
                    node.emergency_stop()
                elif mtype == "ping":
                    await ws.send_json({"type": "pong", "ts": msg.get("ts")})

        recv_task = asyncio.create_task(receiver_loop())
        try:
            while not ws.closed:
                now = time.monotonic()
                if node.latest_map_payload is None:
                    last_sent_map_ver = -1
                need_map = (
                    node.latest_map_payload is not None
                    and node.map_version != last_sent_map_ver
                    and (last_sent_map_ver == -1 or (now - last_map_send_ts) >= 1.0)
                )
                snap = node.get_telemetry_snapshot(include_map=need_map)
                if need_map:
                    last_sent_map_ver = node.map_version
                    last_map_send_ts = now
                await ws.send_json(snap)
                await asyncio.sleep(0.10)  # 10 Hz real-time telemetry
        finally:
            recv_task.cancel()
            if rclpy.ok():
                try:
                    node.send_teleop_velocity(0.0, 0.0)
                except Exception:
                    pass

    return server


def _cleanup_conflicting_processes(is_gazebo_sim: bool) -> None:
    """Stop any stale gateway_node holding the port, and if running Gazebo, stop stale 2D sim."""
    my_pid = os.getpid()
    proc_root = Path("/proc")
    if not proc_root.exists():
        return
    markers = ["alphabot_web_gui/lib/alphabot_web_gui/gateway_node"]
    if is_gazebo_sim:
        markers.append("alphabot_web_gui/lib/alphabot_web_gui/sim_robot_node")

    for entry in proc_root.iterdir():
        if not entry.name.isdigit():
            continue
        pid = int(entry.name)
        if pid == my_pid:
            continue
        try:
            cmdline = (
                (entry / "cmdline")
                .read_bytes()
                .replace(b"\x00", b" ")
                .decode("utf-8", errors="ignore")
            )
            if any(m in cmdline for m in markers):
                os.kill(pid, signal.SIGTERM)
        except Exception:
            continue


def main(args=None) -> None:
    rclpy.init(args=args)
    node = AlphaBotWebGatewayNode()
    _cleanup_conflicting_processes(is_gazebo_sim=node.use_sim_time_for_switches)

    def _spin_safe():
        try:
            rclpy.spin(node)
        except Exception:
            pass

    # Run rclpy spin in a dedicated background daemon thread
    spin_thread = threading.Thread(target=_spin_safe, daemon=True)
    spin_thread.start()

    loop = asyncio.new_event_loop()
    asyncio.set_event_loop(loop)
    server = build_web_server(node)
    loop.run_until_complete(server.start())

    ips = get_lan_ip_addresses()
    for ip in ips:
        node.get_logger().info(
            f"AlphaBot Web GUI ready on Local Network -> http://{ip}:{node.port}"
        )

    try:
        loop.run_forever()
    except KeyboardInterrupt:
        pass
    finally:
        node.emergency_stop()
        node.mode_mgr.shutdown()
        loop.run_until_complete(server.stop())
        try:
            node.destroy_node()
        except Exception:
            pass
        if rclpy.ok():
            try:
                rclpy.shutdown()
            except Exception:
                pass


if __name__ == "__main__":
    main()
