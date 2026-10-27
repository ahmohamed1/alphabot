#!/usr/bin/env python3
"""Built-in 2D Robot Simulator for testing AlphaBot Web GUI (Teleop, SLAM Mapping, A* Nav2, LiDAR, Relocalization)."""

import heapq
import math
import os
import threading
import time
from pathlib import Path
from typing import List, Optional, Tuple

os.environ["RMW_IMPLEMENTATION"] = "rmw_fastrtps_cpp"

import numpy as np
import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import (
    PoseStamped,
    PoseWithCovarianceStamped,
    TransformStamped,
    Twist,
)
from nav2_msgs.action import NavigateToPose
from nav2_msgs.srv import LoadMap
from nav_msgs.msg import OccupancyGrid, Odometry, Path as NavPath
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool
from tf2_ros import TransformBroadcaster

from alphabot_web_gui.gateway_node import resolve_mapping_directories
from alphabot_web_gui.map_manager import (
    MapAndWaypointManager,
    quaternion_to_yaw,
    yaw_to_quaternion,
)


def normalize_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


class AlphaBot2DSimulatorNode(Node):
    """Simulates differential-drive kinematics, raycast LiDAR, progressive SLAM map building, and A* NavigateToPose."""

    def __init__(self):
        super().__init__("alphabot_2d_simulator")

        self.declare_parameter("robot_model", "servicebot")
        self.declare_parameter("map_name", "small_house")
        self.declare_parameter("initial_x", 0.0)
        self.declare_parameter("initial_y", 0.0)
        self.declare_parameter("initial_yaw", 0.0)
        self.declare_parameter("simulate_slam_fog", False)

        self.robot_model: str = str(self.get_parameter("robot_model").value)
        self.map_name: str = self.get_parameter("map_name").value
        self.x: float = float(self.get_parameter("initial_x").value)
        self.y: float = float(self.get_parameter("initial_y").value)
        self.yaw: float = float(self.get_parameter("initial_yaw").value)
        self.simulate_slam_fog: bool = bool(
            self.get_parameter("simulate_slam_fog").value
        )

        self.vx: float = 0.0
        self.wz: float = 0.0
        self.cmd_vx: float = 0.0
        self.cmd_wz: float = 0.0
        self.last_teleop_ts: float = 0.0
        self.safety_locked: bool = False

        # Load ground-truth map from alphabot_mapping/maps
        db_path = Path.home() / ".alphabot_web_gui" / "alphabot_gui.db"
        self.map_mgr = MapAndWaypointManager(db_path, resolve_mapping_directories())

        self.width = 200
        self.height = 200
        self.resolution = 0.05
        self.origin_x = -5.0
        self.origin_y = -5.0
        self.gt_grid = np.zeros((self.height, self.width), dtype=np.int8)
        self.explored_grid = np.full((self.height, self.width), -1, dtype=np.int8)
        self._load_ground_truth_map(self.map_name)

        self.cb_group = ReentrantCallbackGroup()

        # QoS
        map_qos = QoSProfile(
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        # Publishers
        self.map_pub = self.create_publisher(OccupancyGrid, "/map", map_qos)
        self.scan_pub = self.create_publisher(LaserScan, "/scan", 10)
        self.odom_pub = self.create_publisher(Odometry, "/odometry/filtered", 10)
        self.ctrl_odom_pub = self.create_publisher(
            Odometry, "/alphabot_controller/odom", 10
        )
        self.plan_pub = self.create_publisher(NavPath, "/plan", 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        # Subscribers
        self.create_subscription(
            Twist, "/key_vel", self._teleop_cb, 10, callback_group=self.cb_group
        )
        self.create_subscription(
            Twist,
            "/alphabot_controller/cmd_vel_unstamped",
            self._teleop_cb,
            10,
            callback_group=self.cb_group,
        )
        self.create_subscription(
            Bool,
            "/safety_stop",
            self._safety_cb,
            10,
            callback_group=self.cb_group,
        )
        self.create_subscription(
            PoseWithCovarianceStamped,
            "/initialpose",
            self._initialpose_cb,
            10,
            callback_group=self.cb_group,
        )
        self.create_subscription(
            Bool,
            "/sim/set_slam_mode",
            self._sim_slam_mode_cb,
            10,
            callback_group=self.cb_group,
        )

        # Service for switching maps dynamically (/map_server/load_map)
        self.create_service(
            LoadMap,
            "/map_server/load_map",
            self._load_map_srv_cb,
            callback_group=self.cb_group,
        )

        # Nav2 NavigateToPose Action Server
        self._active_goal_lock = threading.Lock()
        self._active_goal_id = 0
        self.nav_action_server = ActionServer(
            self,
            NavigateToPose,
            "navigate_to_pose",
            execute_callback=self._execute_nav_goal,
            goal_callback=self._goal_cb,
            cancel_callback=self._cancel_cb,
            callback_group=self.cb_group,
        )

        # Simulation Timers
        self._last_step_time = time.monotonic()
        self.create_timer(0.05, self._physics_step_cb, callback_group=self.cb_group)  # 20Hz physics + TF + Odom
        self.create_timer(0.15, self._lidar_step_cb, callback_group=self.cb_group)    # ~7Hz LiDAR raycast
        self.create_timer(1.00, self._publish_map_cb, callback_group=self.cb_group)   # 1Hz /map publish

        self._publish_map_cb()
        self.get_logger().info(
            f"AlphaBot 2D Simulator started on map '{self.map_name}' at ({self.x:.2f}, {self.y:.2f})"
        )

    def _load_ground_truth_map(self, map_name: str) -> bool:
        loaded = self.map_mgr.load_static_map_as_grid(map_name)
        if loaded is None:
            return False
        w, h, res, ox, oy, grid_flat, _ = loaded
        self.width = w
        self.height = h
        self.resolution = res
        self.origin_x = ox
        self.origin_y = oy
        self.gt_grid = np.asarray(grid_flat, dtype=np.int8).reshape((h, w)).copy()
        if self.simulate_slam_fog:
            self.explored_grid = np.full((h, w), -1, dtype=np.int8)
        else:
            self.explored_grid = self.gt_grid.copy()
        self.map_name = map_name
        return True

    def _teleop_cb(self, msg: Twist) -> None:
        self.cmd_vx = float(msg.linear.x)
        self.cmd_wz = float(msg.angular.z)
        self.last_teleop_ts = time.monotonic()

    def _safety_cb(self, msg: Bool) -> None:
        self.safety_locked = bool(msg.data)
        if self.safety_locked:
            self.cmd_vx = 0.0
            self.cmd_wz = 0.0
            self.vx = 0.0
            self.wz = 0.0

    def _initialpose_cb(self, msg: PoseWithCovarianceStamped) -> None:
        self.x = float(msg.pose.pose.position.x)
        self.y = float(msg.pose.pose.position.y)
        q = msg.pose.pose.orientation
        self.yaw = quaternion_to_yaw(q.x, q.y, q.z, q.w)
        self.get_logger().info(
            f"Simulator relocalized to ({self.x:.2f}, {self.y:.2f}, yaw={self.yaw:.2f})"
        )

    def _sim_slam_mode_cb(self, msg: Bool) -> None:
        self.simulate_slam_fog = bool(msg.data)
        if self.simulate_slam_fog:
            self.explored_grid = np.full(
                (self.height, self.width), -1, dtype=np.int8
            )
            self._lidar_step_cb()
        else:
            self.explored_grid = self.gt_grid.copy()
        self._publish_map_cb()

    def _load_map_srv_cb(
        self, request: LoadMap.Request, response: LoadMap.Response
    ) -> LoadMap.Response:
        yaml_path = Path(request.map_url)
        folder_name = yaml_path.parent.name
        if self._load_ground_truth_map(folder_name):
            self._publish_map_cb()
            response.result = LoadMap.Response.RESULT_SUCCESS
        else:
            response.result = LoadMap.Response.RESULT_UNDEFINED_FAILURE
        return response

    def _is_cell_occupied(self, wx: float, wy: float) -> bool:
        gx = int(math.floor((wx - self.origin_x) / self.resolution))
        gy = int(math.floor((wy - self.origin_y) / self.resolution))
        if not (0 <= gx < self.width and 0 <= gy < self.height):
            return True
        return bool(self.gt_grid[gy, gx] > 50)

    def _physics_step_cb(self) -> None:
        now = time.monotonic()
        dt = min(0.1, max(0.001, now - self._last_step_time))
        self._last_step_time = now

        # Check if manual teleop is active (within 0.4s)
        if self.safety_locked:
            self.vx = 0.0
            self.wz = 0.0
        elif (now - self.last_teleop_ts) <= 0.40:
            self.vx = self.cmd_vx
            self.wz = self.cmd_wz
        elif self._active_goal_id == 0:
            self.vx = 0.0
            self.wz = 0.0

        # Integrate differential drive kinematics with collision check
        if abs(self.vx) > 1e-4 or abs(self.wz) > 1e-4:
            next_yaw = normalize_angle(self.yaw + self.wz * dt)
            next_x = self.x + self.vx * math.cos(next_yaw) * dt
            next_y = self.y + self.vx * math.sin(next_yaw) * dt
            if not self._is_cell_occupied(next_x, next_y):
                self.x = next_x
                self.y = next_y
            self.yaw = next_yaw

        stamp = self.get_clock().now().to_msg()
        qx, qy, qz, qw = yaw_to_quaternion(self.yaw)

        # Broadcast TF: map -> odom -> base_footprint -> base_link -> laser_link
        t_map_odom = TransformStamped()
        t_map_odom.header.stamp = stamp
        t_map_odom.header.frame_id = "map"
        t_map_odom.child_frame_id = "odom"
        t_map_odom.transform.rotation.w = 1.0

        t_odom_base = TransformStamped()
        t_odom_base.header.stamp = stamp
        t_odom_base.header.frame_id = "odom"
        t_odom_base.child_frame_id = "base_footprint"
        t_odom_base.transform.translation.x = float(self.x)
        t_odom_base.transform.translation.y = float(self.y)
        t_odom_base.transform.rotation.x = qx
        t_odom_base.transform.rotation.y = qy
        t_odom_base.transform.rotation.z = qz
        t_odom_base.transform.rotation.w = qw

        t_base_link = TransformStamped()
        t_base_link.header.stamp = stamp
        t_base_link.header.frame_id = "base_footprint"
        t_base_link.child_frame_id = "base_link"
        t_base_link.transform.rotation.w = 1.0

        t_laser = TransformStamped()
        t_laser.header.stamp = stamp
        t_laser.header.frame_id = "base_link"
        t_laser.child_frame_id = "laser_link"
        t_laser.transform.rotation.w = 1.0

        self.tf_broadcaster.sendTransform(
            [t_map_odom, t_odom_base, t_base_link, t_laser]
        )

        # Publish Odometry
        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_footprint"
        odom.pose.pose.position.x = float(self.x)
        odom.pose.pose.position.y = float(self.y)
        odom.pose.pose.orientation.x = qx
        odom.pose.pose.orientation.y = qy
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw
        odom.twist.twist.linear.x = float(self.vx)
        odom.twist.twist.angular.z = float(self.wz)
        self.odom_pub.publish(odom)
        self.ctrl_odom_pub.publish(odom)

    def _lidar_step_cb(self) -> None:
        num_rays = 180
        angle_min = -math.pi
        angle_max = math.pi
        angle_inc = (angle_max - angle_min) / num_rays
        range_min = 0.15
        range_max = 6.0
        step_m = self.resolution * 0.8

        ranges: List[float] = []
        rx, ry, ryaw = self.x, self.y, self.yaw

        for i in range(num_rays):
            ray_angle = angle_min + i * angle_inc + ryaw
            cos_a = math.cos(ray_angle)
            sin_a = math.sin(ray_angle)
            dist = range_min
            hit_dist = float("inf")
            while dist < range_max:
                wx = rx + dist * cos_a
                wy = ry + dist * sin_a
                gx = int((wx - self.origin_x) / self.resolution)
                gy = int((wy - self.origin_y) / self.resolution)
                if not (0 <= gx < self.width and 0 <= gy < self.height):
                    break
                cell_val = self.gt_grid[gy, gx]
                if self.simulate_slam_fog:
                    self.explored_grid[gy, gx] = cell_val
                if cell_val > 50:
                    hit_dist = dist
                    break
                dist += step_m
            ranges.append(hit_dist)

        scan = LaserScan()
        scan.header.stamp = self.get_clock().now().to_msg()
        scan.header.frame_id = "laser_link"
        scan.angle_min = angle_min
        scan.angle_max = angle_max
        scan.angle_increment = angle_inc
        scan.range_min = range_min
        scan.range_max = range_max
        scan.ranges = ranges
        self.scan_pub.publish(scan)

    def _publish_map_cb(self) -> None:
        msg = OccupancyGrid()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        msg.info.width = int(self.width)
        msg.info.height = int(self.height)
        msg.info.resolution = float(self.resolution)
        msg.info.origin.position.x = float(self.origin_x)
        msg.info.origin.position.y = float(self.origin_y)
        msg.info.origin.orientation.w = 1.0
        grid_to_pub = (
            self.explored_grid if self.simulate_slam_fog else self.gt_grid
        )
        msg.data = grid_to_pub.ravel().tolist()
        self.map_pub.publish(msg)

    # ---------------------------------------------------------
    # A* Path Planner & NavigateToPose Action Execution
    # ---------------------------------------------------------
    def _plan_astar_path(
        self, start_x: float, start_y: float, goal_x: float, goal_y: float
    ) -> List[Tuple[float, float]]:
        # Downsample grid by 2x for fast A* with obstacle inflation
        scale = 2
        res = self.resolution * scale
        cw = max(1, self.width // scale)
        ch = max(1, self.height // scale)

        sx = int(np.clip((start_x - self.origin_x) / res, 0, cw - 1))
        sy = int(np.clip((start_y - self.origin_y) / res, 0, ch - 1))
        gx = int(np.clip((goal_x - self.origin_x) / res, 0, cw - 1))
        gy = int(np.clip((goal_y - self.origin_y) / res, 0, ch - 1))

        # Build coarse obstacle mask with 1-cell inflation (~10cm margin)
        obs = np.zeros((ch, cw), dtype=bool)
        for r in range(ch):
            for c in range(cw):
                sub = self.gt_grid[
                    r * scale : (r + 1) * scale, c * scale : (c + 1) * scale
                ]
                if np.any(sub > 50):
                    obs[r, c] = True

        inflated = obs.copy()
        for dr in (-1, 0, 1):
            for dc in (-1, 0, 1):
                if dr == 0 and dc == 0:
                    continue
                shifted = np.roll(np.roll(obs, dr, axis=0), dc, axis=1)
                inflated |= shifted
        inflated[sy, sx] = False
        inflated[gy, gx] = False

        open_heap = [(0.0, sx, sy)]
        came_from = {}
        g_score = {(sx, sy): 0.0}

        neighbors = [
            (-1, 0, 1.0),
            (1, 0, 1.0),
            (0, -1, 1.0),
            (0, 1, 1.0),
            (-1, -1, 1.414),
            (1, -1, 1.414),
            (-1, 1, 1.414),
            (1, 1, 1.414),
        ]

        found = False
        max_steps = 25000
        steps = 0
        while open_heap and steps < max_steps:
            steps += 1
            _, cx, cy = heapq.heappop(open_heap)
            if (cx, cy) == (gx, gy):
                found = True
                break
            for dx, dy, cost in neighbors:
                nx, ny = cx + dx, cy + dy
                if not (0 <= nx < cw and 0 <= ny < ch):
                    continue
                if inflated[ny, nx]:
                    continue
                tentative_g = g_score[(cx, cy)] + cost
                if tentative_g < g_score.get((nx, ny), float("inf")):
                    came_from[(nx, ny)] = (cx, cy)
                    g_score[(nx, ny)] = tentative_g
                    h = math.hypot(gx - nx, gy - ny)
                    heapq.heappush(open_heap, (tentative_g + h, nx, ny))

        if not found:
            return [(start_x, start_y), (goal_x, goal_y)]

        cells = [(gx, gy)]
        curr = (gx, gy)
        while curr in came_from:
            curr = came_from[curr]
            cells.append(curr)
        cells.reverse()

        world_pts = [
            (
                self.origin_x + (cx + 0.5) * res,
                self.origin_y + (cy + 0.5) * res,
            )
            for cx, cy in cells[::2]
        ]
        world_pts.append((goal_x, goal_y))
        return world_pts

    def _publish_nav_path(self, pts: List[Tuple[float, float]]) -> None:
        path_msg = NavPath()
        path_msg.header.stamp = self.get_clock().now().to_msg()
        path_msg.header.frame_id = "map"
        for wx, wy in pts:
            ps = PoseStamped()
            ps.header = path_msg.header
            ps.pose.position.x = float(wx)
            ps.pose.position.y = float(wy)
            ps.pose.orientation.w = 1.0
            path_msg.poses.append(ps)
        self.plan_pub.publish(path_msg)

    def _goal_cb(self, goal_request) -> GoalResponse:
        return GoalResponse.ACCEPT

    def _cancel_cb(self, goal_handle) -> CancelResponse:
        return CancelResponse.ACCEPT

    def _execute_nav_goal(self, goal_handle):
        with self._active_goal_lock:
            self._active_goal_id += 1
            my_goal_id = self._active_goal_id

        target_pose = goal_handle.request.pose.pose
        gx = float(target_pose.position.x)
        gy = float(target_pose.position.y)
        q = target_pose.orientation
        gyaw = quaternion_to_yaw(q.x, q.y, q.z, q.w)

        waypoints = self._plan_astar_path(self.x, self.y, gx, gy)
        self._publish_nav_path(waypoints)

        feedback = NavigateToPose.Feedback()
        wp_idx = 0

        while rclpy.ok():
            if goal_handle.is_cancel_requested or my_goal_id != self._active_goal_id:
                self.vx = 0.0
                self.wz = 0.0
                self._publish_nav_path([])
                if goal_handle.is_cancel_requested:
                    goal_handle.canceled()
                else:
                    goal_handle.abort()
                return NavigateToPose.Result()

            dist_to_goal = math.hypot(gx - self.x, gy - self.y)
            feedback.distance_remaining = float(dist_to_goal)
            goal_handle.publish_feedback(feedback)

            if dist_to_goal < 0.12:
                # Rotate to final heading
                yaw_err = normalize_angle(gyaw - self.yaw)
                if abs(yaw_err) < 0.12:
                    break
                self.vx = 0.0
                self.wz = float(np.clip(2.0 * yaw_err, -1.2, 1.2))
                time.sleep(0.05)
                continue

            # Advance lookahead waypoint
            while wp_idx < len(waypoints) - 1:
                wx, wy = waypoints[wp_idx]
                if math.hypot(wx - self.x, wy - self.y) < 0.25:
                    wp_idx += 1
                else:
                    break

            tx, ty = waypoints[wp_idx]
            desired_heading = math.atan2(ty - self.y, tx - self.x)
            heading_err = normalize_angle(desired_heading - self.yaw)

            # Only drive if manual teleop isn't overriding
            if (time.monotonic() - self.last_teleop_ts) > 0.40 and not self.safety_locked:
                if abs(heading_err) > 0.65:
                    self.vx = 0.05
                    self.wz = float(np.clip(2.2 * heading_err, -1.3, 1.3))
                else:
                    self.vx = float(min(0.35, max(0.10, dist_to_goal * 0.6)))
                    self.wz = float(np.clip(1.8 * heading_err, -1.2, 1.2))

            self._publish_nav_path(waypoints[wp_idx:])
            time.sleep(0.05)

        if my_goal_id == self._active_goal_id:
            self._active_goal_id = 0
        self.vx = 0.0
        self.wz = 0.0
        self._publish_nav_path([])
        goal_handle.succeed()
        return NavigateToPose.Result()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = AlphaBot2DSimulatorNode()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
