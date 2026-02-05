#!/usr/bin/env python3
"""
Auto-docking node that monitors battery state and searches for ArUco marker.
When battery is low, rotates in place to search for a charging dock marked with ArUco.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import BatteryState, Image, CameraInfo
from geometry_msgs.msg import PoseStamped, Twist
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import cv2
from cv_bridge import CvBridge
import numpy as np
import time
from enum import Enum


class DockState(Enum):
    """States for docking state machine."""
    IDLE = 0
    NAVIGATING = 1
    SEARCHING = 2
    SERVOING = 3
    CHARGING = 4
    FAILED = 5


class AutoDockNode(Node):
    def __init__(self):
        super().__init__('auto_dock_node')
        
        # Declare and get parameters
        self.declare_parameter('battery_threshold', 25.0)
        self.declare_parameter('dock_pose', [3.6, -3.6, -1.59])  # [x, y, yaw]
        self.declare_parameter('aruco_marker_size', 0.15)
        self.declare_parameter('aruco_target_id', 0)
        self.declare_parameter('camera_topic', '/camera/image_raw')
        self.declare_parameter('camera_info_topic', '/camera/camera_info')
        self.declare_parameter('search_angular_speed', 0.5)
        self.declare_parameter('search_timeout', 30.0)
        self.declare_parameter('linear_speed_max', 0.3)
        self.declare_parameter('angular_speed_max', 1.0)
        self.declare_parameter('linear_gain', 0.5)
        self.declare_parameter('angular_gain', 2.0)
        self.declare_parameter('dock_stop_distance', 0.2)
        
        self.battery_threshold = self.get_parameter('battery_threshold').value
        self.dock_pose = self.get_parameter('dock_pose').value  # [x, y, yaw]
        self.marker_size = self.get_parameter('aruco_marker_size').value
        self.target_id = self.get_parameter('aruco_target_id').value
        self.search_speed = self.get_parameter('search_angular_speed').value
        self.search_timeout = self.get_parameter('search_timeout').value
        self.linear_max = self.get_parameter('linear_speed_max').value
        self.angular_speed_max = self.get_parameter('angular_speed_max').value
        self.linear_gain = self.get_parameter('linear_gain').value
        self.angular_gain = self.get_parameter('angular_gain').value
        self.alignment_threshold = self.get_parameter('alignment_threshold').value if self.has_parameter('alignment_threshold') else 0.05
        self.stop_distance = self.get_parameter('dock_stop_distance').value
        
        # State machine
        self.state = DockState.IDLE
        self.battery_percentage = 100.0
        self.search_start_time = None
        self.error_logged = False  # Track if error has been logged
        self.nav_start_time = None  # Track navigation start time
        self.nav_in_progress = False  # Track if navigation goal has been sent
        self.is_aligned = False  # Track if robot is aligned with marker
        self.nav_retry_count = 0  # Track navigation retry attempts
        self.max_nav_retries = 3  # Maximum navigation retry attempts
        self.marker_lost_count = 0  # Track consecutive frames with lost marker
        self.max_marker_lost_frames = 5  # Require 5 lost frames before returning to search
        
        # ArUco setup
        self.bridge = CvBridge()
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)
        self.camera_matrix = None
        self.dist_coeffs = None
        self.marker_detected = False
        self.marker_rvec = None
        self.marker_tvec = None
        
        # Subscribers
        self.battery_sub = self.create_subscription(
            BatteryState,
            '/battery_state',
            self.battery_callback,
            10
        )
        
        self.image_sub = self.create_subscription(
            Image,
            self.get_parameter('camera_topic').value,
            self.image_callback,
            10
        )
        
        self.camera_info_sub = self.create_subscription(
            CameraInfo,
            self.get_parameter('camera_info_topic').value,
            self.camera_info_callback,
            10
        )
        
        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Navigator (for fallback if marker not found)
        self.navigator = BasicNavigator()
        
        # Main control timer
        self.create_timer(0.1, self.control_loop)  # 10 Hz
        
        self.get_logger().info(f'Auto-dock node started')
        self.get_logger().info(f'  Battery threshold: {self.battery_threshold}%')
        self.get_logger().info(f'  Fallback dock pose: {self.dock_pose}')
        self.get_logger().info(f'  ArUco marker size: {self.marker_size}m, ID: {self.target_id}')
    
    def battery_callback(self, msg: BatteryState):
        """Handle battery state updates."""
        self.battery_percentage = msg.percentage * 100.0
    
    def camera_info_callback(self, msg: CameraInfo):
        """Store camera calibration info."""
        if self.camera_matrix is None:
            self.camera_matrix = np.array(msg.k).reshape(3, 3)
            self.dist_coeffs = np.array(msg.d)
            self.get_logger().info('Camera calibration received')
    
    def image_callback(self, msg: Image):
        """Detect ArUco markers in images."""
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            
            # Detect markers using compatible API
            corners, ids, rejected = cv2.aruco.detectMarkers(frame, self.aruco_dict)
            
            self.marker_detected = False
            
            if ids is not None:
                for i, marker_id in enumerate(ids.flatten()):
                    if marker_id == self.target_id:
                        # Found target marker
                        corner = corners[i][0]
                        
                        # Estimate pose
                        rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
                            corners[i], self.marker_size, self.camera_matrix, self.dist_coeffs
                        )
                        
                        self.marker_rvec = rvecs[0]
                        self.marker_tvec = tvecs[0]
                        self.marker_detected = True
                        
                        self.get_logger().debug(
                            f'Marker found at distance: {np.linalg.norm(self.marker_tvec):.2f}m'
                        )
                        break
        
        except Exception as e:
            self.get_logger().warn(f'Error processing image: {str(e)}')
    
    def control_loop(self):
        """Main control loop for state machine."""
        if self.state == DockState.IDLE:
            self.tick_idle()
        elif self.state == DockState.NAVIGATING:
            self.tick_navigating()
        elif self.state == DockState.SEARCHING:
            self.tick_searching()
        elif self.state == DockState.SERVOING:
            self.tick_servoing()
        elif self.state == DockState.CHARGING:
            self.tick_charging()
        elif self.state == DockState.FAILED:
            self.tick_failed()
    
    def tick_idle(self):
        """Idle state - monitor battery."""
        self.error_logged = False  # Reset error flag
        self.nav_retry_count = 0  # Reset retry counter
        if self.battery_percentage < self.battery_threshold:
            self.get_logger().warn(
                f'Battery low ({self.battery_percentage:.1f}%). Navigating to dock...'
            )
            self.state = DockState.NAVIGATING
            self.search_start_time = time.time()
            self.marker_detected = False
    
    def tick_searching(self):
        """Searching state - rotate in place to search for marker."""
        # Check timeout
        elapsed = time.time() - self.search_start_time
        if elapsed > self.search_timeout:
            self.get_logger().error(f'Search timeout ({self.search_timeout}s). Docking failed.')
            self.state = DockState.FAILED
            return
        
        # Check if marker found
        if self.marker_detected:
            self.get_logger().info('ArUco marker found! Switching to visual servo...')
            self.state = DockState.SERVOING
            return
        
        # Continue spinning to search
        cmd = Twist()
        cmd.angular.z = self.search_speed
        self.cmd_vel_pub.publish(cmd)
    
    def tick_servoing(self):
        """Servoing state - align with marker first, then move straight."""
        if not self.marker_detected:
            self.marker_lost_count += 1
            
            # Safety stop if marker is lost
            cmd = Twist()
            self.cmd_vel_pub.publish(cmd)
            
            # Only return to search after multiple lost frames
            if self.marker_lost_count > self.max_marker_lost_frames:
                self.get_logger().warn(f'Marker lost for {self.marker_lost_count} frames. Returning to search...')
                self.state = DockState.SEARCHING
                self.search_start_time = time.time()
                self.is_aligned = False
                self.marker_lost_count = 0
            else:
                self.get_logger().debug(f'Marker lost ({self.marker_lost_count}/{self.max_marker_lost_frames}). Stopping...')
            return
        
        # Marker detected - reset lost counter
        self.marker_lost_count = 0
        
        # Get marker position in camera frame [x, y, z]
        tvec = self.marker_tvec[0]  # [x, y, z]
        
        # Calculate Euclidean distance from camera to marker
        # This is the 3D distance: sqrt(x² + y² + z²)
        distance = np.linalg.norm(tvec)
        
        # Calculate angular error (lateral offset relative to forward direction)
        angular_error = tvec[0] / distance if distance > 0 else 0
        
        self.get_logger().info(f'Visual Servo: Euclidean distance={distance:.3f}m, lateral_error={tvec[0]:.3f}m, aligned={self.is_aligned}')
        
        # Check if close enough to dock (stop at 0.3m)
        if distance <= self.stop_distance:
            self.get_logger().info(f'✓ Docking distance reached! Distance={distance:.3f}m (≤ {self.stop_distance}m). Stopping.')
            # Stop robot immediately
            cmd = Twist()
            self.cmd_vel_pub.publish(cmd)
            self.state = DockState.CHARGING
            self.is_aligned = False
            self.marker_lost_count = 0
            return
        
        cmd = Twist()
        
        # Phase 1: Align with marker (if not aligned)
        if abs(angular_error) > self.alignment_threshold:
            self.is_aligned = False
            # Only rotate to align, no forward motion
            cmd.angular.z = -min(self.angular_gain * angular_error, self.angular_speed_max)
            self.get_logger().debug(f'  Phase 1 - ALIGNING: Rotating at angular_vel={cmd.angular.z:.2f} rad/s, angular_error={angular_error:.3f}rad')
        else:
            # Phase 2: Move straight forward (aligned)
            self.is_aligned = True
            cmd.linear.x = min(self.linear_gain * distance, self.linear_max)
            self.get_logger().debug(f'  Phase 2 - MOVING FORWARD: linear_vel={cmd.linear.x:.2f} m/s (gain*distance={self.linear_gain * distance:.2f})')
        
        self.cmd_vel_pub.publish(cmd)
    
    def tick_navigating(self):
        """Navigating state - use nav2 to reach dock pose (non-blocking)."""
        try:
            # Send navigation goal once
            if not self.nav_in_progress:
                goal_pose = PoseStamped()
                goal_pose.header.frame_id = 'map'
                goal_pose.header.stamp = self.get_clock().now().to_msg()
                goal_pose.pose.position.x = float(self.dock_pose[0])
                goal_pose.pose.position.y = float(self.dock_pose[1])
                goal_pose.pose.position.z = 0.0
                
                # Set orientation (yaw)
                from math import cos, sin
                yaw = float(self.dock_pose[2])
                goal_pose.pose.orientation.x = 0.0
                goal_pose.pose.orientation.y = 0.0
                goal_pose.pose.orientation.z = sin(yaw / 2.0)
                goal_pose.pose.orientation.w = cos(yaw / 2.0)
                
                self.get_logger().info(f'Sending nav2 goal to: {self.dock_pose}')
                self.navigator.goToPose(goal_pose)
                self.nav_in_progress = True
                self.nav_start_time = time.time()
                return
            
            # Check navigation progress (non-blocking)
            elapsed = time.time() - self.nav_start_time
            if elapsed > 60.0:  # 60 second timeout
                self.get_logger().error('Navigation timeout (60s)')
                self.nav_in_progress = False
                self.nav_retry_count += 1
                
                if self.nav_retry_count < self.max_nav_retries:
                    self.get_logger().warn(f'Retrying navigation (attempt {self.nav_retry_count + 1}/{self.max_nav_retries})')
                    # Reset and retry
                    return
                else:
                    self.get_logger().error(f'Navigation failed after {self.max_nav_retries} attempts')
                    self.state = DockState.FAILED
                return
            
            # Check if task complete
            if self.navigator.isTaskComplete():
                result = self.navigator.getResult()
                if result == TaskResult.SUCCEEDED:
                    self.get_logger().info('Successfully reached dock location! Starting marker search...')
                    self.state = DockState.SEARCHING
                    self.search_start_time = time.time()
                    self.marker_detected = False
                    self.nav_in_progress = False
                    self.nav_retry_count = 0
                else:
                    self.get_logger().error(f'Navigation failed with result: {result}')
                    self.nav_in_progress = False
                    self.nav_retry_count += 1
                    
                    if self.nav_retry_count < self.max_nav_retries:
                        self.get_logger().warn(f'Retrying navigation (attempt {self.nav_retry_count + 1}/{self.max_nav_retries})')
                        # Reset and retry on next tick
                        return
                    else:
                        self.get_logger().error(f'Navigation failed after {self.max_nav_retries} attempts')
                        self.state = DockState.FAILED
            else:
                # Still navigating, log progress
                feedback = self.navigator.getFeedback()
                if feedback and elapsed > 5.0:  # Log every 5 seconds
                    self.get_logger().info(
                        f'Navigating... Distance remaining: {feedback.distance_remaining:.2f}m'
                    )
                    self.nav_start_time = time.time()  # Reset timer for next log
        
        except Exception as e:
            self.get_logger().error(f'Error during navigation: {str(e)}')
            self.state = DockState.FAILED
            self.nav_in_progress = False
    
    def tick_charging(self):
        """Charging state - robot is at dock."""
        self.get_logger().info('At charging dock. Charging...')
        # Stop movement
        cmd = Twist()
        self.cmd_vel_pub.publish(cmd)
        # In real system, send charging command here
    
    def tick_failed(self):
        """Failed state - docking failed."""
        if not self.error_logged:
            self.get_logger().error('Docking failed. Stopping robot.')
            self.error_logged = True
        
        # Stop movement
        cmd = Twist()
        self.cmd_vel_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = AutoDockNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
