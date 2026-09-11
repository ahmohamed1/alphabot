#!/usr/bin/env python3
"""Odometry geometry calibration helper for the servicebot.

The diff_drive_controller (instance name ``alphabot_controller``) computes
odometry from the wheel encoders using two geometry parameters:

    wheel_radius      (m)   -- effective rolling radius of each wheel
    wheel_separation  (m)   -- distance between the two wheel contact patches

Both are defined in ``alphabot_controller/config/servicebot/controllers.yaml``
and can be fine-tuned at launch time via the ``*_multiplier`` launch args
(see ``alphabot_firmware/launch/hardware_interface.launch.py``).

This script drives the robot through two standard tests and reports the
correction multipliers you should feed back into the launch args:

  1. STRAIGHT test
     Drive the robot straight for a known distance D (measured with a tape
     measure / laser rangefinder). The odom-reported distance D_odom will
     differ from D because of the (wrong) wheel_radius. The correction is:

         radius_multiplier = D / D_odom

     (If D_odom > D the radius is too big -> multiplier < 1.)

  2. CIRCLE test
     Drive the robot in a full circle (one wheel stationary, the other
     turning) and measure the actual circle radius R (distance from the
     stationary wheel to the robot's centre). The odom-reported radius
     R_odom differs because of the (wrong) wheel_separation. The correction
     is:

         separation_multiplier = R_odom / R

     (If R_odom > R the separation is too big -> multiplier < 1.)

Usage
-----
  # 1. Start the robot (real hardware) with the current nominal geometry:
  #      ROBOT_MODEL=servicebot ros2 launch alphabot_bringup real_robot.launch.py
  #
  # 2. Run the straight test (the script will tell you when to start/stop):
  python3 -m alphabot_firmware.calibrate_odometry straight --distance 2.0
  #
  # 3. Run the circle test:
  python3 -m alphabot_firmware.calibrate_odometry circle --radius 0.5
  #
  # 4. Apply the reported multipliers at launch:
  #      ROBOT_MODEL=servicebot ros2 launch alphabot_bringup real_robot.launch.py \
  #          wheel_separation_multiplier:=<sep_mult> \
  #          left_wheel_radius_multiplier:=<rad_mult> \
  #          right_wheel_radius_multiplier:=<rad_mult>

The script is read-only: it only subscribes to the odometry topic and prints
results. It does NOT publish cmd_vel -- you drive the robot by hand (joystick,
keyboard, or a separate terminal) so the test reflects real slip.
"""

import argparse
import math
import sys

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry


def _quat_to_yaw(q):
    """Extract yaw (rotation about Z) from a unit quaternion (x, y, z, w)."""
    sinr_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosr_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(sinr_cosp, cosr_cosp)


class OdomCalibrator(Node):
    """Subscribes to an odometry topic and accumulates distance / heading."""

    def __init__(self, odom_topic: str):
        super().__init__("odom_calibrator")
        self.odom_topic = odom_topic
        self.last_pose = None          # (x, y, yaw)
        self.last_stamp = None
        self.total_path = 0.0         # integrated path length (m)
        self.first_pose = None        # (x, y, yaw) at start
        self.msg_count = 0
        self.create_subscription(Odometry, odom_topic, self._odom_cb, 10)
        self.get_logger().info(f"Subscribed to {odom_topic}")

    def _odom_cb(self, msg: Odometry):
        p = msg.pose.pose.position
        o = msg.pose.pose.orientation
        pose = (p.x, p.y, _quat_to_yaw(o))
        if self.last_pose is not None:
            dx = pose[0] - self.last_pose[0]
            dy = pose[1] - self.last_pose[1]
            self.total_path += math.hypot(dx, dy)
        else:
            self.first_pose = pose
        self.last_pose = pose
        self.last_stamp = msg.header.stamp
        self.msg_count += 1

    def reset(self):
        self.last_pose = None
        self.first_pose = None
        self.total_path = 0.0
        self.msg_count = 0

    def snapshot(self):
        return {
            "path": self.total_path,
            "first": self.first_pose,
            "last": self.last_pose,
            "count": self.msg_count,
        }


def _wait_for_odom(node: OdomCalibrator, timeout_s: float = 5.0):
    """Spin until at least one odom message arrives (or timeout)."""
    import time
    t0 = time.time()
    while node.msg_count == 0 and (time.time() - t0) < timeout_s:
        rclpy.spin_once(node, timeout_sec=0.1)
    return node.msg_count > 0


def _drive_prompt(label: str, hint: str):
    print(f"\n=== {label} ===")
    print(hint)
    print("Press ENTER when the robot has FINISHED the manoeuvre ...")
    input()


def run_straight_test(measured_distance: float, odom_topic: str):
    node = OdomCalibrator(odom_topic)
    rclpy.spin(node)
    if not _wait_for_odom(node):
        node.get_logger().error(f"No messages on {odom_topic} within 5 s.")
        return None
    node.reset()
    _drive_prompt(
        "STRAIGHT TEST",
        "Drive the robot in a straight line for the measured distance "
        f"({measured_distance:.3f} m). Use a tape measure or a fixed laser "
        "target. Keep it as straight as you can.",
    )
    snap = node.snapshot()
    odom_distance = snap["path"]
    if odom_distance < 1e-3:
        node.get_logger().error("Odom path length ~0 -- did the robot move?")
        return None
    radius_multiplier = measured_distance / odom_distance
    print(f"\n  measured distance : {measured_distance:.4f} m")
    print(f"  odom distance     : {odom_distance:.4f} m")
    print(f"  error             : {(odom_distance/measured_distance - 1)*100:+.1f}%")
    print(f"\n  >>> left_wheel_radius_multiplier  = {radius_multiplier:.4f}")
    print(f"  >>> right_wheel_radius_multiplier = {radius_multiplier:.4f}")
    node.destroy_node()
    return radius_multiplier


def run_circle_test(measured_radius: float, odom_topic: str):
    node = OdomCalibrator(odom_topic)
    rclpy.spin(node)
    if not _wait_for_odom(node):
        node.get_logger().error(f"No messages on {odom_topic} within 5 s.")
        return None
    node.reset()
    _drive_prompt(
        "CIRCLE TEST",
        "Drive the robot in a FULL circle with one wheel stationary (turn in "
        "place around the stationary wheel). Measure the circle radius R "
        f"({measured_radius:.3f} m) = distance from the stationary wheel to "
        "the robot's centre. One full revolution (360 deg) is ideal.",
    )
    snap = node.snapshot()
    first, last = snap["first"], snap["last"]
    if first is None or last is None:
        node.get_logger().error("Not enough odom messages.")
        return None
    # Odom-reported circle radius: for a full circle the robot returns to its
    # start, so the path length ~ 2*pi*R_odom. Use the integrated path length
    # (robust to the exact start/end pose) divided by 2*pi.
    odom_path = snap["path"]
    if odom_path < 1e-3:
        node.get_logger().error("Odom path length ~0 -- did the robot turn?")
        return None
    odom_radius = odom_path / (2.0 * math.pi)
    separation_multiplier = odom_radius / measured_radius
    print(f"\n  measured circle radius : {measured_radius:.4f} m")
    print(f"  odom circle radius     : {odom_radius:.4f} m  (path {odom_path:.4f} m / 2pi)")
    print(f"  error                  : {(odom_radius/measured_radius - 1)*100:+.1f}%")
    print(f"\n  >>> wheel_separation_multiplier = {separation_multiplier:.4f}")
    node.destroy_node()
    return separation_multiplier


def main():
    parser = argparse.ArgumentParser(
        description="Calibrate servicebot odometry geometry (wheel radius / separation)."
    )
    sub = parser.add_subparsers(dest="mode", required=True)

    p_straight = sub.add_parser("straight", help="Straight-line distance test -> radius multiplier")
    p_straight.add_argument("--distance", type=float, default=2.0,
                            help="Measured straight-line distance in metres (default 2.0)")
    p_straight.add_argument("--odom-topic", default="/alphabot_controller/odom",
                            help="Odometry topic (default /alphabot_controller/odom)")

    p_circle = sub.add_parser("circle", help="Full-circle test -> separation multiplier")
    p_circle.add_argument("--radius", type=float, default=0.5,
                          help="Measured circle radius in metres (default 0.5)")
    p_circle.add_argument("--odom-topic", default="/alphabot_controller/odom",
                          help="Odometry topic (default /alphabot_controller/odom)")

    args = parser.parse_args()

    rclpy.init()
    try:
        if args.mode == "straight":
            run_straight_test(args.distance, args.odom_topic)
        elif args.mode == "circle":
            run_circle_test(args.radius, args.odom_topic)
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()
