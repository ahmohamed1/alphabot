#!/usr/bin/env python3

import rclpy
import tf_transformations
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator


def create_pose_stamped(navigator, position_x, position_y, orientation_z):
    q_x, q_y, q_z, q_w = tf_transformations.quaternion_from_euler(
        0.0, 0.0, orientation_z
    )
    pose = PoseStamped()
    pose.header.frame_id = "map"
    pose.header.stamp = navigator.get_clock().now().to_msg()
    pose.pose.position.x = position_x
    pose.pose.position.y = position_y
    pose.pose.position.z = 0.0
    pose.pose.orientation.x = q_x
    pose.pose.orientation.y = q_y
    pose.pose.orientation.z = q_z
    pose.pose.orientation.w = q_w

    return pose


def main():
    rclpy.init()
    nav = BasicNavigator()

    # -- Set initial pose
    initial_pose = create_pose_stamped(nav, 0.0, 0.0, 0.0)
    nav.setInitialPose(initial_pose)
    nav.waitUntilNav2Active()

    goal_pose_1 = create_pose_stamped(nav, 2.5, 1.0, 1.57)
    goal_pose_2 = create_pose_stamped(nav, 2.0, 2.0, 0.0)
    goal_pose_3 = create_pose_stamped(nav, 3.5, -1.0, 0.)

    # --- for single got to point
    nav.goToPose(goal_pose_1)
    while not nav.isTaskComplete():
        feedback = nav.getFeedback()
        print(feedback)

    # --- follow waypoints
    waypoints = [goal_pose_1, goal_pose_2, goal_pose_3]
    nav.followWaypoints(waypoints)
    while not nav.isTaskComplete():
        feedback = nav.getFeedback()
        print(feedback)

    rclpy.shutdown()


if __name__ == "__main__":
    main()
