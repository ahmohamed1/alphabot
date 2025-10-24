import React, { useEffect, useState } from "react";
import ROSLIB from "roslib";
import { Row, Col } from "react-bootstrap";
import Config from "../scripts/Config";
import * as THREE from "three";

const initialState = {
  x: 0,
  y: 0,
  orientation: 0,
  linearVelocity: 0,
  angularVelocity: 0,
};

function RobotState({ ros }) {
  const [robotState, setRobotState] = useState(initialState);

  useEffect(() => {
    if (!ros) {
      return;
    }

    console.log("Connecting to ROS topics...");

    let poseSubscriber = new ROSLIB.Topic({
      ros: ros,
      name: Config.POSE_STATE_TOPIC,
      messageType: "geometry_msgs/msg/PoseWithCovarianceStamped",
    });
    const getOrientationFromQuaternion = (rosQuaternion) => {
      const q = new THREE.Quaternion(
        rosQuaternion.x,
        rosQuaternion.y,
        rosQuaternion.z,
        rosQuaternion.w
      );
      const euler = new THREE.Euler().setFromQuaternion(q);
      return euler.z * (180 / Math.PI);
    };

    poseSubscriber.subscribe((message) => {
      // console.log("Received pose:", message);

      setRobotState((prev) => ({
        ...prev,
        x: message.pose.pose.position.x.toFixed(2),
        y: message.pose.pose.position.y.toFixed(2),
        orientation: getOrientationFromQuaternion(
          message.pose.pose.orientation
        ).toFixed(2),
      }));
    });

    const velocitySubscriber = new ROSLIB.Topic({
      ros: ros,
      name: Config.POSE_STATE_TOPIC_ODOM,
      messageType: "nav_msgs/msg/Odometry",
    });

    velocitySubscriber.subscribe((message) => {
      // console.log("Received velocity:", message);

      setRobotState((prev) => ({
        ...prev,
        linearVelocity: message.twist.twist.linear.x.toFixed(2),
        angularVelocity: message.twist.twist.angular.z.toFixed(2),
      }));
    });

    return () => {
      console.log("Unsubscribing from ROS topics...");
      poseSubscriber.unsubscribe();
      velocitySubscriber.unsubscribe();
    };
  }, [ros]);

  return (
    <>
      <h2>Robot State</h2>
      <Row>
        <Col>
          <h4 className="mt-4">Position</h4>
          <p className="mt-0">x: {robotState.x}</p>
          <p className="mt-0">y: {robotState.y}</p>
          <p className="mt-0">Orientation: {robotState.orientation}°</p>
        </Col>
      </Row>
      <Row>
        <Col>
          <h4 className="mt-4">Velocity</h4>
          <p className="mt-0">Linear Velocity: {robotState.linearVelocity}</p>
          <p className="mt-0">Angular Velocity: {robotState.angularVelocity}</p>
        </Col>
      </Row>
    </>
  );
}

export default RobotState;
