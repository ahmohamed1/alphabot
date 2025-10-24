import React, { useEffect, useState } from "react";

function MapView({ ros }) {
  const [isConnected, setIsConnected] = useState(false);

  useEffect(() => {
    if (!ros) return;

    // Define event handlers inside useEffect
    const handleConnection = () => {
      console.log("Map: ROS connection established");
      setIsConnected(true);
    };

    const handleError = (error) => {
      console.log("Map: ROS connection error:", error);
    };

    const handleClose = () => {
      console.log("Map: ROS connection closed");
      setIsConnected(false);
    };

    // Check if already connected
    if (ros.isConnected) {
      setIsConnected(true);
      console.log("Map: ROS already connected");
    } else {
      // Listen for connection event
      ros.on("connection", handleConnection);
      ros.on("error", handleError);
      ros.on("close", handleClose);
    }

    // Cleanup function
    return () => {
      if (ros) {
        // Remove listeners using the same function references
        ros.off("connection", handleConnection);
        ros.off("error", handleError);
        ros.off("close", handleClose);
      }
    };
  }, [ros]);

  useEffect(() => {
    if (isConnected && ros) {
      console.log("Map: Initializing map viewer");
      view_map();
    }
  }, [isConnected, ros]);

  const view_map = () => {
    if (!ros || !ros.isConnected) {
      console.log("Map: ROS not connected, cannot view map");
      return;
    }

    try {
      var viewer = new window.ROS2D.Viewer({
        divID: "nav_div",
        width: 640,
        height: 480,
      });
      
      var navClient = new window.NAV2D.OccupancyGridClientNav({
        ros: ros,
        rootObject: viewer.scene,
        viewer: viewer,
        actionName: "/navigate_to_pose",
        topic : "/map",
        withOrientation: true,
      });
      
      console.log("Map: Map viewer initialized successfully");
    } catch (error) {
      console.error("Map: Error initializing map viewer:", error);
    }
  };

  return (
    <div>
      <div id="nav_div">
        {!isConnected ? "Connecting to ROS..." : "Viewer"}
      </div>
    </div>
  );
}

export default MapView;