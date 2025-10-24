import React from "react";
import { Joystick } from "react-joystick-component";
import ROSLIB from "roslib";
import Config from "../scripts/Config";


function pbulish_cmd (ros_, x, y){
    let cmd_vel = new ROSLIB.Topic({
            ros: ros_,
            name: Config.CMD_VEL_TOPIC,
            messageType: "geometry_msgs/msg/Twist"
        });

        // Create a twist message
        let twist = new ROSLIB.Message({
            linear:{
                x: y,
                y: 0,
                z: 0
            },
            angular:{
                x:0,
                y:0,
                z: -x
            }
        });
        // publish the topic
        cmd_vel.publish(twist);
}

function TeleOperation({ros}) {

    const handleMove = (event)=>{
        // console.log(event.x, event.y)
        pbulish_cmd(ros, event.x, event.y);
    };


    const handleStop = (event)=>{
        pbulish_cmd(ros,0.0, 0.0);
    };

  return (
    <>
      <h1>TeleOperation</h1>
      <Joystick
        size={150}
        baseColor="#EEEEEE"
        stickColor="#BBBBBB"
        move={handleMove}
        stop={handleStop}
      ></Joystick>
    </>
  );
}

export default TeleOperation;
