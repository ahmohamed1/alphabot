const socket = io();
// make socket available to other scripts
window.socket = socket;

// Expose current robot pose for other scripts (e.g., UI modals)
window.currentRobotPose = null;

socket.on("state", data => {
    // store latest pose on window for other UI code to access
    window.currentRobotPose = {
        x: data.x,
        y: data.y,
        yaw: data.yaw
    };

    document.getElementById("linVel").textContent =
        data.linear_x.toFixed(2);

    document.getElementById("angVel").textContent =
        data.angular_z.toFixed(2);

    document.getElementById("posX").textContent =
        data.x.toFixed(2);

    document.getElementById("posY").textContent =
        data.y.toFixed(2);

    document.getElementById("yaw").textContent =
        data.yaw.toFixed(2);
});

// socket.on("state", state => {
//     const x = (state.x - map.origin_x) / map.resolution;
//     const y = (state.y - map.origin_y) / map.resolution;

//     ctx.fillStyle = "red";
//     ctx.beginPath();
//     ctx.arc(x * scale, canvas.height - y * scale, 4, 0, 2 * Math.PI);
//     ctx.fill();
// });