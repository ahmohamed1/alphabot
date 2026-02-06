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

    const linVelEl = document.getElementById("linVel");
    const angVelEl = document.getElementById("angVel");
    const posXEl = document.getElementById("posX");
    const posYEl = document.getElementById("posY");
    const yawEl = document.getElementById("yaw");

    if (linVelEl) linVelEl.textContent = data.linear_x.toFixed(2);
    if (angVelEl) angVelEl.textContent = data.angular_z.toFixed(2);
    if (posXEl) posXEl.textContent = data.x.toFixed(2);
    if (posYEl) posYEl.textContent = data.y.toFixed(2);
    if (yawEl) yawEl.textContent = data.yaw.toFixed(2);
});

socket.on("camera", data => {
    const img = document.getElementById("cameraFeed");
    if (!img || !data || !data.jpeg) return;
    img.src = "data:image/jpeg;base64," + data.jpeg;
});

// socket.on("state", state => {
//     const x = (state.x - map.origin_x) / map.resolution;
//     const y = (state.y - map.origin_y) / map.resolution;

//     ctx.fillStyle = "red";
//     ctx.beginPath();
//     ctx.arc(x * scale, canvas.height - y * scale, 4, 0, 2 * Math.PI);
//     ctx.fill();
// });