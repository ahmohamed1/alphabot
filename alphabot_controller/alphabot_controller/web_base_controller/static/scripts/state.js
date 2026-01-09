const socket = io();

socket.on("state", data => {
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