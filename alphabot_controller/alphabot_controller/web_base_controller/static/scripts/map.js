// const socket = io();

const canvas = document.getElementById("mapCanvas");
const ctx = canvas.getContext("2d");

let mapInfo = null;
let robotPose = null;
let goalPose = null;

canvas.addEventListener("click", (e) => {
    if (!mapInfo) return;

    const rect = canvas.getBoundingClientRect();
    const cx = e.clientX - rect.left;
    const cy = e.clientY - rect.top;

    const mapPoint = canvasToMap(cx, cy);
    goalPose = mapPoint;
    
    console.log("Goal clicked:", mapPoint);

    socket.emit("goal", mapPoint);
});

function canvasToMap(x, y) {
    const mx = x * mapInfo.resolution + mapInfo.origin_x;
    const my = (mapInfo.height - y) * mapInfo.resolution + mapInfo.origin_y;

    return { x: mx, y: my };
}

/* ---------- RECEIVE MAP ---------- */
socket.on("map", (map) => {
    mapInfo = map;
    draw();
});

/* ---------- RECEIVE ROBOT STATE ---------- */
socket.on("state", (state) => {
    robotPose = state;
    draw();
});

/* ---------- DRAW EVERYTHING ---------- */
function draw() {
    if (!mapInfo || !mapInfo.data) return;

    const w = mapInfo.width;
    const h = mapInfo.height;

    canvas.width = w;
    canvas.height = h;

    const img = ctx.createImageData(w, h);
    const d = img.data;

    // Draw map
    for (let y = 0; y < h; y++) {
        for (let x = 0; x < w; x++) {
            const rosIdx = x + (h - y - 1) * w;
            const val = mapInfo.data[rosIdx];
            const i = (y * w + x) * 4;

            if (val === -1) d[i] = d[i+1] = d[i+2] = 180;
            else if (val === 0) d[i] = d[i+1] = d[i+2] = 255;
            else d[i] = d[i+1] = d[i+2] = 0;

            d[i+3] = 255;
        }
    }

    ctx.putImageData(img, 0, 0);

    // Draw robot
    if (robotPose) drawRobot();

    if (goalPose) drawGoal();
}

/* ---------- MAP → CANVAS ---------- */
function mapToCanvas(x, y) {
    const mx = (x - mapInfo.origin_x) / mapInfo.resolution;
    const my = (y - mapInfo.origin_y) / mapInfo.resolution;

    return {
        x: mx,
        y: mapInfo.height - my
    };
}

/* ---------- DRAW ROBOT ---------- */
function drawRobot() {
    const p = mapToCanvas(robotPose.x, robotPose.y);

    // Robot body
    ctx.beginPath();
    ctx.arc(p.x, p.y, 5, 0, 2 * Math.PI);
    ctx.fillStyle = "red";
    ctx.fill();

    // Heading arrow
    const len = 15;
    ctx.beginPath();
    ctx.moveTo(p.x, p.y);
    ctx.lineTo(
        p.x + len * Math.cos(robotPose.yaw),
        p.y - len * Math.sin(robotPose.yaw)
    );
    ctx.strokeStyle = "red";
    ctx.lineWidth = 2;
    ctx.stroke();
}

function drawGoal() {
    const p = mapToCanvas(goalPose.x, goalPose.y);

    ctx.beginPath();
    ctx.arc(p.x, p.y, 6, 0, 2 * Math.PI);
    ctx.strokeStyle = "green";
    ctx.lineWidth = 2;
    ctx.stroke();
}
