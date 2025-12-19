// joystick.js
// const socket = io();

const joystickBase = document.getElementById("joystickBase");
const joystickStick = document.getElementById("joystickStick");
const xValDisplay = document.getElementById("xVal");
const yValDisplay = document.getElementById("yVal");

let dragging = false;
let currentX = 0;
let currentY = 0;
let sendInterval = null; // interval ID for continuous sending

// Get center coordinates and radius of joystick base
function getBaseCenter() {
    const rect = joystickBase.getBoundingClientRect();
    return {
        x: rect.left + rect.width / 2,
        y: rect.top + rect.height / 2,
        radius: rect.width / 2
    };
}

// Send joystick values to server
function sendJoystick(x, y) {
    socket.emit("joystick", { x: x, y: y });
}

// Update stick position and X/Y display
function updateStick(x, y) {
    const stickRect = joystickStick.getBoundingClientRect();
    const offsetX = stickRect.width / 2;
    const offsetY = stickRect.height / 2;

    joystickStick.style.transform = `translate(${x - offsetX}px, ${y - offsetY}px)`;

    xValDisplay.textContent = Math.round(x);
    yValDisplay.textContent = Math.round(-y); // invert Y for forward

    currentX = x;
    currentY = y;
}

// Mouse/touch events
function startDrag(e) {
    e.preventDefault();
    dragging = true;

    // Start sending continuously
    if (!sendInterval) {
        sendInterval = setInterval(() => {
            sendJoystick(currentX, -currentY); // keep sending while pressed
        }, 50); // 20 Hz
    }
}

function stopDrag() {
    dragging = false;
    updateStick(0, 0); // return stick to center

    // Stop sending
    if (sendInterval) {
        clearInterval(sendInterval);
        sendInterval = null;
        sendJoystick(0, 0); // final stop
    }
}

function moveStick(e) {
    if (!dragging) return;

    const center = getBaseCenter();
    let clientX = e.clientX || e.touches[0].clientX;
    let clientY = e.clientY || e.touches[0].clientY;

    let dx = clientX - center.x;
    let dy = clientY - center.y;

    // Limit to joystick radius
    let distance = Math.sqrt(dx * dx + dy * dy);
    if (distance > center.radius) {
        dx = dx / distance * center.radius;
        dy = dy / distance * center.radius;
    }

    updateStick(dx, dy);
}

// Event listeners
joystickBase.addEventListener("mousedown", startDrag);
joystickBase.addEventListener("touchstart", startDrag);

document.addEventListener("mouseup", stopDrag);
document.addEventListener("touchend", stopDrag);

document.addEventListener("mousemove", moveStick);
document.addEventListener("touchmove", moveStick);
