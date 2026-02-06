// joystick.js
// const socket = io();

const leftBase = document.getElementById("leftJoystickBase");
const leftStick = document.getElementById("leftJoystickStick");
const rightBase = document.getElementById("rightJoystickBase");
const rightStick = document.getElementById("rightJoystickStick");

const rotValDisplay = document.getElementById("rotVal");
const speedValDisplay = document.getElementById("speedVal");

const leftState = { base: leftBase, stick: leftStick, dragging: false, pointerId: null, currentX: 0, currentY: 0 };
const rightState = { base: rightBase, stick: rightStick, dragging: false, pointerId: null, currentX: 0, currentY: 0 };

let sendInterval = null; // interval ID for continuous sending

function getBaseCenter(baseEl) {
    const rect = baseEl.getBoundingClientRect();
    return {
        x: rect.left + rect.width / 2,
        y: rect.top + rect.height / 2,
        radius: rect.width / 2
    };
}

// Send joystick values to server. Only emit when dragging unless `force` is true.
function sendJoystick(x, y, force = false) {
    if (!force && !(leftState.dragging || rightState.dragging)) return;
    const s = window.socket || (typeof socket !== 'undefined' && socket);
    if (s && s.emit) s.emit("joystick", { x: x, y: y });
}

function updateStickPosition(state, dx, dy) {
    state.stick.style.transform = `translate(calc(-50% + ${dx}px), calc(-50% + ${dy}px))`;
    state.currentX = dx;
    state.currentY = dy;
}

function updateDisplays() {
    rotValDisplay.textContent = Math.round(leftState.currentX);
    speedValDisplay.textContent = Math.round(-rightState.currentY);
}

function emitCombined(force = false) {
    const angularX = leftState.currentX;
    const linearY = -rightState.currentY; // forward is negative Y on screen
    sendJoystick(angularX, linearY, force);
}

function ensureInterval() {
    if (!sendInterval) {
        sendInterval = setInterval(() => {
            emitCombined(false);
        }, 50); // 20 Hz
    }
}

function maybeStopInterval() {
    if (!(leftState.dragging || rightState.dragging)) {
        if (sendInterval) {
            clearInterval(sendInterval);
            sendInterval = null;
        }
        emitCombined(true);
    }
}

function startDrag(e, state) {
    e.preventDefault();
    state.dragging = true;
    state.pointerId = e.pointerId;
    if (state.base.setPointerCapture) state.base.setPointerCapture(e.pointerId);
    moveStick(e, state);
    ensureInterval();
}

function stopDrag(e, state) {
    if (!state.dragging) return;
    if (e.pointerId !== state.pointerId) return;
    state.dragging = false;
    state.pointerId = null;
    updateStickPosition(state, 0, 0);
    updateDisplays();
    maybeStopInterval();
}

function moveStick(e, state) {
    if (!state.dragging) return;
    if (e.pointerId !== state.pointerId) return;

    const center = getBaseCenter(state.base);
    const clientX = e.clientX;
    const clientY = e.clientY;

    let dx = clientX - center.x;
    let dy = clientY - center.y;

    const distance = Math.sqrt(dx * dx + dy * dy);
    if (distance > center.radius) {
        dx = (dx / distance) * center.radius;
        dy = (dy / distance) * center.radius;
    }

    updateStickPosition(state, dx, dy);
    updateDisplays();
}

// Pointer events for multi-touch support
leftBase.addEventListener("pointerdown", (e) => startDrag(e, leftState));
rightBase.addEventListener("pointerdown", (e) => startDrag(e, rightState));

document.addEventListener("pointermove", (e) => {
    moveStick(e, leftState);
    moveStick(e, rightState);
});

document.addEventListener("pointerup", (e) => {
    stopDrag(e, leftState);
    stopDrag(e, rightState);
});

document.addEventListener("pointercancel", (e) => {
    stopDrag(e, leftState);
    stopDrag(e, rightState);
});
