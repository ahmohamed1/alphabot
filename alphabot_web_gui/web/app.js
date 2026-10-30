/* AlphaBot ROS 2 Local Network Web GUI Frontend Logic */

let ws = null;
let mapMeta = null;
let mapImage = new Image();
let robotPose = { x: 0, y: 0, yaw: 0 };
let robotVel = { vx: 0, wz: 0 };
let scanPoints = [];
let pathPoints = [];
let waypoints = [];
let activeGoal = null;
let availableMaps = [];
let safetyLocked = false;
let currentMode = "STANDBY";
let activeMapName = "unmapped";
let robotModelName = "servicebot";
let robotRadiusMeters = 0.22;

// Canvas & Map Interaction State
const canvas = document.getElementById("mapCanvas");
const ctx = canvas.getContext("2d");
let view = { scale: 2.0, offsetX: 0, offsetY: 0, initialized: false };
let currentTool = "GOAL"; // 'GOAL' | 'SAVE_WP' | 'INIT_POSE' | 'PAN'
let dragState = {
  active: false,
  startCanvasX: 0,
  startCanvasY: 0,
  currCanvasX: 0,
  currCanvasY: 0,
  startWorldX: 0,
  startWorldY: 0,
  panStartOffsetX: 0,
  panStartOffsetY: 0,
};
let pendingWaypointPose = null;

function showToast(message) {
  const el = document.getElementById("toast");
  el.textContent = message;
  el.style.display = "block";
  clearTimeout(showToast._timer);
  showToast._timer = setTimeout(() => {
    el.style.display = "none";
  }, 3500);
}

// ==========================================
// 1. Initial Status & WebSocket Connection
// ==========================================
async function fetchInitialStatus() {
  try {
    const res = await fetch("/api/status");
    if (!res.ok) return;
    const data = await res.json();
    if (data.lan_ips && data.lan_ips.length > 0) {
      document.getElementById("lanUrlText").textContent =
        `LAN URL: http://${data.lan_ips[0]}:${data.port}`;
    }
    if (data.maps) updateMapsDropdown(data.maps, data.active_map);
    if (data.waypoints) renderWaypointsList(data.waypoints);
    applyTelemetry(data);
  } catch (err) {
    console.warn("Initial status fetch failed:", err);
  }
}

function connectWebSocket() {
  const proto = location.protocol === "https:" ? "wss:" : "ws:";
  ws = new WebSocket(`${proto}//${location.host}/ws`);

  ws.onopen = () => {
    document.getElementById("connDot").classList.add("online");
    fetchInitialStatus();
  };

  ws.onclose = () => {
    document.getElementById("connDot").classList.remove("online");
    setTimeout(connectWebSocket, 1500);
  };

  ws.onmessage = (event) => {
    try {
      const data = JSON.parse(event.data);
      if (data.type === "telemetry") {
        applyTelemetry(data);
      }
    } catch (e) {
      console.error("WS parse error:", e);
    }
  };
}

function applyTelemetry(data) {
  currentMode = data.mode || currentMode;
  activeMapName = data.active_map || activeMapName;
  safetyLocked = Boolean(data.safety_locked);
  if (data.robot_model) {
    robotModelName = data.robot_model;
    const modelEl = document.getElementById("robotModelText");
    if (modelEl) modelEl.textContent = robotModelName;
  }
  if (data.robot_radius) {
    robotRadiusMeters = Number(data.robot_radius);
  }

  // Update Header UI
  const modeBadge = document.getElementById("modeBadge");
  modeBadge.textContent = currentMode;
  modeBadge.className =
    "badge " + (currentMode === "MAPPING" ? "mode-map" : "mode-nav");

  document.getElementById("btnModeMapping").classList.toggle(
    "btn-active",
    currentMode === "MAPPING"
  );
  document.getElementById("btnModeNav").classList.toggle(
    "btn-active",
    currentMode === "NAVIGATION"
  );

  document.getElementById("activeMapText").textContent = activeMapName;

  const navText = data.target_name
    ? `${data.nav_status} -> ${data.target_name} (${(data.distance_remaining || 0).toFixed(2)}m)`
    : data.nav_status || "IDLE";
  document.getElementById("navStatusBadge").textContent = navText;

  if (data.pose) {
    robotPose = data.pose;
    const deg = Math.round((robotPose.yaw * 180) / Math.PI);
    document.getElementById("poseText").textContent =
      `${robotPose.x.toFixed(2)}, ${robotPose.y.toFixed(2)}, ${deg}°`;
  }

  if (data.velocity) {
    robotVel = data.velocity;
    document.getElementById("velText").textContent =
      `${robotVel.vx.toFixed(2)} m/s | ${robotVel.wz.toFixed(2)} rad/s`;
  }

  const lockBtn = document.getElementById("btnSafetyLock");
  lockBtn.textContent = safetyLocked ? "🔒 Lock: ON" : "🔓 Lock: OFF";
  lockBtn.classList.toggle("locked", safetyLocked);

  scanPoints = data.scan || [];
  pathPoints = data.path || [];
  activeGoal = data.active_goal || null;

  if (data.has_map === false) {
    mapMeta = null;
    view.initialized = false;
  } else if (data.map) {
    const prevW = mapMeta ? mapMeta.width : 0;
    const prevH = mapMeta ? mapMeta.height : 0;
    mapMeta = data.map;
    mapImage.onload = () => {
      if (!view.initialized || prevW !== mapMeta.width || prevH !== mapMeta.height) {
        fitMapToScreen();
      }
    };
    mapImage.src = "data:image/png;base64," + data.map.image_b64;
  }
}

// ==========================================
// Mobile Tab / Sheet Switcher & Joystick Toggle
// ==========================================
function setMobileTab(tab) {
  const sidebar = document.getElementById("controlSidebar");
  const panels = sidebar.querySelectorAll(".panel");
  document.getElementById("mobTabMap").classList.toggle("active", tab === "map");
  document.getElementById("mobTabLocations").classList.toggle("active", tab === "locations");
  document.getElementById("mobTabMode").classList.toggle("active", tab === "mode");

  if (tab === "map") {
    sidebar.classList.remove("mobile-open");
  } else {
    sidebar.classList.add("mobile-open");
    panels.forEach((p) => {
      p.style.display = p.dataset.mobileTab === tab ? "flex" : "none";
    });
  }
  setTimeout(fitMapToScreen, 60);
}

function toggleJoystickCollapse() {
  const body = document.getElementById("teleopBody");
  const btn = document.getElementById("joyToggleBtn");
  const collapsed = body.classList.toggle("collapsed");
  btn.textContent = collapsed ? "+" : "−";
}

// ==========================================
// 2. Coordinate Conversions (ROS Map <-> Canvas)
// ==========================================
function worldToMapPixel(wx, wy) {
  const px = (wx - mapMeta.origin_x) / mapMeta.resolution;
  const py = mapMeta.height - (wy - mapMeta.origin_y) / mapMeta.resolution;
  return [px, py];
}

function canvasToWorld(cx, cy) {
  const px = (cx - view.offsetX) / view.scale;
  const py = (cy - view.offsetY) / view.scale;
  const wx = mapMeta.origin_x + px * mapMeta.resolution;
  const wy = mapMeta.origin_y + (mapMeta.height - py) * mapMeta.resolution;
  return [wx, wy];
}

function fitMapToScreen() {
  if (!mapMeta) return;
  const stage = canvas.parentElement;
  if (!stage || stage.clientWidth < 20 || stage.clientHeight < 20) return;
  canvas.width = stage.clientWidth;
  canvas.height = stage.clientHeight;
  const scaleX = (canvas.width * 0.85) / mapMeta.width;
  const scaleY = (canvas.height * 0.85) / mapMeta.height;
  view.scale = Math.max(0.15, Math.min(scaleX, scaleY));
  view.offsetX = (canvas.width - mapMeta.width * view.scale) / 2;
  view.offsetY = (canvas.height - mapMeta.height * view.scale) / 2;
  view.initialized = true;
}

function zoomMap(factor, centerX = canvas.width / 2, centerY = canvas.height / 2) {
  const newScale = Math.max(0.15, Math.min(25.0, view.scale * factor));
  const actualFactor = newScale / view.scale;
  view.offsetX = centerX - (centerX - view.offsetX) * actualFactor;
  view.offsetY = centerY - (centerY - view.offsetY) * actualFactor;
  view.scale = newScale;
}

window.addEventListener("resize", () => {
  fitMapToScreen();
  if (window.innerWidth > 850) {
    const sidebar = document.getElementById("controlSidebar");
    sidebar.querySelectorAll(".panel").forEach((p) => (p.style.display = "flex"));
  }
});

if (typeof ResizeObserver !== "undefined") {
  const ro = new ResizeObserver(() => {
    if (!view.initialized && mapMeta) {
      fitMapToScreen();
    }
  });
  ro.observe(document.getElementById("mapStage") || canvas.parentElement);
}

// ==========================================
// 3. Interactive Canvas Rendering (30+ FPS)
// ==========================================
function renderLoop() {
  if (
    canvas.width !== canvas.parentElement.clientWidth ||
    canvas.height !== canvas.parentElement.clientHeight
  ) {
    canvas.width = canvas.parentElement.clientWidth;
    canvas.height = canvas.parentElement.clientHeight;
  }

  ctx.clearRect(0, 0, canvas.width, canvas.height);

  if (mapMeta && mapImage.complete && mapImage.naturalWidth > 0) {
    ctx.save();
    ctx.translate(view.offsetX, view.offsetY);
    ctx.scale(view.scale, view.scale);
    ctx.imageSmoothingEnabled = false;

    // 1. OccupancyGrid Map Image
    ctx.drawImage(mapImage, 0, 0);

    // 2. Nav2 Planned Global Path
    if (pathPoints.length > 1) {
      ctx.strokeStyle = "#a855f7";
      ctx.lineWidth = Math.max(1.5, 3.0 / view.scale);
      ctx.beginPath();
      pathPoints.forEach(([wx, wy], i) => {
        const [px, py] = worldToMapPixel(wx, wy);
        if (i === 0) ctx.moveTo(px, py);
        else ctx.lineTo(px, py);
      });
      ctx.stroke();
    }

    // 3. Live LiDAR Scan Points (/scan)
    ctx.fillStyle = "#ef4444";
    const ptSize = Math.max(1.2, 3.0 / view.scale);
    scanPoints.forEach(([wx, wy]) => {
      const [px, py] = worldToMapPixel(wx, wy);
      ctx.fillRect(px - ptSize / 2, py - ptSize / 2, ptSize, ptSize);
    });

    // 4. Saved Named Locations (Waypoints)
    waypoints.forEach((wp) => {
      const [px, py] = worldToMapPixel(wp.x, wp.y);
      const r = Math.max(3.0, 6.5 / view.scale);

      // Heading arrow
      ctx.save();
      ctx.translate(px, py);
      ctx.rotate(-wp.yaw);
      ctx.strokeStyle = "#22c55e";
      ctx.lineWidth = Math.max(1.2, 2.2 / view.scale);
      ctx.beginPath();
      ctx.moveTo(0, 0);
      ctx.lineTo(r * 2.2, 0);
      ctx.stroke();
      ctx.restore();

      // Pin circle
      ctx.fillStyle = "#22c55e";
      ctx.beginPath();
      ctx.arc(px, py, r, 0, Math.PI * 2);
      ctx.fill();

      // Label
      ctx.fillStyle = "#0f172a";
      ctx.strokeStyle = "#f8fafc";
      ctx.lineWidth = 2.5 / view.scale;
      const fontPx = Math.max(8, Math.round(12 / view.scale));
      ctx.font = `bold ${fontPx}px sans-serif`;
      ctx.strokeText(wp.name, px + r * 1.4, py + r * 0.5);
      ctx.fillText(wp.name, px + r * 1.4, py + r * 0.5);
    });

    // 4b. Active Navigation Goal Target Ring
    if (activeGoal) {
      const [gx, gy] = worldToMapPixel(activeGoal.x, activeGoal.y);
      const gr = Math.max(5.0, 9.0 / view.scale);
      ctx.save();
      ctx.translate(gx, gy);
      ctx.rotate(-(activeGoal.yaw || 0));
      ctx.strokeStyle = "#f59e0b";
      ctx.lineWidth = Math.max(1.8, 3.0 / view.scale);
      ctx.beginPath();
      ctx.arc(0, 0, gr, 0, Math.PI * 2);
      ctx.stroke();
      ctx.beginPath();
      ctx.moveTo(0, 0);
      ctx.lineTo(gr * 2.2, 0);
      ctx.stroke();
      ctx.restore();
    }

    // 5. Robot Footprint & Heading Arrow
    const [rx, ry] = worldToMapPixel(robotPose.x, robotPose.y);
    const robotRadiusPx = Math.max(4.5, robotRadiusMeters / mapMeta.resolution);
    ctx.save();
    ctx.translate(rx, ry);
    ctx.rotate(-robotPose.yaw);

    ctx.fillStyle = "rgba(56, 189, 248, 0.88)";
    ctx.strokeStyle = "#082f49";
    ctx.lineWidth = Math.max(1.0, 2.0 / view.scale);
    ctx.beginPath();
    ctx.arc(0, 0, robotRadiusPx, 0, Math.PI * 2);
    ctx.fill();
    ctx.stroke();

    // Direction nose
    ctx.strokeStyle = "#ffffff";
    ctx.lineWidth = Math.max(1.5, 2.5 / view.scale);
    ctx.beginPath();
    ctx.moveTo(0, 0);
    ctx.lineTo(robotRadiusPx * 1.6, 0);
    ctx.stroke();
    ctx.restore();

    ctx.restore();

    // 6. Active Click-and-Drag Vector Overlay (Goal / Save Waypoint / 2D Pose Estimate)
    if (dragState.active && currentTool !== "PAN") {
      ctx.save();
      const color =
        currentTool === "GOAL"
          ? "#38bdf8"
          : currentTool === "SAVE_WP"
          ? "#22c55e"
          : "#f59e0b";
      ctx.strokeStyle = color;
      ctx.fillStyle = color;
      ctx.lineWidth = 3;
      ctx.beginPath();
      ctx.arc(dragState.startCanvasX, dragState.startCanvasY, 7, 0, Math.PI * 2);
      ctx.fill();

      ctx.beginPath();
      ctx.moveTo(dragState.startCanvasX, dragState.startCanvasY);
      ctx.lineTo(dragState.currCanvasX, dragState.currCanvasY);
      ctx.stroke();
      ctx.restore();
    }
  } else {
    // Live Metric LiDAR Radar View when in STANDBY (First-Time Environment) or awaiting SLAM /map
    const cx = canvas.width / 2;
    const cy = canvas.height / 2;
    const pxPerMeter = Math.min(canvas.width, canvas.height) / 8.0;

    ctx.save();
    // Draw radar range rings (1m, 2m, 3m)
    ctx.strokeStyle = "rgba(148, 163, 184, 0.16)";
    ctx.fillStyle = "rgba(148, 163, 184, 0.45)";
    ctx.font = "11px sans-serif";
    [1, 2, 3].forEach((m) => {
      ctx.beginPath();
      ctx.arc(cx, cy, m * pxPerMeter, 0, Math.PI * 2);
      ctx.stroke();
      ctx.fillText(`${m}m`, cx + m * pxPerMeter + 4, cy - 4);
    });

    // Draw live LiDAR scan points relative to robot
    ctx.fillStyle = "#ef4444";
    scanPoints.forEach(([wx, wy]) => {
      const sx = cx + (wx - robotPose.x) * pxPerMeter;
      const sy = cy - (wy - robotPose.y) * pxPerMeter;
      ctx.fillRect(sx - 2, sy - 2, 4, 4);
    });

    // Draw robot at center
    ctx.translate(cx, cy);
    ctx.rotate(-robotPose.yaw);
    const rPx = Math.max(8, robotRadiusMeters * pxPerMeter);
    ctx.fillStyle = "rgba(56, 189, 248, 0.88)";
    ctx.beginPath();
    ctx.arc(0, 0, rPx, 0, Math.PI * 2);
    ctx.fill();
    ctx.strokeStyle = "#ffffff";
    ctx.lineWidth = 2.5;
    ctx.beginPath();
    ctx.moveTo(0, 0);
    ctx.lineTo(rPx * 1.6, 0);
    ctx.stroke();
    ctx.restore();

    ctx.fillStyle = "#cbd5e1";
    ctx.font = "bold 14px sans-serif";
    ctx.textAlign = "center";
    const statusMsg =
      currentMode === "MAPPING"
        ? "Starting SLAM... Drive with the joystick to build the map!"
        : "First-Time / Standby Mode: Tap '🗺️ Build Map (SLAM)' or Load a Saved Map";
    ctx.fillText(statusMsg, canvas.width / 2, 78);
  }

  requestAnimationFrame(renderLoop);
}

// ==========================================
// 4. Canvas Pointer Events (Pan, Zoom, Goal, Save WP, 2D Pose)
// ==========================================
function setMapTool(tool) {
  currentTool = tool;
  document.getElementById("toolGoal").classList.toggle("active", tool === "GOAL");
  document.getElementById("toolSaveWp").classList.toggle("active", tool === "SAVE_WP");
  document.getElementById("toolInitPose").classList.toggle("active", tool === "INIT_POSE");
  document.getElementById("toolPan").classList.toggle("active", tool === "PAN");
  canvas.style.cursor = tool === "PAN" ? "grab" : "crosshair";
}

canvas.addEventListener("wheel", (e) => {
  e.preventDefault();
  const rect = canvas.getBoundingClientRect();
  const cx = e.clientX - rect.left;
  const cy = e.clientY - rect.top;
  zoomMap(e.deltaY < 0 ? 1.15 : 0.87, cx, cy);
});

canvas.addEventListener("pointerdown", (e) => {
  if (!mapMeta) return;
  const rect = canvas.getBoundingClientRect();
  const cx = e.clientX - rect.left;
  const cy = e.clientY - rect.top;
  const [wx, wy] = canvasToWorld(cx, cy);

  dragState.active = true;
  dragState.startCanvasX = cx;
  dragState.startCanvasY = cy;
  dragState.currCanvasX = cx;
  dragState.currCanvasY = cy;
  dragState.startWorldX = wx;
  dragState.startWorldY = wy;
  dragState.panStartOffsetX = view.offsetX;
  dragState.panStartOffsetY = view.offsetY;
  canvas.setPointerCapture(e.pointerId);
});

canvas.addEventListener("pointermove", (e) => {
  if (!mapMeta) return;
  const rect = canvas.getBoundingClientRect();
  const cx = e.clientX - rect.left;
  const cy = e.clientY - rect.top;
  const [wx, wy] = canvasToWorld(cx, cy);
  document.getElementById("cursorCoords").textContent =
    `X: ${wx.toFixed(2)} m, Y: ${wy.toFixed(2)} m`;

  if (!dragState.active) return;
  dragState.currCanvasX = cx;
  dragState.currCanvasY = cy;

  if (currentTool === "PAN" || e.buttons === 4 || e.buttons === 2) {
    view.offsetX = dragState.panStartOffsetX + (cx - dragState.startCanvasX);
    view.offsetY = dragState.panStartOffsetY + (cy - dragState.startCanvasY);
  }
});

canvas.addEventListener("pointerup", async (e) => {
  if (!dragState.active || !mapMeta) return;
  dragState.active = false;

  if (currentTool === "PAN") return;

  const dx = dragState.currCanvasX - dragState.startCanvasX;
  const dy = dragState.currCanvasY - dragState.startCanvasY;
  // In Canvas, -dy is +Y in ROS map frame
  const yaw = Math.hypot(dx, dy) > 8 ? Math.atan2(-dy, dx) : 0.0;
  const wx = dragState.startWorldX;
  const wy = dragState.startWorldY;

  if (currentTool === "GOAL") {
    // If user clicked directly on/near a saved waypoint pin without dragging, go to that waypoint
    if (Math.hypot(dx, dy) <= 8 && waypoints.length > 0) {
      const hitWp = waypoints.find(
        (w) => mathDist(w.x, w.y, wx, wy) <= Math.max(0.25, 12 * mapMeta.resolution / view.scale)
      );
      if (hitWp) {
        await goToSavedWaypoint(hitWp.name);
        return;
      }
    }
    await sendNavigateGoal({ x: wx, y: wy, yaw, label: "Map Goal" });
  } else if (currentTool === "INIT_POSE") {
    await sendInitialPose({ x: wx, y: wy, yaw });
    setMapTool("GOAL");
  } else if (currentTool === "SAVE_WP") {
    openWpModal(wx, wy, yaw);
  }
});

// ==========================================
// 5. REST API Actions (Maps, Modes, Waypoints, Navigation)
// ==========================================
function updateMapsDropdown(maps, active) {
  availableMaps = maps;
  const sel = document.getElementById("mapSelect");
  sel.innerHTML = "";
  maps.forEach((m) => {
    const opt = document.createElement("option");
    opt.value = m.name;
    opt.textContent = m.name;
    if (m.name === active) opt.selected = true;
    sel.appendChild(opt);
  });
}

function renderWaypointsList(list) {
  waypoints = list || [];
  document.getElementById("wpCount").textContent = waypoints.length;
  const mobCount = document.getElementById("mobWpCount");
  if (mobCount) mobCount.textContent = waypoints.length;

  const listEl = document.getElementById("wpList");
  const relocSel = document.getElementById("relocSelect");
  const quickStrip = document.getElementById("quickWpStrip");
  listEl.innerHTML = "";
  relocSel.innerHTML = "";
  if (quickStrip) quickStrip.innerHTML = "";

  if (waypoints.length === 0) {
    listEl.innerHTML = `<small class="hint">No saved locations for "${activeMapName}" yet.</small>`;
    return;
  }

  waypoints.forEach((wp) => {
    const deg = Math.round((wp.yaw * 180) / Math.PI);
    const safeName = wp.name.replace(/'/g, "\\'");
    const row = document.createElement("div");
    row.className = "wp-row";
    row.innerHTML = `
      <div class="wp-info">
        <strong>📍 ${wp.name}</strong>
        <small>(${wp.x.toFixed(2)}m, ${wp.y.toFixed(2)}m, ${deg}°)</small>
      </div>
      <div class="wp-btns">
        <button class="btn-go" onclick="goToSavedWaypoint('${safeName}')">Go</button>
        <button class="btn-del" onclick="deleteSavedWaypoint('${safeName}')">✕</button>
      </div>
    `;
    listEl.appendChild(row);

    if (quickStrip) {
      const chip = document.createElement("button");
      chip.className = "quick-wp-chip";
      chip.textContent = `🚀 ${wp.name}`;
      chip.onclick = () => goToSavedWaypoint(wp.name);
      quickStrip.appendChild(chip);
    }

    const opt = document.createElement("option");
    opt.value = wp.name;
    opt.textContent = wp.name;
    relocSel.appendChild(opt);
  });
}

function mathDist(x1, y1, x2, y2) {
  return Math.hypot(x2 - x1, y2 - y1);
}

async function sendNavigateGoal(payload) {
  const res = await fetch("/api/navigate", {
    method: "POST",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify(payload),
  });
  const data = await res.json();
  if (!res.ok) {
    showToast("⚠️ " + (data.detail || "Navigation request failed"));
    return;
  }
  showToast(
    data.snapped
      ? `🚀 Navigating to "${data.target_name}" (adjusted to nearest obstacle-free point)`
      : `🚀 Navigating to "${data.target_name}"`
  );
}

async function goToSavedWaypoint(name) {
  await sendNavigateGoal({ waypoint_name: name });
}

async function saveCurrentRobotPose() {
  const input = document.getElementById("wpNameInput");
  const name = input.value.trim();
  if (!name) {
    showToast("⚠️ Please enter a location name first.");
    return;
  }
  const res = await fetch("/api/waypoints", {
    method: "POST",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify({ name, use_current_pose: true }),
  });
  const data = await res.json();
  if (!res.ok) {
    showToast("⚠️ " + data.detail);
    return;
  }
  input.value = "";
  renderWaypointsList(data.waypoints);
  showToast(`✅ Saved robot's current location as "${name}"`);
}

function openWpModal(x, y, yaw) {
  pendingWaypointPose = { x, y, yaw };
  const deg = Math.round((yaw * 180) / Math.PI);
  document.getElementById("wpModalCoords").textContent =
    `Map coordinates: X=${x.toFixed(2)}m, Y=${y.toFixed(2)}m, Heading=${deg}°`;
  const input = document.getElementById("wpModalInput");
  input.value = "";
  document.getElementById("wpModal").classList.remove("hidden");
  input.focus();
}

function closeWpModal() {
  pendingWaypointPose = null;
  document.getElementById("wpModal").classList.add("hidden");
}

async function confirmWpModal() {
  if (!pendingWaypointPose) return;
  const name = document.getElementById("wpModalInput").value.trim();
  if (!name) {
    showToast("⚠️ Location name cannot be empty.");
    return;
  }
  const res = await fetch("/api/waypoints", {
    method: "POST",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify({
      name,
      x: pendingWaypointPose.x,
      y: pendingWaypointPose.y,
      yaw: pendingWaypointPose.yaw,
      use_current_pose: false,
    }),
  });
  const data = await res.json();
  if (!res.ok) {
    showToast("⚠️ " + data.detail);
    return;
  }
  closeWpModal();
  renderWaypointsList(data.waypoints);
  setMapTool("GOAL");
  showToast(
    data.snapped
      ? `✅ Saved "${name}" (snapped to nearest safe cell)`
      : `✅ Saved location "${name}"`
  );
}

async function deleteSavedWaypoint(name) {
  const res = await fetch(`/api/waypoints/${encodeURIComponent(name)}`, {
    method: "DELETE",
  });
  const data = await res.json();
  if (res.ok) {
    renderWaypointsList(data.waypoints);
    showToast(`🗑️ Deleted location "${name}"`);
  }
}

async function sendInitialPose(payload) {
  const res = await fetch("/api/initialpose", {
    method: "POST",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify(payload),
  });
  const data = await res.json();
  if (res.ok) {
    showToast("🎯 Published 2D Pose Estimate (/initialpose)");
  } else {
    showToast("⚠️ " + data.detail);
  }
}

async function relocalizeAtSelected() {
  const name = document.getElementById("relocSelect").value;
  if (!name) return;
  await sendInitialPose({ waypoint_name: name });
}

async function saveCurrentMap() {
  const nameInput = document.getElementById("newMapName");
  const map_name = nameInput.value.trim() || activeMapName;
  const res = await fetch("/api/maps/save", {
    method: "POST",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify({ map_name, copy_waypoints: true }),
  });
  const data = await res.json();
  if (!res.ok) {
    showToast("⚠️ " + data.detail);
    return;
  }
  nameInput.value = "";
  updateMapsDropdown(data.maps, data.map_name);
  showToast(`💾 Map saved to alphabot_mapping/maps/${data.map_name}`);
}

async function loadSelectedMap() {
  const map_name = document.getElementById("mapSelect").value;
  if (!map_name) return;
  const res = await fetch("/api/maps/load", {
    method: "POST",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify({ map_name }),
  });
  const data = await res.json();
  if (!res.ok) {
    showToast("⚠️ " + data.detail);
    return;
  }
  renderWaypointsList(data.waypoints);
  setMobileTab("map");
  showToast(`🗺️ Loaded map "${map_name}" and started Navigation mode`);
}

async function switchMode(mode) {
  const selectedMap = document.getElementById("mapSelect").value || activeMapName;
  showToast(`🔄 Switching robot to ${mode} mode...`);
  const res = await fetch("/api/mode", {
    method: "POST",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify({ mode, map_name: selectedMap }),
  });
  const data = await res.json();
  if (!res.ok) {
    showToast("⚠️ " + data.detail);
    return;
  }
  setMobileTab("map");
  showToast(`✅ Robot is now in ${data.mode} mode`);
}

async function cancelNavigation() {
  await fetch("/api/cancel_nav", { method: "POST" });
  showToast("⏹ Active navigation goal canceled.");
}

async function triggerEstop() {
  if (ws && ws.readyState === WebSocket.OPEN) {
    ws.send(JSON.stringify({ type: "estop" }));
  }
  await fetch("/api/estop", { method: "POST" });
  showToast("🛑 EMERGENCY STOP TRIGGERED!");
}

async function toggleSafetyLock() {
  const res = await fetch("/api/safety_lock", {
    method: "POST",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify({ locked: !safetyLocked }),
  });
  const data = await res.json();
  if (res.ok) {
    safetyLocked = data.safety_locked;
    showToast(
      safetyLocked
        ? "🔒 Safety Stop Lock ENGAGED (/safety_stop = true)"
        : "🔓 Safety Stop Lock RELEASED"
    );
  }
}

// ==========================================
// 6. Virtual Joystick + WASD Teleop (100ms Heartbeat)
// ==========================================
const joyBase = document.getElementById("joystickBase");
const joyStick = document.getElementById("joystickStick");
const speedSlider = document.getElementById("maxSpeedSlider");
const speedValText = document.getElementById("maxSpeedVal");

let maxLinearSpeed = 0.35;
let joyActive = false;
let cmdVx = 0.0;
let cmdWz = 0.0;
let joyTimer = null;

speedSlider.addEventListener("input", () => {
  maxLinearSpeed = parseFloat(speedSlider.value);
  speedValText.textContent = maxLinearSpeed.toFixed(2);
});

function transmitVelocity(vx, wz) {
  if (ws && ws.readyState === WebSocket.OPEN) {
    ws.send(JSON.stringify({ type: "cmd_vel", vx, wz }));
  }
}

function startJoyHeartbeat() {
  if (!joyTimer) {
    joyTimer = setInterval(() => {
      if (joyActive) transmitVelocity(cmdVx, cmdWz);
    }, 100);
  }
}

function stopJoystick() {
  joyActive = false;
  cmdVx = 0.0;
  cmdWz = 0.0;
  joyStick.style.transform = "translate(0px, 0px)";
  if (joyTimer) {
    clearInterval(joyTimer);
    joyTimer = null;
  }
  transmitVelocity(0.0, 0.0);
}

function updateJoystickFromPointer(clientX, clientY) {
  const rect = joyBase.getBoundingClientRect();
  const cx = rect.left + rect.width / 2;
  const cy = rect.top + rect.height / 2;
  const maxRadius = rect.width / 2 - 24;

  let dx = clientX - cx;
  let dy = clientY - cy;
  const dist = Math.hypot(dx, dy);
  if (dist > maxRadius) {
    dx = (dx / dist) * maxRadius;
    dy = (dy / dist) * maxRadius;
  }

  joyStick.style.transform = `translate(${dx}px, ${dy}px)`;
  cmdVx = Number(((-dy / maxRadius) * maxLinearSpeed).toFixed(3));
  cmdWz = Number(((-dx / maxRadius) * 1.25).toFixed(3));
  transmitVelocity(cmdVx, cmdWz);
}

joyBase.addEventListener("pointerdown", (e) => {
  joyActive = true;
  joyBase.setPointerCapture(e.pointerId);
  updateJoystickFromPointer(e.clientX, e.clientY);
  startJoyHeartbeat();
});

joyBase.addEventListener("pointermove", (e) => {
  if (joyActive) updateJoystickFromPointer(e.clientX, e.clientY);
});

joyBase.addEventListener("pointerup", stopJoystick);
joyBase.addEventListener("pointercancel", stopJoystick);

// Keyboard Teleop (WASD / Arrow keys) when not typing in an input box
const keysPressed = new Set();
window.addEventListener("keydown", (e) => {
  if (e.target.tagName === "INPUT" || e.target.tagName === "SELECT") return;
  const k = e.key.toLowerCase();
  if (["w", "a", "s", "d", "arrowup", "arrowdown", "arrowleft", "arrowright"].includes(k)) {
    keysPressed.add(k);
    updateVelocityFromKeys();
    startJoyHeartbeat();
  } else if (e.key === " ") {
    triggerEstop();
  }
});

window.addEventListener("keyup", (e) => {
  const k = e.key.toLowerCase();
  if (keysPressed.has(k)) {
    keysPressed.delete(k);
    if (keysPressed.size === 0) {
      stopJoystick();
    } else {
      updateVelocityFromKeys();
    }
  }
});

function updateVelocityFromKeys() {
  joyActive = true;
  let vx = 0.0;
  let wz = 0.0;
  if (keysPressed.has("w") || keysPressed.has("arrowup")) vx += maxLinearSpeed;
  if (keysPressed.has("s") || keysPressed.has("arrowdown")) vx -= maxLinearSpeed;
  if (keysPressed.has("a") || keysPressed.has("arrowleft")) wz += 1.0;
  if (keysPressed.has("d") || keysPressed.has("arrowright")) wz -= 1.0;
  cmdVx = vx;
  cmdWz = wz;
  transmitVelocity(cmdVx, cmdWz);
}

// Start App
connectWebSocket();
requestAnimationFrame(renderLoop);
