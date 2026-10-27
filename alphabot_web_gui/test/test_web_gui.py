"""Unit and integration tests for alphabot_web_gui."""

import asyncio
import base64
import json
import math
import os
import struct
import urllib.request
from pathlib import Path

os.environ["RMW_IMPLEMENTATION"] = "rmw_fastrtps_cpp"

import numpy as np
import pytest
import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from std_msgs.msg import Bool

from alphabot_web_gui.gateway_node import AlphaBotWebGatewayNode, build_web_server
from alphabot_web_gui.map_manager import (
    MapAndWaypointManager,
    quaternion_to_yaw,
    yaw_to_quaternion,
)
from alphabot_web_gui.sim_robot_node import AlphaBot2DSimulatorNode


def test_quaternion_yaw_roundtrip():
    for angle in (-1.57, -0.5, 0.0, 0.785, 2.1):
        qx, qy, qz, qw = yaw_to_quaternion(angle)
        recovered = quaternion_to_yaw(qx, qy, qz, qw)
        assert math.isclose(angle, recovered, abs_tol=1e-5)


def test_map_and_waypoint_manager(tmp_path: Path):
    db_file = tmp_path / "test.db"
    maps_dir = tmp_path / "maps"
    mgr = MapAndWaypointManager(db_file, [maps_dir])

    # 1. Waypoints CRUD
    wp1 = mgr.save_waypoint("house", "Kitchen", 1.25, -2.5, 1.57)
    assert wp1["name"] == "Kitchen"
    assert wp1["x"] == 1.25

    wp2 = mgr.save_waypoint("house", "Charging Dock", 0.0, 0.0, 0.0)
    all_wps = mgr.list_waypoints("house")
    assert len(all_wps) == 2

    # Update existing waypoint
    mgr.save_waypoint("house", "Kitchen", 2.0, -3.0, 0.5)
    fetched = mgr.get_waypoint("house", "Kitchen")
    assert fetched is not None
    assert fetched["x"] == 2.0
    assert fetched["y"] == -3.0

    # Copy waypoints to a new map
    mgr.copy_waypoints("house", "house_v2")
    assert len(mgr.list_waypoints("house_v2")) == 2

    # Delete waypoint
    assert mgr.delete_waypoint("house", "Kitchen") is True
    assert len(mgr.list_waypoints("house")) == 1

    # 2. OccupancyGrid -> PNG encoding & saving/loading map
    w, h, res = 20, 20, 0.05
    ox, oy = -0.5, -0.5
    grid = np.zeros((h, w), dtype=np.int8)
    grid[10, 10] = 100  # Obstacle at center (gx=10, gy=10 -> x=0.0, y=0.0)

    png_payload = mgr.encode_occupancy_grid_to_png(w, h, res, ox, oy, grid.ravel())
    assert png_payload is not None
    assert png_payload["width"] == w
    assert png_payload["height"] == h
    raw_png = base64.b64decode(png_payload["image_b64"])
    assert raw_png.startswith(b"\x89PNG")

    # 3. Safe-point BFS snapping
    # Free cell at (gx=2, gy=2 -> x=-0.38, y=-0.38) should not snap
    sx, sy, snapped = mgr.validate_and_snap_point(
        -0.38, -0.38, w, h, res, ox, oy, grid.ravel()
    )
    assert snapped is False

    # Obstacle cell at (gx=10, gy=10 -> x=0.02, y=0.02) should snap to adjacent free cell
    sx2, sy2, snapped2 = mgr.validate_and_snap_point(
        0.02, 0.02, w, h, res, ox, oy, grid.ravel()
    )
    assert snapped2 is True
    assert math.hypot(sx2 - 0.02, sy2 - 0.02) <= 0.15

    # Outside map bounds should raise ValueError
    with pytest.raises(ValueError):
        mgr.validate_and_snap_point(10.0, 10.0, w, h, res, ox, oy, grid.ravel())

    # 4. Save map to disk & load back
    saved_name = mgr.save_occupancy_grid_to_map_dirs(
        "test_lab", w, h, res, ox, oy, grid.ravel()
    )
    assert saved_name == "test_lab"
    assert (maps_dir / "test_lab" / "map.yaml").is_file()
    assert (maps_dir / "test_lab" / "map.pgm").is_file()

    loaded = mgr.load_static_map_as_grid("test_lab")
    assert loaded is not None
    lw, lh, lres, lox, loy, lgrid, lpayload = loaded
    assert lw == w and lh == h
    assert math.isclose(lres, res, abs_tol=1e-5)


def test_gateway_http_and_websocket_integration(tmp_path: Path):
    rclpy.init()
    try:
        db_file = tmp_path / "gui.db"
        maps_dir = tmp_path / "maps"
        node = AlphaBotWebGatewayNode(db_path=db_file, mapping_dirs=[maps_dir])
        node.port = 18089
        node.watchdog_timeout_sec = 0.15

        # Create a small valid map in memory
        w, h, res = 30, 30, 0.05
        grid = np.zeros((h, w), dtype=np.int8)
        node.grid_width = w
        node.grid_height = h
        node.grid_resolution = res
        node.grid_origin_x = -0.5
        node.grid_origin_y = -0.5
        node.grid_data = grid.ravel()
        node.latest_map_payload = node.map_mgr.encode_occupancy_grid_to_png(
            w, h, res, -0.5, -0.5, grid.ravel()
        )
        node.map_version = 1

        received_twists = []
        received_safety = []
        received_initpose = []

        sub_twist = node.create_subscription(
            Twist,
            node.cmd_vel_topic,
            lambda msg: received_twists.append((msg.linear.x, msg.angular.z)),
            10,
        )
        sub_safety = node.create_subscription(
            Bool,
            node.safety_stop_topic,
            lambda msg: received_safety.append(msg.data),
            10,
        )
        sub_init = node.create_subscription(
            PoseWithCovarianceStamped,
            "/initialpose",
            lambda msg: received_initpose.append(
                (msg.pose.pose.position.x, msg.pose.pose.position.y)
            ),
            10,
        )

        def drain_ros_callbacks(count: int = 12):
            for _ in range(count):
                rclpy.spin_once(node, timeout_sec=0.005)

        async def run_test_flow():
            server = build_web_server(node)
            await server.start()
            try:
                base_url = f"http://127.0.0.1:{node.port}"

                # 1. Verify static HTML served
                html = await asyncio.to_thread(
                    lambda: urllib.request.urlopen(f"{base_url}/").read().decode("utf-8")
                )
                assert "ServiceBot" in html

                # 2. Save a named location via HTTP POST /api/waypoints
                def post_json(path: str, payload: dict):
                    req = urllib.request.Request(
                        f"{base_url}{path}",
                        data=json.dumps(payload).encode("utf-8"),
                        headers={"Content-Type": "application/json"},
                        method="POST",
                    )
                    with urllib.request.urlopen(req) as resp:
                        return json.loads(resp.read().decode("utf-8"))

                wp_res = await asyncio.to_thread(
                    post_json,
                    "/api/waypoints",
                    {"name": "Station_A", "x": 0.25, "y": 0.30, "yaw": 0.5},
                )
                assert wp_res["status"] == "ok"
                assert wp_res["waypoint"]["name"] == "Station_A"

                # 3. Relocalize (/initialpose) at saved waypoint
                init_res = await asyncio.to_thread(
                    post_json, "/api/initialpose", {"waypoint_name": "Station_A"}
                )
                assert init_res["status"] == "ok"
                drain_ros_callbacks()
                assert len(received_initpose) >= 1
                assert math.isclose(received_initpose[-1][0], 0.25, abs_tol=1e-3)

                # 4. Save map via HTTP POST /api/maps/save
                save_res = await asyncio.to_thread(
                    post_json, "/api/maps/save", {"map_name": "test_area"}
                )
                assert save_res["status"] == "saved"
                assert (maps_dir / "test_area" / "map.yaml").is_file()

                # 5. Connect via RFC 6455 WebSocket, receive telemetry, and send cmd_vel
                reader, writer = await asyncio.open_connection("127.0.0.1", node.port)
                ws_req = (
                    "GET /ws HTTP/1.1\r\n"
                    f"Host: 127.0.0.1:{node.port}\r\n"
                    "Upgrade: websocket\r\n"
                    "Connection: Upgrade\r\n"
                    "Sec-WebSocket-Key: dGhlIHNhbXBsZSBub25jZQ==\r\n"
                    "Sec-WebSocket-Version: 13\r\n\r\n"
                )
                writer.write(ws_req.encode("ascii"))
                await writer.drain()
                resp_hdr = await reader.readuntil(b"\r\n\r\n")
                assert b"101 Switching Protocols" in resp_hdr

                # Read first telemetry frame from server
                b1, b2 = await reader.readexactly(2)
                length = b2 & 0x7F
                if length == 126:
                    length = struct.unpack("!H", await reader.readexactly(2))[0]
                elif length == 127:
                    length = struct.unpack("!Q", await reader.readexactly(8))[0]
                payload_bytes = await reader.readexactly(length)
                telemetry = json.loads(payload_bytes.decode("utf-8"))
                assert telemetry["type"] == "telemetry"
                assert "map" in telemetry

                # Send a masked client WebSocket frame with cmd_vel
                cmd_bytes = json.dumps(
                    {"type": "cmd_vel", "vx": 0.25, "wz": -0.4}
                ).encode("utf-8")
                mask_key = b"\x12\x34\x56\x78"
                masked_payload = bytes(
                    cmd_bytes[i] ^ mask_key[i & 3] for i in range(len(cmd_bytes))
                )
                frame = (
                    bytes([0x81, 0x80 | len(cmd_bytes)]) + mask_key + masked_payload
                )
                writer.write(frame)
                await writer.drain()

                await asyncio.sleep(0.05)
                drain_ros_callbacks()
                assert any(
                    math.isclose(vx, 0.25, abs_tol=1e-3) for vx, _ in received_twists
                )

                # Wait for 150ms watchdog to trigger and publish zero Twist
                await asyncio.sleep(0.22)
                node.watchdog_timer_cb()
                drain_ros_callbacks()
                assert received_twists[-1] == (0.0, 0.0)

                writer.close()
            finally:
                await server.stop()

        asyncio.run(run_test_flow())
        node.destroy_node()
    finally:
        rclpy.shutdown()


def test_2d_simulator_astar_and_lidar():
    rclpy.init()
    try:
        sim = AlphaBot2DSimulatorNode()
        path = sim._plan_astar_path(0.0, 0.0, 0.5, 0.5)
        assert len(path) >= 2
        assert math.isclose(path[-1][0], 0.5, abs_tol=1e-3)
        assert math.isclose(path[-1][1], 0.5, abs_tol=1e-3)
        sim.destroy_node()
    finally:
        rclpy.shutdown()

