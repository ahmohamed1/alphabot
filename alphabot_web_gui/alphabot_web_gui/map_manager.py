"""Map compression, OccupancyGrid validation/BFS snapping, Nav2 map saving, and SQLite waypoints."""

import base64
import io
import json
import math
import shutil
import sqlite3
import time
from collections import deque
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple

import numpy as np
import yaml
from PIL import Image


def yaw_to_quaternion(yaw: float) -> Tuple[float, float, float, float]:
    half = yaw * 0.5
    return (0.0, 0.0, math.sin(half), math.cos(half))


def quaternion_to_yaw(x: float, y: float, z: float, w: float) -> float:
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


class MapAndWaypointManager:
    """Manages SQLite persistence for named waypoints, map files, and OccupancyGrid processing."""

    def __init__(
        self,
        db_path: Path,
        mapping_dirs: List[Path],
    ):
        self.db_path = Path(db_path)
        self.db_path.parent.mkdir(parents=True, exist_ok=True)
        # First directory is primary persistent dir, rest are mirrored (e.g. install share)
        self.mapping_dirs = [Path(d) for d in mapping_dirs if d is not None]
        for d in self.mapping_dirs:
            d.mkdir(parents=True, exist_ok=True)
        self._init_db()

    def _init_db(self) -> None:
        conn = sqlite3.connect(self.db_path)
        c = conn.cursor()
        c.execute(
            """
            CREATE TABLE IF NOT EXISTS waypoints (
                id INTEGER PRIMARY KEY AUTOINCREMENT,
                map_name TEXT NOT NULL,
                name TEXT NOT NULL,
                x REAL NOT NULL,
                y REAL NOT NULL,
                yaw REAL NOT NULL,
                created_at REAL NOT NULL,
                UNIQUE(map_name, name)
            )
            """
        )
        c.execute(
            """
            CREATE TABLE IF NOT EXISTS robot_state (
                key TEXT PRIMARY KEY,
                value TEXT NOT NULL
            )
            """
        )
        conn.commit()
        conn.close()

    # ---------------------------------------------------------
    # Waypoints CRUD
    # ---------------------------------------------------------
    def list_waypoints(self, map_name: str) -> List[Dict[str, Any]]:
        conn = sqlite3.connect(self.db_path)
        c = conn.cursor()
        c.execute(
            "SELECT id, name, x, y, yaw FROM waypoints WHERE map_name = ? ORDER BY name ASC",
            (map_name,),
        )
        rows = [
            {
                "id": r[0],
                "name": r[1],
                "x": round(float(r[2]), 3),
                "y": round(float(r[3]), 3),
                "yaw": round(float(r[4]), 3),
            }
            for r in c.fetchall()
        ]
        conn.close()
        return rows

    def get_waypoint(self, map_name: str, name: str) -> Optional[Dict[str, Any]]:
        conn = sqlite3.connect(self.db_path)
        c = conn.cursor()
        c.execute(
            "SELECT id, name, x, y, yaw FROM waypoints WHERE map_name = ? AND name = ?",
            (map_name, name),
        )
        row = c.fetchone()
        conn.close()
        if not row:
            return None
        return {
            "id": row[0],
            "name": row[1],
            "x": float(row[2]),
            "y": float(row[3]),
            "yaw": float(row[4]),
        }

    def save_waypoint(
        self, map_name: str, name: str, x: float, y: float, yaw: float
    ) -> Dict[str, Any]:
        clean_name = name.strip()
        if not clean_name:
            raise ValueError("Location name cannot be empty.")

        conn = sqlite3.connect(self.db_path)
        c = conn.cursor()
        c.execute(
            """
            INSERT INTO waypoints (map_name, name, x, y, yaw, created_at)
            VALUES (?, ?, ?, ?, ?, ?)
            ON CONFLICT(map_name, name)
            DO UPDATE SET x=excluded.x, y=excluded.y, yaw=excluded.yaw, created_at=excluded.created_at
            """,
            (map_name, clean_name, float(x), float(y), float(yaw), time.time()),
        )
        conn.commit()
        conn.close()
        return {
            "map_name": map_name,
            "name": clean_name,
            "x": round(float(x), 3),
            "y": round(float(y), 3),
            "yaw": round(float(yaw), 3),
        }

    def delete_waypoint(self, map_name: str, name: str) -> bool:
        conn = sqlite3.connect(self.db_path)
        c = conn.cursor()
        c.execute(
            "DELETE FROM waypoints WHERE map_name = ? AND name = ?",
            (map_name, name),
        )
        deleted = c.rowcount > 0
        conn.commit()
        conn.close()
        return deleted

    def copy_waypoints(self, from_map: str, to_map: str) -> None:
        if from_map == to_map:
            return
        wps = self.list_waypoints(from_map)
        for wp in wps:
            self.save_waypoint(to_map, wp["name"], wp["x"], wp["y"], wp["yaw"])

    # ---------------------------------------------------------
    # Robot State Persistence (Last Pose & Active Map)
    # ---------------------------------------------------------
    def set_state(self, key: str, value: Any) -> None:
        conn = sqlite3.connect(self.db_path)
        c = conn.cursor()
        c.execute(
            "INSERT OR REPLACE INTO robot_state (key, value) VALUES (?, ?)",
            (key, json.dumps(value)),
        )
        conn.commit()
        conn.close()

    def get_state(self, key: str, default: Any = None) -> Any:
        conn = sqlite3.connect(self.db_path)
        c = conn.cursor()
        c.execute("SELECT value FROM robot_state WHERE key = ?", (key,))
        row = c.fetchone()
        conn.close()
        if not row:
            return default
        try:
            return json.loads(row[0])
        except Exception:
            return default

    # ---------------------------------------------------------
    # OccupancyGrid -> PNG Encoding & Safe-Point BFS Validation
    # ---------------------------------------------------------
    @staticmethod
    def encode_occupancy_grid_to_png(
        width: int,
        height: int,
        resolution: float,
        origin_x: float,
        origin_y: float,
        data: Any,
    ) -> Optional[Dict[str, Any]]:
        if width <= 0 or height <= 0:
            return None

        grid = np.asarray(data, dtype=np.int8).reshape((height, width))
        # Render styled grayscale image:
        # Unknown (-1) -> 110 (slate gray)
        # Free (0..25) -> 242 (off-white)
        # Intermediate (26..50) -> 180
        # Occupied (>50) -> 20 (near black)
        img_arr = np.full((height, width), 110, dtype=np.uint8)
        img_arr[(grid >= 0) & (grid <= 25)] = 242
        img_arr[(grid > 25) & (grid <= 50)] = 175
        img_arr[grid > 50] = 20

        # Flip vertically so row 0 is top (north/+Y in ROS map frame)
        img_arr = np.flipud(img_arr)

        pil_img = Image.fromarray(img_arr, mode="L")
        buf = io.BytesIO()
        pil_img.save(buf, format="PNG", optimize=True)
        b64_png = base64.b64encode(buf.getvalue()).decode("ascii")

        return {
            "width": int(width),
            "height": int(height),
            "resolution": round(float(resolution), 5),
            "origin_x": round(float(origin_x), 4),
            "origin_y": round(float(origin_y), 4),
            "image_b64": b64_png,
        }

    @staticmethod
    def validate_and_snap_point(
        x: float,
        y: float,
        width: int,
        height: int,
        resolution: float,
        origin_x: float,
        origin_y: float,
        data: Any,
        max_snap_distance_m: float = 0.65,
    ) -> Tuple[float, float, bool]:
        """Validate (x, y) in map frame. Snaps to nearest free cell if inside obstacle."""
        if width <= 0 or height <= 0 or resolution <= 0:
            return round(x, 3), round(y, 3), False

        gx = int(math.floor((x - origin_x) / resolution))
        gy = int(math.floor((y - origin_y) / resolution))

        if not (0 <= gx < width and 0 <= gy < height):
            raise ValueError("Selected point is outside the map boundaries.")

        grid = np.asarray(data, dtype=np.int8).reshape((height, width))

        # Check if clicked cell is free (0 <= val <= 35)
        if 0 <= grid[gy, gx] <= 35:
            return round(x, 3), round(y, 3), False

        # BFS search for closest free cell with 1-cell clearance
        max_cells = max(1, int(max_snap_distance_m / resolution))
        visited = {(gx, gy)}
        queue = deque([(gx, gy, 0)])

        while queue:
            cx, cy, dist = queue.popleft()
            if dist > max_cells:
                break
            if 0 <= grid[cy, cx] <= 25:
                snapped_x = round(origin_x + (cx + 0.5) * resolution, 3)
                snapped_y = round(origin_y + (cy + 0.5) * resolution, 3)
                return snapped_x, snapped_y, True

            for dx, dy in (
                (-1, 0),
                (1, 0),
                (0, -1),
                (0, 1),
                (-1, -1),
                (1, -1),
                (-1, 1),
                (1, 1),
            ):
                nx, ny = cx + dx, cy + dy
                if 0 <= nx < width and 0 <= ny < height and (nx, ny) not in visited:
                    visited.add((nx, ny))
                    queue.append((nx, ny, dist + 1))

        # If clicked cell is unknown (-1) and no free cell was within snap radius,
        # allow it because Nav2 SmacPlanner2D is configured with allow_unknown: true
        if grid[gy, gx] < 0:
            return round(x, 3), round(y, 3), False

        raise ValueError(
            "Selected point is inside an obstacle/wall with no nearby free space."
        )

    # ---------------------------------------------------------
    # Map Listing, Saving & Static Loading (alphabot_mapping/maps/<name>/)
    # ---------------------------------------------------------
    def list_available_maps(self) -> List[Dict[str, Any]]:
        discovered: Dict[str, Dict[str, Any]] = {}
        for base_dir in self.mapping_dirs:
            if not base_dir.exists():
                continue
            for item in sorted(base_dir.iterdir()):
                if not item.is_dir():
                    continue
                yaml_file = item / "map.yaml"
                if not yaml_file.is_file():
                    yaml_file = item / f"{item.name}.yaml"
                if yaml_file.is_file() and item.name not in discovered:
                    discovered[item.name] = {
                        "name": item.name,
                        "yaml_path": str(yaml_file),
                        "updated_at": round(yaml_file.stat().st_mtime, 1),
                    }
        return sorted(discovered.values(), key=lambda m: m["name"])

    def find_map_yaml(self, map_name: str) -> Optional[Path]:
        for base_dir in self.mapping_dirs:
            for candidate in (
                base_dir / map_name / "map.yaml",
                base_dir / map_name / f"{map_name}.yaml",
            ):
                if candidate.is_file():
                    return candidate
        return None

    def load_static_map_as_grid(
        self, map_name: str
    ) -> Optional[Tuple[int, int, float, float, float, np.ndarray, Dict[str, Any]]]:
        """Read a saved map.yaml + map.pgm from disk and return grid + PNG payload."""
        yaml_path = self.find_map_yaml(map_name)
        if yaml_path is None:
            return None

        meta = yaml.safe_load(yaml_path.read_text(encoding="utf-8"))
        img_rel = meta.get("image", "map.pgm")
        pgm_path = (yaml_path.parent / img_rel).resolve()
        if not pgm_path.is_file():
            return None

        resolution = float(meta.get("resolution", 0.05))
        origin = meta.get("origin", [0.0, 0.0, 0.0])
        origin_x, origin_y = float(origin[0]), float(origin[1])

        pil_img = Image.open(pgm_path).convert("L")
        width, height = pil_img.size
        # PGM row 0 is top of image (north), whereas ROS OccupancyGrid row 0 is bottom (south)
        pixels_top_down = np.asarray(pil_img, dtype=np.uint8)
        pixels_bottom_up = np.flipud(pixels_top_down)

        grid = np.full((height, width), -1, dtype=np.int8)
        grid[pixels_bottom_up >= 230] = 0
        grid[pixels_bottom_up <= 100] = 100

        payload = self.encode_occupancy_grid_to_png(
            width, height, resolution, origin_x, origin_y, grid.ravel()
        )
        return width, height, resolution, origin_x, origin_y, grid.ravel(), payload

    def save_occupancy_grid_to_map_dirs(
        self,
        map_name: str,
        width: int,
        height: int,
        resolution: float,
        origin_x: float,
        origin_y: float,
        data: Any,
    ) -> str:
        """Save OccupancyGrid directly into alphabot_mapping/maps/<map_name>/map.{pgm,yaml}."""
        clean_name = "".join(
            c for c in map_name.strip() if c.isalnum() or c in ("_", "-")
        )
        if not clean_name:
            raise ValueError("Invalid map name. Use letters, numbers, '_', or '-'.")
        if width <= 0 or height <= 0:
            raise ValueError("No valid map data available to save yet.")

        grid = np.asarray(data, dtype=np.int8).reshape((height, width))
        # Nav2 standard trinary PGM values: free=254, occupied=0, unknown=205
        pgm = np.full((height, width), 205, dtype=np.uint8)
        pgm[(grid >= 0) & (grid <= 25)] = 254
        pgm[grid >= 65] = 0
        # Flip vertically so row 0 in PGM is top (+Y)
        pgm_top_down = np.flipud(pgm)

        yaml_content = (
            "image: map.pgm\n"
            f"resolution: {resolution:.6f}\n"
            f"origin: [{origin_x:.6f}, {origin_y:.6f}, 0.000000]\n"
            "negate: 0\n"
            "occupied_thresh: 0.65\n"
            "free_thresh: 0.196\n"
        )

        saved_yaml_path = ""
        for base_dir in self.mapping_dirs:
            target_dir = base_dir / clean_name
            target_dir.mkdir(parents=True, exist_ok=True)
            pgm_file = target_dir / "map.pgm"
            yaml_file = target_dir / "map.yaml"

            Image.fromarray(pgm_top_down, mode="L").save(pgm_file, format="PPM")
            # Ensure standard P5 PGM header
            with open(pgm_file, "wb") as f:
                header = f"P5\n{width} {height}\n255\n".encode("ascii")
                f.write(header)
                f.write(pgm_top_down.tobytes())

            yaml_file.write_text(yaml_content, encoding="utf-8")
            if not saved_yaml_path:
                saved_yaml_path = str(yaml_file)

        return clean_name
