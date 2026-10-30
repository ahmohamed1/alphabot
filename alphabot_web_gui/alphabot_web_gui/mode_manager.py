"""Mode Manager for switching cleanly between SLAM (Mapping) and AMCL (Localization/Navigation)."""

import os
import signal
import subprocess
import time
from pathlib import Path
from typing import List, Optional


class ModeManager:
    """Manages SLAM vs Localization launch processes without leaving orphan ROS 2 nodes."""

    SLAM_EXEC_MARKERS = (
        "sync_slam_toolbox_node",
        "async_slam_toolbox_node",
        "mapping_with_known_poses",
        "slam_map_to_odom_broadcaster",
        "map_saver_server",
        "lifecycle_manager_slam",
        "slam.launch.py",
    )
    LOCALIZATION_EXEC_MARKERS = (
        "nav2_map_server/map_server",
        "nav2_amcl/amcl",
        "lifecycle_manager_localization",
        "global_localization.launch.py",
    )
    NAVIGATION_EXEC_MARKERS = (
        "nav2_controller/controller_server",
        "nav2_planner/planner_server",
        "nav2_bt_navigator/bt_navigator",
        "lifecycle_manager_navigation",
        "navigation.launch.py",
    )

    def __init__(self, initial_mode: str = "STANDBY", initial_map: str = ""):
        self.current_mode = initial_mode.upper()
        self.active_map = initial_map
        self._managed_proc: Optional[subprocess.Popen] = None
        self._nav_proc: Optional[subprocess.Popen] = None

    @staticmethod
    def _find_Matching_pids(markers: tuple) -> List[int]:
        """Scan /proc for processes matching any of the marker strings (excluding self)."""
        my_pid = os.getpid()
        matched_pids: List[int] = []
        proc_root = Path("/proc")
        if not proc_root.exists():
            return matched_pids

        for entry in proc_root.iterdir():
            if not entry.name.isdigit():
                continue
            pid = int(entry.name)
            if pid == my_pid:
                continue
            try:
                cmdline_bytes = (entry / "cmdline").read_bytes()
                cmdline = cmdline_bytes.replace(b"\x00", b" ").decode(
                    "utf-8", errors="ignore"
                )
                if any(m in cmdline for m in markers):
                    matched_pids.append(pid)
            except Exception:
                continue
        return matched_pids

    def _stop_mode_processes(self, markers: tuple) -> None:
        # 1. Stop our own managed subprocess group if active
        if self._managed_proc is not None and self._managed_proc.poll() is None:
            try:
                os.killpg(os.getpgid(self._managed_proc.pid), signal.SIGINT)
                self._managed_proc.wait(timeout=5.0)
            except Exception:
                try:
                    os.killpg(os.getpgid(self._managed_proc.pid), signal.SIGTERM)
                except Exception:
                    pass
            self._managed_proc = None

        # 2. Also gracefully stop any externally launched SLAM/Localization nodes by PID
        pids = self._find_Matching_pids(markers)
        for pid in pids:
            try:
                os.kill(pid, signal.SIGINT)
            except OSError:
                pass

        if pids:
            deadline = time.monotonic() + 4.0
            while time.monotonic() < deadline:
                remaining = [p for p in pids if Path(f"/proc/{p}").exists()]
                if not remaining:
                    break
                time.sleep(0.2)
            for pid in pids:
                if Path(f"/proc/{pid}").exists():
                    try:
                        os.kill(pid, signal.SIGTERM)
                    except OSError:
                        pass

    def _ensure_navigation_running(self, use_sim_time: bool) -> None:
        """Start alphabot_navigation navigation.launch.py if bt_navigator is not already running."""
        if self._nav_proc is not None and self._nav_proc.poll() is None:
            return
        existing_nav = self._find_Matching_pids(("nav2_bt_navigator/bt_navigator",))
        if existing_nav:
            return

        sim_str = "true" if use_sim_time else "false"
        env = os.environ.copy()
        env["RMW_IMPLEMENTATION"] = "rmw_fastrtps_cpp"
        cmd = [
            "ros2",
            "launch",
            "alphabot_navigation",
            "navigation.launch.py",
            f"use_sim_time:={sim_str}",
        ]
        self._nav_proc = subprocess.Popen(
            cmd,
            env=env,
            preexec_fn=os.setsid,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.STDOUT,
        )

    def detect_active_mode_from_ros_nodes(self, node_names: List[str]) -> str:
        if any(
            k in n
            for n in node_names
            for k in ("slam_toolbox", "mapping_with_known_poses")
        ):
            self.current_mode = "MAPPING"
        elif any(n in ("amcl", "map_server") for n in node_names):
            self.current_mode = "NAVIGATION"
        return self.current_mode

    def switch_to_mapping(self, use_sim_time: bool = False) -> None:
        """Stop AMCL/map_server and launch alphabot_mapping slam.launch.py + Nav2."""
        all_markers = self.SLAM_EXEC_MARKERS + self.LOCALIZATION_EXEC_MARKERS
        self._stop_mode_processes(all_markers)

        sim_str = "true" if use_sim_time else "false"
        env = os.environ.copy()
        env["RMW_IMPLEMENTATION"] = "rmw_fastrtps_cpp"
        cmd = [
            "ros2",
            "launch",
            "alphabot_mapping",
            "slam.launch.py",
            f"use_sim_time:={sim_str}",
        ]
        self._managed_proc = subprocess.Popen(
            cmd,
            env=env,
            preexec_fn=os.setsid,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.STDOUT,
        )
        self._ensure_navigation_running(use_sim_time)
        self.current_mode = "MAPPING"

    def switch_to_navigation(
        self, map_name: str, use_sim_time: bool = False
    ) -> None:
        """Stop SLAM and launch alphabot_localization global_localization.launch.py + Nav2."""
        all_markers = self.SLAM_EXEC_MARKERS + self.LOCALIZATION_EXEC_MARKERS
        self._stop_mode_processes(all_markers)

        sim_str = "true" if use_sim_time else "false"
        env = os.environ.copy()
        env["RMW_IMPLEMENTATION"] = "rmw_fastrtps_cpp"
        cmd = [
            "ros2",
            "launch",
            "alphabot_localization",
            "global_localization.launch.py",
            f"use_sim_time:={sim_str}",
            f"map_name:={map_name}",
        ]
        self._managed_proc = subprocess.Popen(
            cmd,
            env=env,
            preexec_fn=os.setsid,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.STDOUT,
        )
        self._ensure_navigation_running(use_sim_time)
        self.current_mode = "NAVIGATION"
        self.active_map = map_name

    def shutdown(self) -> None:
        for proc in (self._managed_proc, self._nav_proc):
            if proc is not None and proc.poll() is None:
                try:
                    os.killpg(os.getpgid(proc.pid), signal.SIGINT)
                    proc.wait(timeout=3.0)
                except Exception:
                    pass
        self._managed_proc = None
        self._nav_proc = None
