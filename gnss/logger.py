from __future__ import annotations
import csv
from pathlib import Path
from dataclasses import asdict
from typing import Any
from rov_control.protocol import SensorFrame
from rov_control.gnss import GNSSFix
from rov_control.controller import ControlOutput
from rov_control.geo import LocalPoint
from rov_control.thruster_mixer import preview_thruster_pwms


class CSVLogger:
    def __init__(self, folder: Path):
        self.folder = folder
        self.files: dict[str, Any] = {}
        self.writers: dict[str, csv.DictWriter] = {}

    def _writer(self, filename: str, fieldnames: list[str]) -> csv.DictWriter:
        if filename not in self.writers:
            f = open(self.folder / filename, "w", newline="", encoding="utf-8")
            w = csv.DictWriter(f, fieldnames=fieldnames)
            w.writeheader()
            f.flush()
            self.files[filename] = f
            self.writers[filename] = w
        return self.writers[filename]

    def _write_row(self, filename: str, fieldnames: list[str], row: dict[str, Any]) -> None:
        self._writer(filename, fieldnames).writerow(row)
        self.files[filename].flush()

    def log_gnss(self, fix: GNSSFix, local: LocalPoint | None = None) -> None:
        row = {
            "timestamp": fix.timestamp, "lon": fix.point.lon, "lat": fix.point.lat, "alt": fix.point.alt,
            "fix_status": fix.fix_status, "satellites": fix.satellites, "hdop": fix.hdop, "vdop": fix.vdop,
            "diff_age_s": fix.diff_age_s, "speed_mps": fix.speed_mps, "heading_deg": fix.heading_deg,
            "lost": fix.lost, "local_x_m": None if local is None else local.x,
            "local_y_m": None if local is None else local.y, "local_z_m": None if local is None else local.z,
        }
        self._write_row("gnss_log.csv", list(row.keys()), row)

    def log_imu(self, imu: SensorFrame) -> None:
        row = asdict(imu)
        self._write_row("imu_log.csv", list(row.keys()), row)

    def log_control(self, output: ControlOutput) -> None:
        cmd = output.command
        thrusters = preview_thruster_pwms(cmd)
        row = {
            "timestamp": output.timestamp, "mode": output.mode,
            "target_heading_deg": output.target_heading_deg, "actual_heading_deg": output.actual_heading_deg,
            "yaw_error_deg": output.yaw_error_deg, "target_speed_mps": output.target_speed_mps,
            "target_depth_m": output.target_depth_m, "actual_depth_m": output.actual_depth_m,
            "forward_pwm": cmd.forward, "yaw_pwm": cmd.yaw, "lateral_pwm": cmd.lateral, "vertical_pwm": cmd.vertical,
            "heading_hold": cmd.heading_hold, "depth_hold": cmd.depth_hold, "arm": cmd.arm, "gear": cmd.gear,
            "thruster_l1_pwm": thrusters["l1"], "thruster_l2_pwm": thrusters["l2"],
            "thruster_l3_pwm": thrusters["l3"], "thruster_l4_pwm": thrusters["l4"],
            "thruster_r1_pwm": thrusters["r1"], "thruster_r2_pwm": thrusters["r2"],
            "thruster_r3_pwm": thrusters["r3"], "thruster_r4_pwm": thrusters["r4"],
        }
        self._write_row("control_log.csv", list(row.keys()), row)

    def log_waypoints(self, points: list[LocalPoint]) -> None:
        fieldnames = ["index", "x_m", "y_m", "z_m"]
        w = self._writer("waypoint_log.csv", fieldnames)
        for i, p in enumerate(points):
            self._write_row("waypoint_log.csv", fieldnames, {"index": i, "x_m": p.x, "y_m": p.y, "z_m": p.z})

    def log_trajectory(self, filename: str, timestamp: float, p: LocalPoint, source: str) -> None:
        row = {"timestamp": timestamp, "x_m": p.x, "y_m": p.y, "z_m": p.z, "source": source}
        self._write_row(filename, list(row.keys()), row)

    def log_event(self, timestamp: float, level: str, event: str, detail: str = "") -> None:
        row = {"timestamp": timestamp, "level": level, "event": event, "detail": detail}
        self._write_row("event_log.csv", list(row.keys()), row)

    def close(self) -> None:
        for f in self.files.values():
            f.flush()
            f.close()
        self.files.clear()
        self.writers.clear()
