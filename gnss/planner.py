from __future__ import annotations
from dataclasses import dataclass, field
from typing import Iterable
from rov_control.geo import LocalPoint, distance_2d, bearing_deg, cross_track_error


@dataclass
class PathStatus:
    target: LocalPoint | None
    target_heading_deg: float
    target_speed_mps: float
    remaining_distance_m: float
    arrived: bool
    waypoint_index: int
    cross_track_error_m: float


@dataclass
class WaypointPlanner:
    waypoints: list[LocalPoint] = field(default_factory=list)
    max_speed_mps: float = 0.5
    arrival_radius_m: float = 0.8
    waypoint_index: int = 0
    reverse_history: list[LocalPoint] = field(default_factory=list)

    def set_path(self, points: Iterable[LocalPoint]) -> None:
        self.waypoints = list(points)
        self.waypoint_index = 0
        self.reverse_history.clear()

    def append_history(self, p: LocalPoint) -> None:
        if not self.reverse_history or distance_2d(self.reverse_history[-1], p) > 0.2:
            self.reverse_history.append(p)

    def build_return_path(self, mode: str = "reverse_history") -> list[LocalPoint]:
        if mode == "reverse_history" and len(self.reverse_history) >= 2:
            return list(reversed(self.reverse_history))
        return list(reversed(self.waypoints))

    def update(self, current: LocalPoint) -> PathStatus:
        if not self.waypoints or self.waypoint_index >= len(self.waypoints):
            return PathStatus(None, 0.0, 0.0, 0.0, True, self.waypoint_index, 0.0)
        target = self.waypoints[self.waypoint_index]
        remaining = distance_2d(current, target)
        if remaining <= self.arrival_radius_m:
            self.waypoint_index += 1
            if self.waypoint_index >= len(self.waypoints):
                return PathStatus(target, 0.0, 0.0, remaining, True, self.waypoint_index, 0.0)
            target = self.waypoints[self.waypoint_index]
            remaining = distance_2d(current, target)
        heading = bearing_deg(current, target)
        speed = min(self.max_speed_mps, max(0.15, remaining * 0.35))
        prev = self.waypoints[max(0, self.waypoint_index - 1)] if self.waypoint_index > 0 else current
        xte = cross_track_error(current, prev, target)
        return PathStatus(target, heading, speed, remaining, False, self.waypoint_index, xte)
