from __future__ import annotations
from dataclasses import dataclass
import math

EARTH_RADIUS_M = 6378137.0


@dataclass
class GeoPoint:
    lat: float
    lon: float
    alt: float = 0.0


@dataclass
class LocalPoint:
    x: float
    y: float
    z: float = 0.0


class LocalTangentPlane:
    """以起点为原点的局部 ENU 坐标：x 东、y 北、z 深度向下。"""

    def __init__(self, origin: GeoPoint):
        self.origin = origin
        self.lat0 = math.radians(origin.lat)
        self.lon0 = math.radians(origin.lon)
        self.cos_lat0 = math.cos(self.lat0)

    def geo_to_local(self, p: GeoPoint, depth_m: float = 0.0) -> LocalPoint:
        lat = math.radians(p.lat)
        lon = math.radians(p.lon)
        x = (lon - self.lon0) * EARTH_RADIUS_M * self.cos_lat0
        y = (lat - self.lat0) * EARTH_RADIUS_M
        return LocalPoint(x=x, y=y, z=depth_m)

    def local_to_geo(self, p: LocalPoint) -> GeoPoint:
        lat = self.lat0 + p.y / EARTH_RADIUS_M
        lon = self.lon0 + p.x / (EARTH_RADIUS_M * self.cos_lat0)
        return GeoPoint(lat=math.degrees(lat), lon=math.degrees(lon), alt=self.origin.alt - p.z)


def distance_2d(a: LocalPoint, b: LocalPoint) -> float:
    return math.hypot(b.x - a.x, b.y - a.y)


def bearing_deg(a: LocalPoint, b: LocalPoint) -> float:
    """从 a 指向 b 的航向角：正北 0 度，顺时针。"""
    dx = b.x - a.x
    dy = b.y - a.y
    return (math.degrees(math.atan2(dx, dy)) + 360.0) % 360.0


def wrap_angle_deg(angle: float) -> float:
    return (angle + 180.0) % 360.0 - 180.0


def cross_track_error(point: LocalPoint, start: LocalPoint, end: LocalPoint) -> float:
    vx = end.x - start.x
    vy = end.y - start.y
    wx = point.x - start.x
    wy = point.y - start.y
    norm = math.hypot(vx, vy)
    if norm < 1e-6:
        return distance_2d(point, start)
    return abs(vx * wy - vy * wx) / norm
