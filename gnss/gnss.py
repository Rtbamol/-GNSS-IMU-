from __future__ import annotations
from dataclasses import dataclass
import time
from rov_control.geo import GeoPoint


@dataclass
class GNSSFix:
    timestamp: float
    point: GeoPoint
    fix_status: str = "unknown"   # fixed / float / single / none
    satellites: int = 0
    hdop: float = 99.0
    vdop: float = 99.0
    diff_age_s: float = 999.0
    speed_mps: float = 0.0
    heading_deg: float | None = None
    lost: bool = False

    @property
    def is_fixed(self) -> bool:
        return self.fix_status.lower() in {"fixed", "fix", "rtk_fixed", "4", "固定解"} and not self.lost


def parse_simple_csv_line(line: str) -> GNSSFix:
    """解析简化 RTK 输入。

    推荐 RTK 接入模块把 NMEA/厂商协议转换为：
    timestamp,lat,lon,alt,fix_status,satellites,hdop,vdop,diff_age,speed,heading
    若没有 RTK 硬件，可先用这个格式做联调。
    """
    parts = [x.strip() for x in line.split(",")]
    if len(parts) < 4:
        raise ValueError("RTK CSV 至少需要 timestamp,lat,lon,alt")
    ts = float(parts[0]) if parts[0] else time.time()
    heading = float(parts[10]) if len(parts) > 10 and parts[10] else None
    return GNSSFix(
        timestamp=ts,
        point=GeoPoint(lat=float(parts[1]), lon=float(parts[2]), alt=float(parts[3])),
        fix_status=parts[4] if len(parts) > 4 else "unknown",
        satellites=int(parts[5]) if len(parts) > 5 and parts[5] else 0,
        hdop=float(parts[6]) if len(parts) > 6 and parts[6] else 99.0,
        vdop=float(parts[7]) if len(parts) > 7 and parts[7] else 99.0,
        diff_age_s=float(parts[8]) if len(parts) > 8 and parts[8] else 999.0,
        speed_mps=float(parts[9]) if len(parts) > 9 and parts[9] else 0.0,
        heading_deg=heading,
        lost=False,
    )


def is_usable_for_navigation(fix: GNSSFix, require_fixed: bool = False) -> bool:
    """判断 GNSS 是否可用于水面导航。

    require_fixed=True 时只接受 RTK 固定解；False 时接受 single/dgps/float/fixed，但过滤丢失与空坐标。
    """
    if fix.lost:
        return False
    if require_fixed:
        return fix.fix_status.lower() in {"fixed", "rtk_fixed", "4", "固定解"}
    return fix.fix_status.lower() not in {"none", "invalid", "0"}




