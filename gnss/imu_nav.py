from __future__ import annotations

from dataclasses import dataclass
import math

from rov_control.protocol import SensorFrame
from rov_control.geo import (
    GeoPoint,
    LocalPoint,
    LocalTangentPlane,
    wrap_angle_deg,
)
from rov_control.gnss import GNSSFix, is_usable_for_navigation
from rov_control.config import ROVConfig
try:
    from rov_control.dvl import DVLMeasurement
except Exception:  # pragma: no cover
    DVLMeasurement = object


G = 9.80665


@dataclass
class NavState:
    timestamp: float = 0.0

    # 局部坐标，x 东，y 北，z 向下
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0

    # 局部速度
    vx: float = 0.0
    vy: float = 0.0

    # 姿态角
    # yaw_deg: 经 GNSS 航迹角修正后的真北航向，正北 0 度、顺时针为正。
    yaw_deg: float = 0.0
    # raw_yaw_deg: IMU 自身输出的相对 yaw。六轴 IMU 无磁力计时，开机朝向通常就是 0 度。
    raw_yaw_deg: float = 0.0
    # yaw_offset_deg: GNSS 自动估计的航向零偏，yaw_deg = raw_yaw_deg + yaw_offset_deg。
    yaw_offset_deg: float = 0.0
    # gnss_course_deg: 最近一次由 GNSS 位移计算出的航迹角；静止或位移不足时为 None。
    gnss_course_deg: float | None = None
    pitch_deg: float = 0.0
    roll_deg: float = 0.0

    source: str = "uninitialized"

    def local_point(self) -> LocalPoint:
        return LocalPoint(self.x, self.y, self.z)


class SixAxisIMUNavigator:
    """六轴 IMU + 水面 RTK/GNSS 修正导航器。

    说明：
    1. 六轴 IMU 没有磁力计，水下 yaw 只能短时积分；
    2. 水面使用 RTK/GNSS 修正位置；
    3. 第一次可用 GNSS fix 会作为局部坐标原点；
    4. 上浮后继续用 GNSS 修正位置和航向漂移。
    """

    def __init__(self, config: ROVConfig):
        self.cfg = config
        self.state = NavState()
        self.initialized = False

        self.last_imu: SensorFrame | None = None
        self.last_fixed_gnss: GNSSFix | None = None

        # GNSS 航迹角用于估计 IMU 相对 yaw 到真北航向的零偏。
        self.yaw_offset_deg = 0.0
        self.heading_offset_initialized = False
        self.last_heading_local: LocalPoint | None = None
        self.last_heading_timestamp: float | None = None

        # 新增：局部坐标系，以第一次可用 GNSS 为原点
        self.local_frame: LocalTangentPlane | None = None
        self.origin_gnss: GNSSFix | None = None

        # DVL 对底速度积分。DVL 只在 valid 且数据新鲜时参与融合；失效/无数据时不改变状态。
        self.last_dvl_timestamp: float | None = None
        self.last_dvl_valid: bool = False

    def _apply_rov_heading_offset(self, course_deg: float) -> float:
        """Convert GNSS course-over-ground to the ROV bow heading.

        A single-antenna GNSS course is the direction of movement. If field
        tests show the ROV tail is treated as the head, add 180 deg in config.
        """

        return (course_deg + float(getattr(self.cfg, "rov_heading_offset_deg", 0.0))) % 360.0

    def initialize(
        self,
        yaw_deg: float,
        initial_pos: LocalPoint | None = None,
        timestamp: float = 0.0,
    ) -> None:
        p = initial_pos or LocalPoint(0.0, 0.0, 0.0)

        self.state = NavState(
            timestamp=timestamp,
            x=p.x,
            y=p.y,
            z=p.z,
            yaw_deg=(yaw_deg) % 360.0,
            raw_yaw_deg=yaw_deg % 360.0,
            yaw_offset_deg=self.yaw_offset_deg,
            source="init",
        )

        self.initialized = True

    def initialize_from_rtk_motion(
        self,
        prev_pos: LocalPoint,
        curr_pos: LocalPoint,
        timestamp: float,
    ) -> bool:
        """用 RTK 两点位移初始化 yaw。

        dx 为东向位移，dy 为北向位移。
        航向角定义：正北 0 度，顺时针为正。
        """

        # RTK 两点位移应使用 curr - prev，表示真实运动方向。
        # 原先使用 prev - curr 会把航向整体反 180°，现场表现为把尾部当作头部。
        dx = curr_pos.x - prev_pos.x
        dy = curr_pos.y - prev_pos.y

        if math.hypot(dx, dy) < 0.5:
            return False

        course = (math.degrees(math.atan2(dx, dy)) + 360.0) % 360.0
        yaw = self._apply_rov_heading_offset(course)
        self.initialize(yaw, curr_pos, timestamp)

        return True

    def _get_initial_yaw(self, imu: SensorFrame) -> float:
        """获取初始航向。

        如果配置为 manual，则使用人工航向；
        否则优先使用 imu.heading_deg；
        如果 imu.heading_deg 不存在，则回退到人工航向。
        """

        if self.cfg.imu_yaw_init_mode == "manual":
            return self.cfg.manual_initial_yaw_deg

        heading = getattr(imu, "heading_deg", None)

        if heading is None:
            return self.cfg.manual_initial_yaw_deg

        return heading

    def update_with_imu(
        self,
        imu: SensorFrame,
        commanded_speed_mps: float = 0.0,
    ) -> NavState:
        """使用 IMU 和控制指令做短时航位推算。"""

        if not self.initialized:
            yaw0 = self._get_initial_yaw(imu)
            self.initialize(
                yaw0,
                LocalPoint(0.0, 0.0, imu.depth_m),
                imu.timestamp,
            )

        dt = 0.0 if self.last_imu is None else max(
            0.0,
            imu.timestamp - self.last_imu.timestamp,
        )

        # 防止长时间断帧导致积分跳变
        if dt > 0.5:
            dt = 0.1

        gz = imu.gz_dps - self.cfg.yaw_gyro_bias_deg_s

        # 六轴 IMU 无磁力计：IMU 的 heading_deg 只能看作“开机零位下的相对 yaw”。
        # 优先使用 IMU 已经给出的相对 yaw；如果设备没有输出 heading_deg，才退回到 gz 短时积分。
        # 注意：这里没有使用加速度计积分 yaw。
        if getattr(self.cfg, "use_imu_reported_yaw", True) and getattr(imu, "heading_deg", None) is not None:
            raw_yaw = imu.heading_deg % 360.0
        else:
            raw_yaw = (self.state.raw_yaw_deg + gz * dt) % 360.0

        self.state.raw_yaw_deg = raw_yaw
        self.state.yaw_offset_deg = self.yaw_offset_deg
        self.state.yaw_deg = (raw_yaw + self.yaw_offset_deg) % 360.0
        self.state.pitch_deg = imu.pitch_deg
        self.state.roll_deg = imu.roll_deg
        self.state.z = imu.depth_m

        # 水下不做加速度双积分，主要使用控制指令速度约束
        yaw_rad = math.radians(self.state.yaw_deg)

        cmd_vx = commanded_speed_mps * math.sin(yaw_rad)
        cmd_vy = commanded_speed_mps * math.cos(yaw_rad)

        alpha = self.cfg.imu_alpha_velocity

        self.state.vx = (1.0 - alpha) * cmd_vx + alpha * self.state.vx
        self.state.vy = (1.0 - alpha) * cmd_vy + alpha * self.state.vy

        self.state.x += self.state.vx * dt
        self.state.y += self.state.vy * dt

        self.state.timestamp = imu.timestamp
        self.state.source = "imu_dead_reckoning"

        self.last_imu = imu

        return self.state

    def ensure_local_frame(
        self,
        fix: GNSSFix,
        require_fixed_origin: bool = True,
    ) -> bool:
        """确保已经建立局部坐标系。

        默认要求第一次建立原点时必须是 RTK fixed。
        如果现场没有 fixed，可以把 require_fixed_origin=False。
        """

        if self.local_frame is not None:
            return True

        if not is_usable_for_navigation(fix, require_fixed=require_fixed_origin):
            return False

        self.local_frame = LocalTangentPlane(fix.point)
        self.origin_gnss = fix

        return True

    def _gnss_course_from_motion(
        self,
        local_pos: LocalPoint,
        timestamp: float,
    ) -> tuple[float | None, float]:
        """由连续 GNSS 位置计算航迹角。

        单天线 GNSS 不能在静止时给出车体航向；只有 ROV 有足够水平位移时，
        才能把“运动方向”作为航向观测来修正 IMU yaw 零偏。
        """

        if self.last_heading_local is None or self.last_heading_timestamp is None:
            self.last_heading_local = local_pos
            self.last_heading_timestamp = timestamp
            return None, 0.0

        dt = timestamp - self.last_heading_timestamp
        if dt <= 0.0 or dt > self.cfg.gnss_heading_max_dt_s:
            self.last_heading_local = local_pos
            self.last_heading_timestamp = timestamp
            return None, 0.0

        dx = local_pos.x - self.last_heading_local.x
        dy = local_pos.y - self.last_heading_local.y
        dist = math.hypot(dx, dy)

        if dist < self.cfg.gnss_heading_min_distance_m:
            return None, dist / dt if dt > 0.0 else 0.0

        speed = dist / dt
        course_deg = (math.degrees(math.atan2(dx, dy)) + 360.0) % 360.0

        # 用当前点作为下一次航迹角计算的起点，避免旧点长期累积导致航向滞后。
        self.last_heading_local = local_pos
        self.last_heading_timestamp = timestamp

        if speed < self.cfg.gnss_heading_min_speed_mps:
            return None, speed

        return course_deg, speed

    def _correct_yaw_offset_with_gnss_course(
        self,
        gnss_course_deg: float,
    ) -> None:
        """用 GNSS 航迹角自动估计 IMU 相对 yaw 到真北航向的零偏。"""

        measured_offset = wrap_angle_deg(gnss_course_deg - self.state.raw_yaw_deg)

        if not self.heading_offset_initialized:
            # 第一次有可信 GNSS 航迹角时，直接建立开机 yaw 零位与真北之间的关系。
            self.yaw_offset_deg = measured_offset % 360.0
            self.heading_offset_initialized = True
        else:
            offset_err = wrap_angle_deg(measured_offset - self.yaw_offset_deg)
            if abs(offset_err) <= self.cfg.max_heading_jump_deg:
                self.yaw_offset_deg = (
                    self.yaw_offset_deg
                    + self.cfg.rtk_heading_correction_gain * offset_err
                ) % 360.0

        self.state.yaw_offset_deg = self.yaw_offset_deg
        self.state.yaw_deg = (self.state.raw_yaw_deg + self.yaw_offset_deg) % 360.0
        self.state.gnss_course_deg = gnss_course_deg


    def correct_with_dvl(self, dvl: DVLMeasurement) -> NavState:
        """Fuse DVL body-frame bottom-track velocity into local navigation state.

        DVL vx/vy are treated as body-frame forward/right velocities. They are
        rotated into the local ENU plane using the current yaw. Position is then
        propagated for short underwater intervals. GNSS corrections still have
        higher absolute-position authority whenever available.
        """

        timestamp = float(getattr(dvl, "timestamp", 0.0) or 0.0)
        valid = bool(getattr(dvl, "valid", False) or getattr(dvl, "velocity_valid", False))
        if not valid or timestamp <= 0.0:
            self.last_dvl_valid = False
            return self.state

        if not self.initialized:
            self.initialize(self.cfg.manual_initial_yaw_deg, LocalPoint(0.0, 0.0, 0.0), timestamp)

        if self.last_dvl_timestamp is None:
            self.last_dvl_timestamp = timestamp
            self.last_dvl_valid = True
            return self.state

        dt = timestamp - self.last_dvl_timestamp
        self.last_dvl_timestamp = timestamp
        self.last_dvl_valid = True

        # 过滤异常间隔，避免断连恢复后一次性积分过大。
        if dt <= 0.0 or dt > max(2.0, float(self.cfg.dvl_stale_timeout_s) * 2.0):
            return self.state

        body_forward = float(getattr(dvl, "vx", 0.0) or 0.0)
        body_right = float(getattr(dvl, "vy", 0.0) or 0.0)
        body_vertical = float(getattr(dvl, "vz", 0.0) or 0.0)

        yaw = math.radians(self.state.yaw_deg)
        east_v = body_forward * math.sin(yaw) + body_right * math.cos(yaw)
        north_v = body_forward * math.cos(yaw) - body_right * math.sin(yaw)

        vg = max(0.0, min(1.0, float(getattr(self.cfg, "dvl_velocity_gain", 0.8))))
        pg = max(0.0, min(1.0, float(getattr(self.cfg, "dvl_position_gain", 0.75))))

        self.state.vx = (1.0 - vg) * self.state.vx + vg * east_v
        self.state.vy = (1.0 - vg) * self.state.vy + vg * north_v
        self.state.x += pg * self.state.vx * dt
        self.state.y += pg * self.state.vy * dt

        altitude = getattr(dvl, "altitude", None)
        if altitude is not None and altitude > 0.0:
            # 这里不把 altitude 直接等同于深度，只保留 vz 的短时趋势；深度仍以 ROV 传感器/定深控制为主。
            pass
        if abs(body_vertical) > 1e-6:
            self.state.z += pg * body_vertical * dt

        self.state.timestamp = timestamp
        self.state.source = "dvl_fused"
        return self.state

    def gnss_to_local(
        self,
        fix: GNSSFix,
        depth_m: float = 0.0,
    ) -> LocalPoint | None:
        """把 GNSS 经纬度转换为局部坐标。"""

        if self.local_frame is None:
            return None

        return self.local_frame.geo_to_local(
            fix.point,
            depth_m=depth_m,
        )

    def correct_with_gnss(
        self,
        fix: GNSSFix,
        depth_m: float = 0.0,
        require_fixed_origin: bool = True,
    ) -> NavState:
        """直接使用 GNSSFix 修正导航状态。

        这是适配前面串口采集 GNSSSerialReader 的主要入口。
        """

        if not is_usable_for_navigation(fix, require_fixed=False):
            return self.state

        if not self.ensure_local_frame(
            fix,
            require_fixed_origin=require_fixed_origin,
        ):
            return self.state

        local_pos = self.gnss_to_local(
            fix,
            depth_m=depth_m,
        )

        if local_pos is None:
            return self.state

        return self.correct_with_rtk(local_pos, fix)

    def correct_with_rtk(
        self,
        local_pos: LocalPoint,
        fix: GNSSFix,
    ) -> NavState:
        """使用已经转换好的局部 RTK/GNSS 坐标修正导航状态。"""

        if not self.initialized:
            yaw = (
                fix.heading_deg
                if fix.heading_deg is not None
                else self.cfg.manual_initial_yaw_deg
            )

            self.initialize(
                yaw,
                local_pos,
                fix.timestamp,
            )

        g = self.cfg.rtk_position_correction_gain if fix.is_fixed else 0.35

        self.state.x = (1.0 - g) * self.state.x + g * local_pos.x
        self.state.y = (1.0 - g) * self.state.y + g * local_pos.y
        self.state.z = local_pos.z

        # GNSS 航向来源：
        # 1) 若接入设备提供 RMC/VTG/双天线航向，可直接使用 fix.heading_deg；
        # 2) 当前串口解析主要是 GGA，没有 heading_deg，因此默认由连续 GNSS 位置计算航迹角。
        gnss_course = fix.heading_deg
        if gnss_course is None:
            gnss_course, _speed = self._gnss_course_from_motion(local_pos, fix.timestamp)

        if gnss_course is not None:
            rov_heading = self._apply_rov_heading_offset(gnss_course)
            self._correct_yaw_offset_with_gnss_course(rov_heading)
        else:
            self.state.gnss_course_deg = None

        self.state.timestamp = fix.timestamp
        self.state.source = "rtk_corrected" if fix.is_fixed else "rtk_float_corrected"

        if fix.is_fixed:
            self.last_fixed_gnss = fix

        return self.state