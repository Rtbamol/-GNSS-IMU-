from __future__ import annotations
from dataclasses import dataclass
import time
from rov_control.protocol import ChannelCommand, clamp_int
from rov_control.geo import wrap_angle_deg
from rov_control.planner import PathStatus
from rov_control.imu_nav import NavState
from rov_control.config import ROVConfig


@dataclass
class ControlOutput:
    command: ChannelCommand
    mode: str
    target_heading_deg: float
    actual_heading_deg: float
    target_speed_mps: float
    target_depth_m: float
    actual_depth_m: float
    yaw_error_deg: float
    timestamp: float


class HighLevelController:
    """上位机高级目标量控制。

    上位机只生成通道级目标，不直接分配每个推进器；STM32 负责 PID 和推力分配。
    """

    def __init__(self, config: ROVConfig):
        self.cfg = config
        self.enabled = False
        self.paused = False
        self.estop = False
        self.last_send_time = 0.0

    def arm(self) -> None:
        self.enabled = True
        self.estop = False

    def disarm(self) -> None:
        self.enabled = False

    def emergency_stop(self) -> ChannelCommand:
        self.estop = True
        self.enabled = False
        return ChannelCommand.neutral(arm=0)

    def make_command(
        self,
        nav: NavState,
        status: PathStatus,
        target_depth_m: float | None = None,
        heading_hold: bool = True,
        depth_hold: bool = True,
    ) -> ControlOutput:
        now = time.time()
        depth_target = self.cfg.target_depth_m if target_depth_m is None else target_depth_m
        if self.estop or not self.enabled or self.paused or status.arrived:
            cmd = ChannelCommand.neutral(arm=0 if self.estop else int(self.enabled))
            return ControlOutput(cmd, "stop", status.target_heading_deg, nav.yaw_deg, 0.0, depth_target, nav.z, 0.0, now)

        yaw_err = wrap_angle_deg(status.target_heading_deg - nav.yaw_deg)
        yaw_pwm = self.cfg.pwm_mid + int(3.2 * yaw_err)
        forward_pwm = self.cfg.pwm_mid + int(420 * min(status.target_speed_mps, self.cfg.max_speed_mps) / max(self.cfg.max_speed_mps, 1e-6))
        depth_err = depth_target - nav.z
        vertical_pwm = self.cfg.pwm_mid + int(160 * depth_err)

        # yaw_pwm = self.cfg.pwm_mid
        # forward_pwm = self.cfg.pwm_mid
        # vertical_pwm = self.cfg.pwm_mid

        cmd = ChannelCommand(
            forward=clamp_int(forward_pwm, self.cfg.pwm_min, self.cfg.pwm_max),
            yaw=clamp_int(yaw_pwm, self.cfg.pwm_min, self.cfg.pwm_max),
            lateral=self.cfg.pwm_mid,
            vertical=clamp_int(vertical_pwm, self.cfg.pwm_min, self.cfg.pwm_max),
            light=0,
            heading_hold=1 if heading_hold else 0,
            depth_hold=1 if depth_hold and abs(depth_target) > 0.05 else 0,
            gear=2,
            arm=1,
        )
        self.last_send_time = now
        return ControlOutput(cmd, "auto", status.target_heading_deg, nav.yaw_deg, status.target_speed_mps, depth_target, nav.z, yaw_err, now)
