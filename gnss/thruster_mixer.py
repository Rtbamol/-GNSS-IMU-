from __future__ import annotations

from dataclasses import asdict, dataclass

from rov_control.config import ROVConfig
from rov_control.protocol import ChannelCommand, MotorCommand, PWM_MAX, PWM_MID, PWM_MIN, clamp_int


@dataclass(frozen=True)
class ThrusterLayout:
    """8 推进器显示标签。

    注意：当前 ROV 协议的自动控制仍然发送 ChannelCommand，底层 STM32 做真实混控。
    这里的 8 路 PWM 用于 App 可视化/日志预览。若实船推进器方向与这里不同，
    只需要调整 DEFAULT_MIX_SIGN 中每个通道的符号。
    """

    l1: str = "L1 左前水平"
    l2: str = "L2 左后水平"
    l3: str = "L3 左前垂直"
    l4: str = "L4 左后垂直"
    r1: str = "R1 右前水平"
    r2: str = "R2 右后水平"
    r3: str = "R3 右前垂直"
    r4: str = "R4 右后垂直"


# forward/yaw/lateral/vertical sign matrix for preview mixing.
# The horizontal signs follow a common vectored 4-thruster assumption.
# The vertical signs assume all 4 vertical thrusters push in the same positive direction.
DEFAULT_MIX_SIGN: dict[str, tuple[float, float, float, float]] = {
    "l1": (1.0, 1.0, 1.0, 0.0),
    "l2": (1.0, 1.0, -1.0, 0.0),
    "r1": (1.0, -1.0, -1.0, 0.0),
    "r2": (1.0, -1.0, 1.0, 0.0),
    "l3": (0.0, 0.0, 0.0, 1.0),
    "l4": (0.0, 0.0, 0.0, 1.0),
    "r3": (0.0, 0.0, 0.0, 1.0),
    "r4": (0.0, 0.0, 0.0, 1.0),
}


def _limits(cfg: ROVConfig | None = None) -> tuple[int, int, int]:
    if cfg is None:
        return PWM_MIN, PWM_MID, PWM_MAX
    return cfg.pwm_min, cfg.pwm_mid, cfg.pwm_max


def _axis_from_pwm(pwm: int, cfg: ROVConfig | None = None) -> float:
    pwm_min, pwm_mid, pwm_max = _limits(cfg)
    span = max(pwm_max - pwm_mid, pwm_mid - pwm_min, 1)
    return max(-1.0, min(1.0, (float(pwm) - pwm_mid) / span))


def _pwm_from_axis(axis: float, cfg: ROVConfig | None = None) -> int:
    pwm_min, pwm_mid, pwm_max = _limits(cfg)
    span = max(pwm_max - pwm_mid, pwm_mid - pwm_min, 1)
    return clamp_int(round(pwm_mid + max(-1.0, min(1.0, axis)) * span), pwm_min, pwm_max)


def channel_axes(cmd: ChannelCommand, cfg: ROVConfig | None = None) -> dict[str, float]:
    """Return normalized channel axes in [-1, 1]."""

    return {
        "forward": _axis_from_pwm(cmd.forward, cfg),
        "yaw": _axis_from_pwm(cmd.yaw, cfg),
        "lateral": _axis_from_pwm(cmd.lateral, cfg),
        "vertical": _axis_from_pwm(cmd.vertical, cfg),
    }


def mix_channel_to_motor_command(
    cmd: ChannelCommand,
    cfg: ROVConfig | None = None,
    mix_sign: dict[str, tuple[float, float, float, float]] | None = None,
) -> MotorCommand:
    """Estimate per-thruster PWM from channel command for UI/logging.

    This does not replace the STM32 mixer. It is intentionally separated so the
    real-world command path remains ChannelCommand unless the firmware is changed.
    """

    signs = mix_sign or DEFAULT_MIX_SIGN
    axes = channel_axes(cmd, cfg)
    f = axes["forward"]
    y = axes["yaw"]
    lat = axes["lateral"]
    v = axes["vertical"]

    raw: dict[str, float] = {}
    for name, (sf, sy, sl, sv) in signs.items():
        raw[name] = sf * f + sy * y + sl * lat + sv * v

    peak = max(1.0, *(abs(x) for x in raw.values()))
    scaled = {name: value / peak for name, value in raw.items()}

    return MotorCommand(
        l1=_pwm_from_axis(scaled["l1"], cfg),
        l2=_pwm_from_axis(scaled["l2"], cfg),
        l3=_pwm_from_axis(scaled["l3"], cfg),
        l4=_pwm_from_axis(scaled["l4"], cfg),
        r1=_pwm_from_axis(scaled["r1"], cfg),
        r2=_pwm_from_axis(scaled["r2"], cfg),
        r3=_pwm_from_axis(scaled["r3"], cfg),
        r4=_pwm_from_axis(scaled["r4"], cfg),
        light=cmd.light,
        gimbal1=cmd.gimbal1,
        gimbal2=cmd.gimbal2,
        gripper=cmd.gripper,
        arm=cmd.arm,
    )


def motor_pwm_dict(motor: MotorCommand) -> dict[str, int]:
    values = asdict(motor)
    return {k: int(values[k]) for k in ("l1", "l2", "l3", "l4", "r1", "r2", "r3", "r4")}


def preview_thruster_pwms(cmd: ChannelCommand, cfg: ROVConfig | None = None) -> dict[str, int]:
    """Convenience helper used by UI and logger."""

    return motor_pwm_dict(mix_channel_to_motor_command(cmd, cfg))
