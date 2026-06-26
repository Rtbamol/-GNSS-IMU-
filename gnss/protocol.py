from __future__ import annotations
from dataclasses import dataclass
from typing import Optional


FRAME_HEAD_CMD = 0xFF
FRAME_TAIL_CHANNEL = 0xAA
FRAME_TAIL_MOTOR = 0xBB
FRAME_HEAD_SENSOR = 0xAA
FRAME_TAIL_SENSOR = 0xFF

PWM_MIN = 1200
PWM_MID = 1500
PWM_MAX = 1800


class ProtocolError(ValueError):
    pass


def clamp_int(value: int, lo: int, hi: int) -> int:
    return max(lo, min(hi, int(value)))


def u16be(value: int) -> bytes:
    value = clamp_int(value, 0, 0xFFFF)
    return bytes([(value >> 8) & 0xFF, value & 0xFF])


def u24be_to_int(b: bytes) -> int:
    if len(b) != 3:
        raise ProtocolError("u24 数据长度必须为 3")
    return (b[0] << 16) | (b[1] << 8) | b[2]


def sign_magnitude_2(sign_byte: int, mag_byte: int) -> float:
    """解析俯仰/横滚：0x10 为负，0x01 为正。"""
    sign = -1.0 if sign_byte in (0x10, 0x11) else 1.0
    return sign * float(mag_byte)


def sign_imu_value(sign_byte: int, b1: int, b2: int, scale: float) -> float:
    """解析三字节 IMU 值。

    协议例：01 23 04 表示 +2.304g，因此加速度 scale=1000；
    10 50 23 表示 -50.23deg/s，因此陀螺 scale=100。
    对 0x10、0x11 均按负号处理，兼容文档中的 10/11 写法。
    """
    sign = -1.0 if sign_byte in (0x10, 0x11) else 1.0
    return sign * ((b1 * 100 + b2) / scale)


@dataclass
class ChannelCommand:
    """通道级控制帧，长度 24 字节，帧头 FF，帧尾 AA。"""
    forward: int = PWM_MID
    yaw: int = PWM_MID
    lateral: int = PWM_MID
    vertical: int = PWM_MID
    light: int = 0           # 0~100
    heading_hold: int = 0    # 0/1
    depth_hold: int = 0      # 0/1
    gear: int = 1            # 1~3
    gimbal1: int = 0         # 0x00/0x01/0x11/0x20
    gimbal2: int = 0
    gripper: int = 0         # 0 闭合，1 张开
    arm: int = 0             # 0 未开启，非 0 开启

    def to_bytes(self) -> bytes:
        buf = bytearray(24)
        buf[0] = FRAME_HEAD_CMD
        buf[1:3] = u16be(clamp_int(self.forward, PWM_MIN, PWM_MAX))
        buf[3:5] = u16be(clamp_int(self.yaw, PWM_MIN, PWM_MAX))
        buf[5:7] = u16be(clamp_int(self.lateral, PWM_MIN, PWM_MAX))
        buf[7:9] = u16be(clamp_int(self.vertical, PWM_MIN, PWM_MAX))
        buf[9] = clamp_int(self.light, 0, 100)
        buf[10] = 1 if self.heading_hold else 0
        buf[11] = 1 if self.depth_hold else 0
        buf[12] = clamp_int(self.gear, 1, 3)
        buf[13] = self.gimbal1 & 0xFF
        buf[14] = self.gimbal2 & 0xFF
        buf[15] = 1 if self.gripper else 0
        buf[16] = 0
        buf[17] = 0
        buf[18] = self.arm & 0xFF
        buf[19:23] = b"\x00\x00\x00\x00"
        buf[23] = FRAME_TAIL_CHANNEL
        return bytes(buf)

    @classmethod
    def neutral(cls, arm: int = 0) -> "ChannelCommand":
        return cls(arm=arm)


@dataclass
class MotorCommand:
    """电机级控制帧，长度 24 字节，帧头 FF，帧尾 BB。"""
    l1: int = PWM_MID
    l2: int = PWM_MID
    l3: int = PWM_MID
    l4: int = PWM_MID
    r1: int = PWM_MID
    r2: int = PWM_MID
    r3: int = PWM_MID
    r4: int = PWM_MID
    light: int = 0
    gimbal1: int = 0
    gimbal2: int = 0
    gripper: int = 0
    arm: int = 0

    def to_bytes(self) -> bytes:
        vals = [self.l1, self.l2, self.l3, self.l4, self.r1, self.r2, self.r3, self.r4]
        buf = bytearray(24)
        buf[0] = FRAME_HEAD_CMD
        idx = 1
        for v in vals:
            buf[idx:idx + 2] = u16be(clamp_int(v, PWM_MIN, PWM_MAX))
            idx += 2
        buf[17] = clamp_int(self.light, 0, 100)
        buf[18] = self.gimbal1 & 0xFF
        buf[19] = self.gimbal2 & 0xFF
        buf[20] = 1 if self.gripper else 0
        buf[21] = self.arm & 0xFF
        buf[22] = 0
        buf[23] = FRAME_TAIL_MOTOR
        return bytes(buf)


@dataclass
class SensorFrame:
    timestamp: float
    heading_deg: float
    pitch_deg: float
    roll_deg: float
    depth_m: float
    water_temp_c: int
    no_leak: bool
    ax_g: float
    ay_g: float
    az_g: float
    gx_dps: float
    gy_dps: float
    gz_dps: float
    battery_percent: int

    @classmethod
    def from_bytes(cls, data: bytes, timestamp: float = 0.0) -> "SensorFrame":
        if len(data) != 32:
            raise ProtocolError(f"传感器帧长度应为 32，实际 {len(data)}")
        if data[0] != FRAME_HEAD_SENSOR or data[31] != FRAME_TAIL_SENSOR:
            raise ProtocolError("传感器帧头/帧尾错误")
        heading = ((data[1] << 8) | data[2]) % 361
        pitch = sign_magnitude_2(data[3], data[4])
        roll = sign_magnitude_2(data[5], data[6])
        depth_m = u24be_to_int(data[7:10]) / 100.0
        return cls(
            timestamp=timestamp,
            heading_deg=float(heading),
            pitch_deg=pitch,
            roll_deg=roll,
            depth_m=depth_m,
            water_temp_c=data[10],
            no_leak=(data[11] == 0x01),
            ax_g=sign_imu_value(data[12], data[13], data[14], 1000.0),
            ay_g=sign_imu_value(data[15], data[16], data[17], 1000.0),
            az_g=sign_imu_value(data[18], data[19], data[20], 1000.0),
            gx_dps=sign_imu_value(data[21], data[22], data[23], 100.0),
            gy_dps=sign_imu_value(data[24], data[25], data[26], 100.0),
            gz_dps=sign_imu_value(data[27], data[28], data[29], 100.0),
            battery_percent=data[30],
        )


def extract_sensor_frames(buffer: bytearray) -> list[bytes]:
    """从 TCP 字节流中提取完整 32 字节传感器帧。"""
    frames: list[bytes] = []
    while True:
        try:
            start = buffer.index(FRAME_HEAD_SENSOR)
        except ValueError:
            buffer.clear()
            break
        if start > 0:
            del buffer[:start]
        if len(buffer) < 32:
            break
        candidate = bytes(buffer[:32])
        if candidate[-1] == FRAME_TAIL_SENSOR:
            frames.append(candidate)
            del buffer[:32]
        else:
            del buffer[0]
    return frames
