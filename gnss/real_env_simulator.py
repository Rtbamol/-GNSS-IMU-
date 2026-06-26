from __future__ import annotations

import argparse
import json
import math
import random
import socket
import threading
import time
from dataclasses import dataclass
from datetime import datetime, date, timezone
from pathlib import Path

from rov_control.geo import GeoPoint, LocalPoint, LocalTangentPlane

PWM_MID = 1500


def clamp(v: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, v))


def u16_at(data: bytes, idx: int, default: int = PWM_MID) -> int:
    if len(data) < idx + 2:
        return default
    return (data[idx] << 8) | data[idx + 1]


def wrap360(a: float) -> float:
    return a % 360.0


def signed_triplet(value: float, scale: float) -> bytes:
    sign = 0x10 if value < 0 else 0x01
    mag = int(round(abs(value) * scale))
    return bytes([sign, (mag // 100) & 0xFF, mag % 100])


def nmea_checksum(sentence_without_dollar_star: str) -> str:
    c = 0
    for ch in sentence_without_dollar_star:
        c ^= ord(ch)
    return f"{c:02X}"


def decimal_to_nmea_dm(decimal_deg: float, is_lat: bool) -> tuple[str, str]:
    hemi = "N" if is_lat and decimal_deg >= 0 else "S" if is_lat else "E" if decimal_deg >= 0 else "W"
    d = abs(decimal_deg)
    deg = int(d)
    minutes = (d - deg) * 60.0
    if is_lat:
        return f"{deg:02d}{minutes:08.5f}", hemi
    return f"{deg:03d}{minutes:08.5f}", hemi


@dataclass
class SimState:
    x: float = 0.0       # East, m
    y: float = 0.0       # North, m
    z: float = 0.0       # Depth down, m

    # true_heading_deg 是艇体/运动使用的真北航向；开机时可以不是 0。
    true_heading_deg: float = 65.0
    # imu_yaw_deg 是无磁力计 IMU 上报给控制程序的相对 yaw；开机方向为 0。
    imu_yaw_deg: float = 0.0

    speed_mps: float = 0.0
    yaw_rate_dps: float = 0.0
    vertical_rate_mps: float = 0.0
    battery: int = 85
    last_cmd_hex: str = ""


class RealEnvSimulator:
    """GNSS raw.txt + ROV TCP sensor/command simulator.

    It emulates two inputs used by rov_control.app:
    1) ROV TCP server: receives 24-byte channel commands and returns 32-byte sensor frames.
    2) GNSS logger: appends GNGGA lines to base_dir/cid/cid/YYYY-MM-DD/raw.txt.

    Important for yaw correction tests:
    - GNSS position is generated from true_heading_deg.
    - Sensor frame heading is imu_yaw_deg, whose startup direction is 0 deg.
    """

    def __init__(
        self,
        host: str,
        port: int,
        gnss_base_dir: Path,
        cid: str,
        origin_lat: float,
        origin_lon: float,
        origin_alt: float,
        sensor_hz: float = 10.0,
        gnss_hz: float = 1.0,
        max_speed_mps: float = 0.5,
        initial_true_heading_deg: float = 65.0,
        imu_yaw_bias_dps: float = 0.0,
        gnss_noise_m: float = 0.0,
        force_speed_mps: float | None = None,
        dvl_enabled: bool = False,
        dvl_host: str = "127.0.0.1",
        dvl_port: int = 16171,
        dvl_hz: float = 10.0,
    ):
        self.host = host
        self.port = port
        self.gnss_base_dir = gnss_base_dir
        self.cid = cid
        self.sensor_period = 1.0 / sensor_hz
        self.gnss_period = 1.0 / gnss_hz
        self.max_speed_mps = max_speed_mps
        self.imu_yaw_bias_dps = imu_yaw_bias_dps
        self.gnss_noise_m = gnss_noise_m
        self.force_speed_mps = force_speed_mps
        self.dvl_enabled = bool(dvl_enabled)
        self.dvl_host = dvl_host
        self.dvl_port = int(dvl_port)
        self.dvl_period = 1.0 / max(0.1, float(dvl_hz))
        self._dvl_start_time = time.time()
        self.state = SimState(true_heading_deg=wrap360(initial_true_heading_deg), imu_yaw_deg=0.0)
        self.lock = threading.Lock()
        self.stop_event = threading.Event()
        self.ltp = LocalTangentPlane(GeoPoint(origin_lat, origin_lon, origin_alt))
        self.raw_path = self._raw_path_for_today()

    def _raw_path_for_today(self) -> Path:
        return self.gnss_base_dir / self.cid / self.cid / date.today().strftime("%Y-%m-%d") / "raw.txt"

    def update_from_command(self, cmd: bytes) -> None:
        # ChannelCommand bytes: FF, forward[1:3], yaw[3:5], lateral[5:7], vertical[7:9], ..., arm[18], ..., AA
        if len(cmd) < 19 or cmd[0] != 0xFF:
            return
        forward_pwm = u16_at(cmd, 1)
        yaw_pwm = u16_at(cmd, 3)
        vertical_pwm = u16_at(cmd, 7)
        arm = cmd[18]
        with self.lock:
            if arm == 0:
                self.state.speed_mps = 0.0
                self.state.yaw_rate_dps = 0.0
                self.state.vertical_rate_mps = 0.0
            else:
                if self.force_speed_mps is None:
                    self.state.speed_mps = clamp(
                        (forward_pwm - PWM_MID) / 420.0 * self.max_speed_mps,
                        -self.max_speed_mps,
                        self.max_speed_mps,
                    )
                else:
                    # 用于只测试 GNSS 航向纠偏：即使控制器暂时输出 1500，也强制向前运动。
                    self.state.speed_mps = clamp(self.force_speed_mps, -self.max_speed_mps, self.max_speed_mps)

                self.state.yaw_rate_dps = clamp((yaw_pwm - PWM_MID) / 500.0 * 60.0, -60.0, 60.0)
                self.state.vertical_rate_mps = clamp((vertical_pwm - PWM_MID) / 500.0 * 0.4, -0.4, 0.4)
            self.state.last_cmd_hex = " ".join(f"{b:02x}" for b in cmd)

    def step_physics(self, dt: float) -> None:
        with self.lock:
            s = self.state
            s.true_heading_deg = wrap360(s.true_heading_deg + s.yaw_rate_dps * dt)
            s.imu_yaw_deg = wrap360(s.imu_yaw_deg + (s.yaw_rate_dps + self.imu_yaw_bias_dps) * dt)

            yaw = math.radians(s.true_heading_deg)
            s.x += s.speed_mps * math.sin(yaw) * dt
            s.y += s.speed_mps * math.cos(yaw) * dt
            s.z = clamp(s.z + s.vertical_rate_mps * dt, 0.0, 20.0)

    def make_sensor_frame(self) -> bytes:
        with self.lock:
            s = SimState(**self.state.__dict__)
        # 关键：传感器帧上报 IMU 相对 yaw，不上报 true heading。
        heading = int(round(s.imu_yaw_deg)) % 360
        depth_cm = int(round(s.z * 100.0))
        b = bytearray(32)
        b[0] = 0xAA
        b[1] = (heading >> 8) & 0xFF
        b[2] = heading & 0xFF
        b[3], b[4] = 0x01, 0       # pitch +0 deg
        b[5], b[6] = 0x01, 0       # roll +0 deg
        b[7], b[8], b[9] = (depth_cm >> 16) & 0xFF, (depth_cm >> 8) & 0xFF, depth_cm & 0xFF
        b[10] = 17                 # water temp
        b[11] = 1                  # no leak
        b[12:15] = signed_triplet(0.0, 1000.0)
        b[15:18] = signed_triplet(0.0, 1000.0)
        b[18:21] = signed_triplet(0.98, 1000.0)
        b[21:24] = signed_triplet(0.0, 100.0)
        b[24:27] = signed_triplet(0.0, 100.0)
        b[27:30] = signed_triplet(s.yaw_rate_dps, 100.0)
        b[30] = int(clamp(s.battery, 0, 100))
        b[31] = 0xFF
        return bytes(b)

    def make_gga_line(self) -> str:
        with self.lock:
            s = SimState(**self.state.__dict__)
        x = s.x + random.gauss(0.0, self.gnss_noise_m) if self.gnss_noise_m > 0 else s.x
        y = s.y + random.gauss(0.0, self.gnss_noise_m) if self.gnss_noise_m > 0 else s.y
        geo = self.ltp.local_to_geo(LocalPoint(x, y, s.z))
        now = datetime.now()
        utc = datetime.now(timezone.utc)
        lat_dm, lat_hemi = decimal_to_nmea_dm(geo.lat, True)
        lon_dm, lon_hemi = decimal_to_nmea_dm(geo.lon, False)
        body = (
            f"GNGGA,{utc:%H%M%S}.00,{lat_dm},{lat_hemi},{lon_dm},{lon_hemi},"
            f"4,18,0.60,{geo.alt:.3f},M,0.000,M,0.5,0000"
        )
        sentence = f"${body}*{nmea_checksum(body)}"
        return f"[{now:%Y-%m-%d %H:%M:%S.%f}] {sentence}\n"

    def _dvl_phase(self) -> str:
        """Return the simulated DVL state: valid, invalid, or nodata.

        The cycle deliberately injects two abnormal cases so the app can verify
        fallback behavior:
        - invalid: JSON frames continue but velocity_valid=false.
        - nodata: TCP connection stays open but no frame is sent.
        """
        elapsed = time.time() - self._dvl_start_time
        phase_t = elapsed % 30.0
        if phase_t < 16.0:
            return "valid"
        if phase_t < 23.0:
            return "invalid"
        return "nodata"

    def make_dvl_line(self) -> str | None:
        phase = self._dvl_phase()
        if phase == "nodata":
            return None

        with self.lock:
            s = SimState(**self.state.__dict__)

        # DVL-A50 reader expects body-frame bottom-track velocity in JSON.
        # vx: body forward, vy: body right/lateral, vz: vertical.
        noise = 0.01 if phase == "valid" else 0.0
        payload = {
            "type": "sim_dvl",
            "time": time.time(),
            "velocity_valid": phase == "valid",
            "vx": s.speed_mps + random.gauss(0.0, noise),
            "vy": random.gauss(0.0, noise),
            "vz": s.vertical_rate_mps + random.gauss(0.0, noise),
            "altitude": max(0.2, 20.0 - s.z),
            "event": phase,
        }
        return json.dumps(payload, ensure_ascii=False) + "\n"

    def dvl_server_loop(self) -> None:
        srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        srv.bind((self.dvl_host, self.dvl_port))
        srv.listen(1)
        srv.settimeout(0.5)
        print(f"DVL TCP simulator listening on {self.dvl_host}:{self.dvl_port}")
        print("DVL cycle: 0-16s 有效, 16-23s 失效/未锁底, 23-30s 无数据, 循环")
        conn = None
        try:
            while not self.stop_event.is_set():
                if conn is None:
                    try:
                        conn, addr = srv.accept()
                        conn.settimeout(0.001)
                        print(f"DVL client connected: {addr}")
                    except socket.timeout:
                        continue

                line = self.make_dvl_line()
                if line is not None:
                    try:
                        conn.sendall(line.encode("utf-8"))
                    except (ConnectionError, OSError):
                        try:
                            conn.close()
                        except Exception:
                            pass
                        conn = None
                time.sleep(self.dvl_period)
        finally:
            if conn is not None:
                try:
                    conn.close()
                except Exception:
                    pass
            srv.close()

    def gnss_writer_loop(self) -> None:
        self.raw_path.parent.mkdir(parents=True, exist_ok=True)
        with open(self.raw_path, "a", encoding="utf-8") as f:
            while not self.stop_event.is_set():
                f.write(self.make_gga_line())
                f.flush()
                time.sleep(self.gnss_period)

    def tcp_server_loop(self) -> None:
        srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        srv.bind((self.host, self.port))
        srv.listen(1)
        srv.settimeout(0.5)
        print(f"ROV TCP simulator listening on {self.host}:{self.port}")
        print(f"GNSS raw.txt writing to: {self.raw_path}")
        conn = None
        try:
            while not self.stop_event.is_set():
                try:
                    conn, addr = srv.accept()
                    print(f"client connected: {addr}")
                    conn.settimeout(0.001)
                    break
                except socket.timeout:
                    continue
            last = time.time()
            while not self.stop_event.is_set() and conn is not None:
                now = time.time()
                dt = now - last
                last = now
                try:
                    data = conn.recv(1024)
                    if data:
                        for i in range(0, len(data), 24):
                            self.update_from_command(data[i:i+24])
                except socket.timeout:
                    pass
                except (ConnectionError, OSError):
                    break
                self.step_physics(dt)
                try:
                    conn.sendall(self.make_sensor_frame())
                except (ConnectionError, OSError):
                    break
                time.sleep(self.sensor_period)
        finally:
            if conn:
                conn.close()
            srv.close()

    def status_loop(self) -> None:
        while not self.stop_event.is_set():
            with self.lock:
                s = SimState(**self.state.__dict__)
            dvl_text = self._dvl_phase() if self.dvl_enabled else "off"
            print(
                f"sim state: x={s.x:6.2f} y={s.y:6.2f} z={s.z:4.2f} "
                f"true_hdg={s.true_heading_deg:6.1f} imu_yaw={s.imu_yaw_deg:6.1f} "
                f"speed={s.speed_mps:4.2f} yaw_rate={s.yaw_rate_dps:5.1f} dvl={dvl_text}"
            )
            time.sleep(2.0)

    def run(self) -> None:
        threads = [
            threading.Thread(target=self.gnss_writer_loop, daemon=True),
            threading.Thread(target=self.status_loop, daemon=True),
        ]
        if self.dvl_enabled:
            threads.append(threading.Thread(target=self.dvl_server_loop, daemon=True))
        for t in threads:
            t.start()
        try:
            self.tcp_server_loop()
        except KeyboardInterrupt:
            pass
        finally:
            self.stop_event.set()


def main() -> None:
    parser = argparse.ArgumentParser(description="ROV + GNSS real-environment software-in-the-loop simulator")
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=8887)
    parser.add_argument("--gnss-base-dir", default="sim_rtk_log")
    parser.add_argument("--cid", default="864865086384444")
    parser.add_argument("--origin-lat", type=float, default=34.000000)
    parser.add_argument("--origin-lon", type=float, default=108.000000)
    parser.add_argument("--origin-alt", type=float, default=0.0)
    parser.add_argument("--max-speed", type=float, default=0.5)
    parser.add_argument("--initial-true-heading", type=float, default=65.0)
    parser.add_argument("--imu-yaw-bias-dps", type=float, default=0.0)
    parser.add_argument("--gnss-noise-m", type=float, default=0.0)
    parser.add_argument("--force-speed", type=float, default=None, help="force forward speed for yaw-correction-only tests")
    parser.add_argument("--dvl", action="store_true", help="启动模拟 DVL TCP/JSON 数据；不加此参数表示没有 DVL 数据")
    parser.add_argument("--dvl-host", default="127.0.0.1", help="模拟 DVL TCP 监听 IP，默认 127.0.0.1")
    parser.add_argument("--dvl-port", type=int, default=16171, help="模拟 DVL TCP 端口，默认 16171")
    parser.add_argument("--dvl-hz", type=float, default=10.0, help="模拟 DVL 输出频率，默认 10Hz")
    args = parser.parse_args()
    sim = RealEnvSimulator(
        host=args.host,
        port=args.port,
        gnss_base_dir=Path(args.gnss_base_dir),
        cid=args.cid,
        origin_lat=args.origin_lat,
        origin_lon=args.origin_lon,
        origin_alt=args.origin_alt,
        max_speed_mps=args.max_speed,
        initial_true_heading_deg=args.initial_true_heading,
        imu_yaw_bias_dps=args.imu_yaw_bias_dps,
        gnss_noise_m=args.gnss_noise_m,
        force_speed_mps=args.force_speed,
        dvl_enabled=args.dvl,
        dvl_host=args.dvl_host,
        dvl_port=args.dvl_port,
        dvl_hz=args.dvl_hz,
    )
    sim.run()


if __name__ == "__main__":
    main()
