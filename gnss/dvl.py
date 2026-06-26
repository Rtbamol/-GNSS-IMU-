from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
import json
import socket
import time
from typing import Any


@dataclass
class DVLConfig:
    """DVL-A50 TCP/JSON reader configuration."""

    host: str = "192.168.194.95"
    port: int = 16171
    connect_timeout_s: float = 3.0
    recv_timeout_s: float = 0.01
    stale_timeout_s: float = 1.0
    log_dir: Path | str = "dvl_logs"
    print_raw: bool = False


@dataclass
class DVLMeasurement:
    timestamp: float
    vx: float = 0.0  # body forward, m/s
    vy: float = 0.0  # body right/lateral, m/s
    vz: float = 0.0  # body vertical, m/s
    altitude: float | None = None
    velocity_valid: bool = False
    raw: str = ""

    @property
    def valid(self) -> bool:
        return bool(self.velocity_valid)


class DVLTCPReader:
    """Read DVL-A50 newline-delimited JSON frames from TCP.

    The reader is deliberately non-fatal after construction: if the DVL is not
    connected, has no data, or reports velocity_valid=false, callers can still
    run the ROV with GNSS/IMU only and show the DVL status in the window.
    """

    def __init__(self, config: DVLConfig):
        self.config = config
        self.sock: socket.socket | None = None
        self._buffer = b""
        self.last_measurement: DVLMeasurement | None = None
        self.last_rx_time: float | None = None
        self._last_connect_attempt = 0.0
        self.status_text = "未连接"
        self._log_fp = None
        self._recent_raw: list[str] = []
        self.connect()

    def connect(self) -> None:
        self._last_connect_attempt = time.time()
        try:
            self.close()
            self.sock = socket.create_connection(
                (self.config.host, int(self.config.port)),
                timeout=float(self.config.connect_timeout_s),
            )
            self.sock.settimeout(float(self.config.recv_timeout_s))
            log_dir = Path(self.config.log_dir)
            log_dir.mkdir(parents=True, exist_ok=True)
            log_path = log_dir / f"dvl_raw_{time.strftime('%Y%m%d_%H%M%S')}.jsonl"
            self._log_fp = open(log_path, "a", encoding="utf-8")
            self.status_text = "等待数据"
        except Exception as exc:
            self.sock = None
            self.status_text = f"未连接: {exc}"

    def close(self) -> None:
        if self.sock is not None:
            try:
                self.sock.close()
            except Exception:
                pass
            self.sock = None
        if self._log_fp is not None:
            try:
                self._log_fp.close()
            except Exception:
                pass
            self._log_fp = None

    def _parse_json_line(self, line: str) -> DVLMeasurement | None:
        try:
            data: dict[str, Any] = json.loads(line)
        except Exception:
            self.status_text = "JSON解析失败"
            return None

        valid = bool(data.get("velocity_valid", data.get("valid", False)))
        try:
            meas = DVLMeasurement(
                timestamp=time.time(),
                vx=float(data.get("vx", 0.0) or 0.0),
                vy=float(data.get("vy", 0.0) or 0.0),
                vz=float(data.get("vz", 0.0) or 0.0),
                altitude=None if data.get("altitude") is None else float(data.get("altitude")),
                velocity_valid=valid,
                raw=line,
            )
        except Exception:
            self.status_text = "字段解析失败"
            return None

        self.last_measurement = meas
        self.last_rx_time = meas.timestamp
        self.status_text = "有效" if meas.valid else "失效/未锁底"
        return meas

    def poll(self) -> DVLMeasurement | None:
        if self.sock is None:
            # 不阻塞主循环，周期性尝试重连，窗口继续显示 DVL 无连接/无数据。
            if time.time() - self._last_connect_attempt > 3.0:
                self.connect()
            return None

        latest: DVLMeasurement | None = None
        while True:
            try:
                chunk = self.sock.recv(4096)
                if not chunk:
                    self.status_text = "连接断开"
                    self.close()
                    return latest
                self._buffer += chunk
            except socket.timeout:
                break
            except BlockingIOError:
                break
            except Exception as exc:
                self.status_text = f"读取错误: {exc}"
                return latest

            while b"\n" in self._buffer:
                raw, self._buffer = self._buffer.split(b"\n", 1)
                line = raw.decode("utf-8", errors="replace").strip()
                if not line:
                    continue
                if self.config.print_raw:
                    print("DVL RAW:", line)
                if self._log_fp is not None:
                    self._log_fp.write(line + "\n")
                    self._log_fp.flush()
                self._recent_raw.append(line)
                if len(self._recent_raw) > 100:
                    self._recent_raw = self._recent_raw[-100:]
                meas = self._parse_json_line(line)
                if meas is not None:
                    latest = meas

        if self.last_rx_time is None:
            self.status_text = "无数据"
        elif time.time() - self.last_rx_time > float(self.config.stale_timeout_s):
            self.status_text = "无新数据"
        return latest

    def is_fresh_valid(self) -> bool:
        return (
            self.last_measurement is not None
            and self.last_measurement.valid
            and self.last_rx_time is not None
            and time.time() - self.last_rx_time <= float(self.config.stale_timeout_s)
        )

    def pop_recent_raw_lines(self) -> list[str]:
        lines = self._recent_raw[:]
        self._recent_raw.clear()
        return lines
