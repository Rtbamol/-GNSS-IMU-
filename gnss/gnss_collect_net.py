from __future__ import annotations

import socket
import time
from dataclasses import dataclass, field
from pathlib import Path
from datetime import datetime
from typing import Iterator

from rov_control.gnss import GNSSFix
from rov_control.gnss_file import extract_nmea_from_log_line, parse_gga_sentence


@dataclass
class GNSSNetConfig:
    """GNSS 网口采集配置。默认值与 config.ROVConfig 保持一致。"""

    host: str = "192.168.7.7"
    port: int = 8848
    connect_timeout_s: float = 5.0
    recv_timeout_s: float = 1.0
    log_dir: Path = field(default_factory=lambda: Path("gnss_logs_net"))
    log_prefix: str = "gnss_data"
    print_raw_bytes: bool = False

    def make_log_file(self) -> Path:
        self.log_dir.mkdir(parents=True, exist_ok=True)
        start_time = datetime.now().strftime("%Y%m%d_%H%M%S")
        return self.log_dir / f"{self.log_prefix}_{start_time}.txt"


class GNSSNetReader:
    """从 GNSS 网口实时接收 NMEA 数据，同时写入日志文件，并解析 GGA。

    poll() 非阻塞：
    - 有新 GGA 数据时，返回最新一条 GNSSFix；
    - 没有新数据时，返回 None。
    """

    def __init__(self, config: GNSSNetConfig, validate_checksum: bool = True):
        self.config = config
        self.validate_checksum = validate_checksum
        self.log_file: Path | None = None
        self._sock: socket.socket | None = None
        self._log_fp = None
        self._text_buffer = ""

        self.recent_raw_lines: list[str] = []
        self.max_recent_raw_lines = 80

    def open(self) -> None:
        if self._sock is not None:
            return

        print(f"Trying to connect to GNSS net {self.config.host}:{self.config.port}...")
        self.log_file = self.config.make_log_file()

        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.settimeout(self.config.connect_timeout_s)
        sock.connect((self.config.host, self.config.port))
        sock.settimeout(self.config.recv_timeout_s)

        self._sock = sock
        self._log_fp = open(self.log_file, "a", encoding="utf-8")

        print(f"Successfully connected to GNSS net {self.config.host}:{self.config.port}")
        print(f"Data will be saved to: {self.log_file}")

        self._log_fp.write(f"Start time: {datetime.now()}\n")
        self._log_fp.write(f"IP: {self.config.host}\n")
        self._log_fp.write(f"Port: {self.config.port}\n")
        self._log_fp.write("=" * 80 + "\n")
        self._log_fp.flush()

    def pop_recent_raw_lines(self) -> list[str]:
        lines = self.recent_raw_lines
        self.recent_raw_lines = []
        return lines

    def close(self) -> None:
        if self._sock is not None:
            try:
                self._sock.close()
            finally:
                self._sock = None

        if self._log_fp is not None:
            self._log_fp.close()
            self._log_fp = None

    def _write_log_line(self, raw_line: str) -> str:
        timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]
        raw_line = raw_line.strip()
        log_line = f"[{timestamp}] {raw_line}"

        # 新增：缓存原始 GNSS 数据，供窗口显示
        self.recent_raw_lines.append(raw_line)
        if len(self.recent_raw_lines) > self.max_recent_raw_lines:
            self.recent_raw_lines = self.recent_raw_lines[-self.max_recent_raw_lines:]

        if self._log_fp is not None:
            self._log_fp.write(log_line + "\n")
            self._log_fp.flush()

        return log_line

    def poll(self) -> GNSSFix | None:
        self.open()
        if self._sock is None:
            return None

        try:
            data = self._sock.recv(4096)
        except socket.timeout:
            return None

        if not data:
            self.close()
            raise ConnectionError("GNSS net connection closed by remote device")

        if self.config.print_raw_bytes:
            print(data)

        text = data.decode("utf-8", errors="ignore")
        text = text.replace("\r", "\n")
        self._text_buffer += text

        lines = self._text_buffer.split("\n")
        self._text_buffer = lines.pop()

        latest: GNSSFix | None = None
        for raw_line in lines:
            raw_line = raw_line.strip()
            if not raw_line:
                continue

            log_line = self._write_log_line(raw_line)
            item = extract_nmea_from_log_line(log_line)
            if item is None:
                continue

            ts, sentence = item
            try:
                latest = parse_gga_sentence(
                    sentence,
                    log_timestamp=ts,
                    validate_checksum=self.validate_checksum,
                )
            except ValueError:
                continue

        return latest

    def iter_forever(self, interval_s: float = 0.1) -> Iterator[GNSSFix]:
        try:
            while True:
                fix = self.poll()
                if fix is not None:
                    yield fix
                time.sleep(interval_s)
        finally:
            self.close()

    def run_forever(self, interval_s: float = 0.1) -> None:
        try:
            print("Press Ctrl+C to exit")
            for fix in self.iter_forever(interval_s=interval_s):
                if fix.lost:
                    print(f"{fix.timestamp:.3f} GNSS无效 sats={fix.satellites} hdop={fix.hdop}")
                else:
                    print(
                        f"{fix.timestamp:.3f} lat={fix.point.lat:.8f} lon={fix.point.lon:.8f} "
                        f"alt={fix.point.alt:.3f} status={fix.fix_status} "
                        f"sats={fix.satellites} hdop={fix.hdop}"
                    )
        except KeyboardInterrupt:
            print("\nDisconnecting...")
            if self.log_file is not None:
                print(f"Data saved to: {self.log_file}")
        finally:
            self.close()


def connect_device() -> None:
    """兼容原独立脚本入口：python -m rov_control.gnss_collect_net。"""
    from rov_control.config import ROVConfig

    cfg = ROVConfig()
    reader = GNSSNetReader(
        GNSSNetConfig(
            host=cfg.gnss_net_ip,
            port=cfg.gnss_net_port,
            connect_timeout_s=cfg.gnss_net_connect_timeout_s,
            recv_timeout_s=cfg.gnss_net_recv_timeout_s,
            log_dir=Path(cfg.gnss_net_log_dir),
            print_raw_bytes=cfg.gnss_net_print_raw_bytes,
        ),
        validate_checksum=cfg.gnss_validate_checksum,
    )
    reader.run_forever()


if __name__ == "__main__":
    connect_device()
