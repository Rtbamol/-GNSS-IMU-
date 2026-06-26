from __future__ import annotations

import re
import time
from dataclasses import dataclass, field
from datetime import datetime
from pathlib import Path
from typing import Iterator

try:
    import serial
except ImportError:  # pyserial is only required when --gnss-source serial is used.
    serial = None

from rov_control.geo import GeoPoint
from rov_control.gnss import GNSSFix


LOG_LINE_RE = re.compile(
    r"^\[(?P<ts>\d{4}-\d{2}-\d{2} \d{2}:\d{2}:\d{2}\.\d{1,6})\]\s*(?P<body>.*)$"
)

GGA_FIX_STATUS = {
    "0": "none",
    "1": "single",
    "2": "dgps_float",
    "4": "fixed",
    "5": "float",
    "6": "estimated",
}


@dataclass
class GNSSSerialConfig:
    """GNSS 串口采集配置。"""

    port: str = "COM4"
    baudrate: int = 115200
    timeout_s: float = 0.1

    log_dir: Path = field(
        default_factory=lambda: Path(
            r"D:\A_A\A_Schedule\date_set\Scarab\gnss_python\gnss_logs"
        )
    )
    log_prefix: str = "gnss_data"
    print_raw_bytes: bool = True

    def make_log_file(self) -> Path:
        self.log_dir.mkdir(parents=True, exist_ok=True)
        start_time = datetime.now().strftime("%Y%m%d_%H%M%S")
        return self.log_dir / f"{self.log_prefix}_{start_time}.txt"


def nmea_checksum_ok(sentence: str) -> bool:
    """校验 NMEA checksum。没有 * 时返回 True。"""
    sentence = sentence.strip()

    if not sentence.startswith("$") or "*" not in sentence:
        return True

    data, checksum = sentence[1:].split("*", 1)
    checksum = checksum[:2]

    value = 0
    for ch in data:
        value ^= ord(ch)

    try:
        return value == int(checksum, 16)
    except ValueError:
        return False


def dm_to_decimal(value: str, hemisphere: str) -> float:
    """NMEA ddmm.mmmm / dddmm.mmmm 转十进制度。"""
    if not value:
        raise ValueError("empty lat/lon")

    raw = float(value)
    degrees = int(raw // 100)
    minutes = raw - degrees * 100

    decimal = degrees + minutes / 60.0

    if hemisphere.upper() in {"S", "W"}:
        decimal = -decimal

    return decimal


def extract_nmea_from_log_line(line: str) -> tuple[float, str] | None:
    """从 '[2026-06-18 10:20:30.123] $GNGGA,...' 中提取时间戳和 GGA 语句。"""

    m = LOG_LINE_RE.match(line.strip())
    if not m:
        return None

    body = m.group("body")

    candidates = []
    for marker in ("$GNGGA", "$GPGGA"):
        idx = body.find(marker)
        if idx >= 0:
            candidates.append(idx)

    if not candidates:
        return None

    idx = min(candidates)
    sentence = body[idx:].strip()

    # 如果一行里有多个 NMEA 语句，只取第一条 GGA。
    next_dollar = sentence.find("$", 1)
    if next_dollar >= 0:
        sentence = sentence[:next_dollar].strip()

    # 如果有 checksum，只截取到 * 后两位。
    star = sentence.find("*")
    if star >= 0 and len(sentence) >= star + 3:
        sentence = sentence[: star + 3]

    ts = datetime.strptime(
        m.group("ts"),
        "%Y-%m-%d %H:%M:%S.%f",
    ).timestamp()

    return ts, sentence


def parse_gga_sentence(
    sentence: str,
    log_timestamp: float | None = None,
    validate_checksum: bool = True,
) -> GNSSFix:
    """解析 GNGGA / GPGGA。

    GGA 字段：
    $GNGGA,time,lat,N,lon,E,quality,numSV,HDOP,alt,M,geoidSep,M,diffAge,stationID*CS
    """

    sentence = sentence.strip()

    if validate_checksum and not nmea_checksum_ok(sentence):
        raise ValueError(f"NMEA checksum failed: {sentence}")

    if not (sentence.startswith("$GNGGA") or sentence.startswith("$GPGGA")):
        raise ValueError("not GGA")

    body = sentence[1:].split("*", 1)[0]
    p = body.split(",")

    if len(p) < 10:
        raise ValueError("GGA fields not enough")

    quality = p[6] or "0"
    satellites = int(p[7]) if len(p) > 7 and p[7].isdigit() else 0
    hdop = float(p[8]) if len(p) > 8 and p[8] else 99.0
    diff_age_s = float(p[13]) if len(p) > 13 and p[13] else 999.0

    fix_status = GGA_FIX_STATUS.get(quality, quality)
    timestamp = log_timestamp if log_timestamp is not None else time.time()

    if quality == "0" or not p[2] or not p[4]:
        return GNSSFix(
            timestamp=timestamp,
            point=GeoPoint(lat=0.0, lon=0.0, alt=0.0),
            fix_status="none",
            satellites=satellites,
            hdop=hdop,
            diff_age_s=diff_age_s,
            lost=True,
        )

    lat = dm_to_decimal(p[2], p[3])
    lon = dm_to_decimal(p[4], p[5])
    alt = float(p[9]) if p[9] else 0.0

    return GNSSFix(
        timestamp=timestamp,
        point=GeoPoint(lat=lat, lon=lon, alt=alt),
        fix_status=fix_status,
        satellites=satellites,
        hdop=hdop,
        diff_age_s=diff_age_s,
        lost=False,
    )


class GNSSSerialReader:
    """从串口实时采集 GNSS 数据，同时写入日志文件，并解析 GGA。

    poll() 非阻塞：
    - 有新 GGA 数据时，返回最新一条 GNSSFix；
    - 没有新数据时，返回 None。
    """

    def __init__(
        self,
        config: GNSSSerialConfig,
        validate_checksum: bool = True,
    ):
        self.config = config
        self.validate_checksum = validate_checksum

        self.log_file: Path | None = None
        self._ser: serial.Serial | None = None
        self._log_fp = None
        self._text_buffer = ""

    def open(self) -> None:
        if self._ser is not None and self._ser.is_open:
            return

        print(f"Trying to connect to {self.config.port} at {self.config.baudrate} baud...")

        self.log_file = self.config.make_log_file()

        if serial is None:
            raise RuntimeError("未安装 pyserial，无法读取 GNSS 串口；请运行 pip install pyserial，或在仿真中使用 --gnss-source file")

        self._ser = serial.Serial(
            port=self.config.port,
            baudrate=self.config.baudrate,
            timeout=self.config.timeout_s,
        )

        self._log_fp = open(self.log_file, "a", encoding="utf-8")

        print(f"Successfully connected to {self.config.port}")
        print(f"Data will be saved to: {self.log_file}")

        self._log_fp.write(f"Start time: {datetime.now()}\n")
        self._log_fp.write(f"Port: {self.config.port}\n")
        self._log_fp.write(f"Baudrate: {self.config.baudrate}\n")
        self._log_fp.write("=" * 80 + "\n")
        self._log_fp.flush()

    def close(self) -> None:
        if self._ser is not None:
            self._ser.close()

        if self._log_fp is not None:
            self._log_fp.close()

        self._ser = None
        self._log_fp = None

    def _write_log_line(self, raw_line: str) -> str:
        timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]
        log_line = f"[{timestamp}] {raw_line.strip()}"

        if self._log_fp is not None:
            self._log_fp.write(log_line + "\n")
            self._log_fp.flush()

        return log_line

    def poll(self) -> GNSSFix | None:
        self.open()

        if self._ser is None:
            return None

        waiting = self._ser.in_waiting
        if waiting <= 0:
            return None

        data = self._ser.read(waiting)

        if self.config.print_raw_bytes:
            print(data)

        text = data.decode("utf-8", errors="ignore")

        # 统一换行符，避免 \r\n、\r、\n 混用。
        text = text.replace("\r", "\n")
        self._text_buffer += text

        lines = self._text_buffer.split("\n")

        # 最后一段可能是不完整 NMEA，留到下一次 poll 再处理。
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
                print(fix)

        except PermissionError as e:
            print(f"Permission denied when opening {self.config.port}")
            print("原因通常是 COM 口被其他程序占用。")
            print(f"Original error: {e}")

        except serial.SerialException as e:
            print(f"Serial connection failed: {e}")

        except KeyboardInterrupt:
            print("\nDisconnecting...")
            if self.log_file is not None:
                print(f"Data saved to: {self.log_file}")

        finally:
            self.close()


def connect_device() -> None:
    """兼容原（3）的入口函数。"""
    config = GNSSSerialConfig(
        port="COM4",
        baudrate=115200,
        log_dir=Path(r"D:\A_A\A_Schedule\date_set\Scarab\gnss_python\gnss_logs"),
    )

    reader = GNSSSerialReader(config)
    reader.run_forever()

class GNSSLogTailReader:
    """Tail a simulated GNSS raw.txt file and parse GGA lines."""

    def __init__(self, raw_path: Path, validate_checksum: bool = True, read_from_beginning: bool = False):
        self.raw_path = Path(raw_path)
        self.validate_checksum = validate_checksum
        self.read_from_beginning = read_from_beginning
        self._fp = None
        self._buf = ""

        self.recent_raw_lines: list[str] = []
        self.max_recent_raw_lines = 80
    def pop_recent_raw_lines(self) -> list[str]:
        lines = self.recent_raw_lines
        self.recent_raw_lines = []
        return lines
    def open(self) -> None:
        if self._fp is not None:
            return
        if not self.raw_path.exists():
            return
        self._fp = open(self.raw_path, "r", encoding="utf-8")
        if not self.read_from_beginning:
            self._fp.seek(0, 2)

    def close(self) -> None:
        if self._fp is not None:
            self._fp.close()
            self._fp = None

    def poll(self):
        self.open()
        if self._fp is None:
            return None

        data = self._fp.read()
        if not data:
            return None

        self._buf += data
        lines = self._buf.splitlines(keepends=False)

        if self._buf and not self._buf.endswith(("\n", "\r")):
            self._buf = lines.pop() if lines else self._buf
        else:
            self._buf = ""

        latest = None

        for line in lines:
            item = extract_nmea_from_log_line(line)
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
        for line in lines:
            raw_line = line.strip()
            if raw_line:
                self.recent_raw_lines.append(raw_line)
                if len(self.recent_raw_lines) > self.max_recent_raw_lines:
                    self.recent_raw_lines = self.recent_raw_lines[-self.max_recent_raw_lines:]

            item = extract_nmea_from_log_line(line)
            if item is None:
                continue

        return latest


if __name__ == "__main__":
    connect_device()