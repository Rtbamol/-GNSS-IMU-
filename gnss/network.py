from __future__ import annotations
import socket
import time
from typing import Callable
from rov_control.protocol import ChannelCommand, SensorFrame, extract_sensor_frames, ProtocolError


class ROVSocketClient:
    """ROV TCP 客户端。ROV 为服务端，默认 192.168.1.7:8887。"""

    def __init__(self, host: str, port: int, timeout_s: float = 2.0, recv_timeout_s: float = 0.005):
        self.host = host
        self.port = port
        self.timeout_s = timeout_s
        # Tkinter 界面与控制循环共用主线程时，recv 超时过大会直接造成窗口卡顿。
        # 这里只把 TCP 读取做成近似非阻塞；没数据就立即返回，控制指令发送频率不变。
        self.recv_timeout_s = recv_timeout_s
        self.sock: socket.socket | None = None
        self.rx_buffer = bytearray()
        self.last_rx_time = 0.0

    def connect(self) -> None:
        self.close()
        self.sock = socket.create_connection((self.host, self.port), timeout=self.timeout_s)
        self.sock.settimeout(self.recv_timeout_s)

    def close(self) -> None:
        if self.sock is not None:
            try:
                self.sock.close()
            finally:
                self.sock = None

    def send_channel(self, cmd: ChannelCommand) -> None:
        if self.sock is None:
            raise RuntimeError("socket 未连接")
        self.sock.sendall(cmd.to_bytes())

    def poll_sensors(self) -> list[SensorFrame]:
        if self.sock is None:
            raise RuntimeError("socket 未连接")
        try:
            chunk = self.sock.recv(4096)
            if chunk:
                self.rx_buffer.extend(chunk)
                self.last_rx_time = time.time()
        except socket.timeout:
            pass
        frames: list[SensorFrame] = []
        for raw in extract_sensor_frames(self.rx_buffer):
            try:
                frames.append(SensorFrame.from_bytes(raw, timestamp=time.time()))
            except ProtocolError:
                continue
        return frames

    def run_receive_loop(self, on_frame: Callable[[SensorFrame], None], stop_flag: Callable[[], bool]) -> None:
        while not stop_flag():
            for frame in self.poll_sensors():
                on_frame(frame)
            time.sleep(0.01)
