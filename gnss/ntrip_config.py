from __future__ import annotations

import json
import socket
import time
from pathlib import Path


def _log(logger, level: str, event: str, message: str) -> None:
    if logger is not None:
        try:
            logger.log_event(time.time(), level, event, message)
        except Exception:
            pass


def _state_is_done(path: str | Path) -> bool:
    p = Path(path)
    if not p.exists():
        return False
    try:
        data = json.loads(p.read_text(encoding="utf-8"))
        return bool(data.get("configured"))
    except Exception:
        return False


def _write_state(path: str | Path, transport: str, commands: tuple[str, ...]) -> None:
    p = Path(path)
    p.parent.mkdir(parents=True, exist_ok=True) if p.parent != Path('.') else None
    p.write_text(
        json.dumps(
            {
                "configured": True,
                "transport": transport,
                "commands_count": len(commands),
                "timestamp": time.time(),
            },
            ensure_ascii=False,
            indent=2,
        ),
        encoding="utf-8",
    )


def _send_net(cfg, commands: tuple[str, ...], logger=None) -> None:
    with socket.create_connection(
        (cfg.ntrip_net_ip, int(cfg.ntrip_net_port)),
        timeout=float(cfg.ntrip_net_connect_timeout_s),
    ) as sock:
        sock.settimeout(float(cfg.ntrip_net_recv_timeout_s))
        for cmd in commands:
            payload = (cmd.strip() + "\r\n").encode("ascii", errors="ignore")
            sock.sendall(payload)
            _log(logger, "INFO", "ntrip_config_cmd", cmd)
            try:
                _ = sock.recv(1024)
            except socket.timeout:
                pass
            time.sleep(float(getattr(cfg, "ntrip_command_interval_s", 0.5)))


def _send_serial(cfg, commands: tuple[str, ...], logger=None) -> None:
    try:
        import serial  # type: ignore
    except Exception as exc:
        raise RuntimeError("未安装 pyserial，不能使用串口方式配置 NTRIP；请安装 pyserial 或改用 --ntrip-transport net") from exc

    with serial.Serial(
        port=cfg.ntrip_serial_port,
        baudrate=int(cfg.ntrip_serial_baudrate),
        timeout=float(cfg.ntrip_serial_timeout_s),
    ) as ser:
        for cmd in commands:
            payload = (cmd.strip() + "\r\n").encode("ascii", errors="ignore")
            ser.write(payload)
            ser.flush()
            _log(logger, "INFO", "ntrip_config_cmd", cmd)
            time.sleep(float(getattr(cfg, "ntrip_command_interval_s", 0.5)))
            try:
                _ = ser.read_all()
            except Exception:
                pass


def configure_ntrip_if_needed(cfg, force: bool = False, logger=None) -> bool:
    """Configure GNSS NTRIP once.

    Returns True when commands were sent in this run, False when skipped.
    This function is intentionally non-invasive: app.py catches exceptions so a
    missing COM port or unreachable GNSS module will not prevent the UI from starting.
    """
    if not getattr(cfg, "ntrip_config_enabled", False):
        _log(logger, "INFO", "ntrip_config_skip", "NTRIP 配置已关闭")
        return False

    state_file = getattr(cfg, "ntrip_state_file", "gnss_ntrip_configured.json")
    if not force and _state_is_done(state_file):
        _log(logger, "INFO", "ntrip_config_skip", f"已配置过，状态文件={state_file}")
        return False

    commands = tuple(getattr(cfg, "ntrip_commands", ()) or ())
    if not commands:
        _log(logger, "WARN", "ntrip_config_skip", "没有 NTRIP AT 命令")
        return False

    transport = str(getattr(cfg, "ntrip_config_transport", "net")).lower()
    _log(logger, "INFO", "ntrip_config_start", f"开始通过 {transport} 配置 NTRIP，共 {len(commands)} 条命令")

    if transport == "net":
        _send_net(cfg, commands, logger=logger)
    elif transport == "serial":
        _send_serial(cfg, commands, logger=logger)
    else:
        raise ValueError(f"未知 NTRIP 配置通道：{transport}")

    _write_state(state_file, transport, commands)
    _log(logger, "INFO", "ntrip_config_done", f"NTRIP 配置完成，状态文件={state_file}")
    return True
