from __future__ import annotations

import argparse
import time
from pathlib import Path

from rov_control.config import ROVConfig, create_test_folder, save_config
from rov_control.geo import GeoPoint, LocalPoint, LocalTangentPlane
from rov_control.gnss import is_usable_for_navigation

from rov_control.imu_nav import SixAxisIMUNavigator
from rov_control.planner import WaypointPlanner
from rov_control.controller import HighLevelController
from rov_control.network import ROVSocketClient
from rov_control.logger import CSVLogger

from rov_control.gnss_file import GNSSSerialConfig, GNSSSerialReader, GNSSLogTailReader
from rov_control.gnss_collect_net import GNSSNetConfig, GNSSNetReader
from rov_control.ntrip_config import configure_ntrip_if_needed

from datetime import date


def build_demo_waypoints() -> list[LocalPoint]:
    # x 东、y 北、z 深度向下。当前控制深度主要使用 cfg.target_depth_m，航点 z 暂时用于记录和扩展。
    return [
        LocalPoint(0, 0, 0),
        LocalPoint(1, 0, 0),
        LocalPoint(2, 0, 0),
        LocalPoint(3, 0.6, 0),
        LocalPoint(3, 1.6, 0),
        LocalPoint(2, 2.4, 0),
        LocalPoint(1, 2.4, 0),
        LocalPoint(0, 1.6, 0),
        LocalPoint(0, 0.6, 0),
        LocalPoint(1, 0, 0),
        LocalPoint(2, 0.4, 0),
    ]


def default_sim_gnss_raw_path(base_dir: str | Path, cid: str) -> Path:
    """Return the raw.txt path written by rov_control.real_env_simulator."""
    return Path(base_dir) / cid / cid / date.today().strftime("%Y-%m-%d") / "raw.txt"


def make_runtime_config(args: argparse.Namespace) -> ROVConfig:
    """Create a runtime config from config.py defaults plus command-line overrides."""
    cfg = ROVConfig()

    cfg.rov_ip = args.ip
    cfg.rov_port = args.port
    cfg.max_speed_mps = args.speed
    cfg.gnss_validate_checksum = not args.no_gnss_checksum
    cfg.rov_heading_offset_deg = args.rov_heading_offset_deg

    cfg.gnss_net_ip = args.gnss_net_ip
    cfg.gnss_net_port = args.gnss_net_port
    cfg.gnss_net_connect_timeout_s = args.gnss_net_connect_timeout_s
    cfg.gnss_net_recv_timeout_s = args.gnss_net_recv_timeout_s
    cfg.gnss_net_log_dir = args.gnss_net_log_dir
    cfg.gnss_net_print_raw_bytes = args.gnss_print_raw_bytes

    cfg.gnss_port = args.gnss_port
    cfg.gnss_baudrate = args.gnss_baudrate
    cfg.gnss_timeout_s = args.gnss_timeout_s
    cfg.gnss_log_dir = args.gnss_log_dir
    cfg.gnss_print_raw_bytes = args.gnss_print_raw_bytes

    cfg.sim_gnss_base_dir = args.sim_gnss_base_dir
    cfg.sim_cid = args.sim_cid

    cfg.ntrip_config_enabled = cfg.ntrip_config_enabled and not args.no_ntrip_config
    cfg.ntrip_config_transport = args.ntrip_transport
    cfg.ntrip_net_ip = args.ntrip_net_ip
    cfg.ntrip_net_port = args.ntrip_net_port
    cfg.ntrip_net_connect_timeout_s = args.ntrip_net_connect_timeout_s
    cfg.ntrip_net_recv_timeout_s = args.ntrip_net_recv_timeout_s
    cfg.ntrip_serial_port = args.ntrip_port
    cfg.ntrip_serial_baudrate = args.ntrip_baudrate
    cfg.ntrip_serial_timeout_s = args.ntrip_timeout_s
    cfg.ntrip_state_file = args.ntrip_state_file

    cfg.dvl_enabled = not getattr(args, "no_dvl", False)
    cfg.dvl_ip = args.dvl_ip
    cfg.dvl_port = args.dvl_port
    cfg.dvl_connect_timeout_s = args.dvl_connect_timeout_s
    cfg.dvl_recv_timeout_s = args.dvl_recv_timeout_s
    cfg.dvl_stale_timeout_s = args.dvl_stale_timeout_s
    cfg.dvl_log_dir = args.dvl_log_dir
    cfg.dvl_print_raw = args.dvl_print_raw
    cfg.dvl_position_gain = args.dvl_position_gain
    cfg.dvl_velocity_gain = args.dvl_velocity_gain

    return cfg


def maybe_configure_ntrip(args: argparse.Namespace, cfg: ROVConfig, logger: CSVLogger) -> None:
    """Configure GNSS NTRIP once before real-device GNSS network reading."""
    if args.sim or args.no_gnss:
        return

    # NTRIP 账号写入 GNSS 模块本体，与后续 GNSS 数据是否走网口无冲突。
    # 只有实机运行时尝试，仿真 app --sim 不做串口配置。
    try:
        configure_ntrip_if_needed(
            cfg,
            force=args.force_ntrip_config,
            logger=logger,
        )
    except Exception as exc:
        # 现场如果模块已经配置过但状态文件丢失，不应阻塞 app 主流程；记录清楚便于排查。
        logger.log_event(time.time(), "WARN", "ntrip_config_failed", str(exc))
        print(f"NTRIP 一键配置失败，主程序继续启动：{exc}")


def create_gnss_reader(args: argparse.Namespace, cfg: ROVConfig, logger: CSVLogger):
    if args.no_gnss:
        return None

    if args.gnss_source == "file":
        raw_path = (
            Path(args.gnss_raw_path)
            if args.gnss_raw_path
            else default_sim_gnss_raw_path(cfg.sim_gnss_base_dir, cfg.sim_cid)
        )

        logger.log_event(
            time.time(),
            "INFO",
            "gnss_file_reader",
            f"GNSS raw file={raw_path}",
        )
        return GNSSLogTailReader(
            raw_path,
            validate_checksum=cfg.gnss_validate_checksum,
            read_from_beginning=False,
        )

    if args.gnss_source == "net":
        logger.log_event(
            time.time(),
            "INFO",
            "gnss_net_reader",
            f"GNSS net={cfg.gnss_net_ip}:{cfg.gnss_net_port}",
        )
        return GNSSNetReader(
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

    # 串口读取作为调试/兼容保留，实机 app 默认不再使用。
    gnss_config = GNSSSerialConfig(
        port=cfg.gnss_port,
        baudrate=cfg.gnss_baudrate,
        timeout_s=cfg.gnss_timeout_s,
        log_dir=Path(cfg.gnss_log_dir),
        print_raw_bytes=cfg.gnss_print_raw_bytes,
    )
    logger.log_event(
        time.time(),
        "INFO",
        "gnss_serial_reader",
        f"GNSS serial={cfg.gnss_port}, baudrate={cfg.gnss_baudrate}",
    )
    return GNSSSerialReader(
        gnss_config,
        validate_checksum=cfg.gnss_validate_checksum,
    )


def run_auto_surface_demo(args: argparse.Namespace) -> None:
    from rov_control.plotter import TrajectoryPlotter

    cfg = make_runtime_config(args)

    folder = create_test_folder(args.log_dir)
    save_config(cfg, folder)
    logger = CSVLogger(folder)

    maybe_configure_ntrip(args, cfg, logger)
    gnss_reader = create_gnss_reader(args, cfg, logger)

    planner = WaypointPlanner(
        max_speed_mps=cfg.max_speed_mps,
        arrival_radius_m=cfg.arrival_radius_m,
    )

    waypoints = build_demo_waypoints()
    planner.set_path(waypoints)
    logger.log_waypoints(waypoints)

    nav = SixAxisIMUNavigator(cfg)
    nav.initialize(cfg.manual_initial_yaw_deg, waypoints[0], time.time())

    controller = HighLevelController(cfg)
    controller.arm()

    plotter = TrajectoryPlotter(planned=waypoints)

    client = ROVSocketClient(cfg.rov_ip, cfg.rov_port)
    client.connect()

    try:
        while True:
            frames = client.poll_sensors()
            for imu in frames:
                state = nav.update_with_imu(
                    imu,
                    commanded_speed_mps=cfg.max_speed_mps,
                )
                logger.log_imu(imu)
                logger.log_trajectory(
                    "trajectory_raw.csv",
                    imu.timestamp,
                    state.local_point(),
                    state.source,
                )
                plotter.add_imu(state.local_point())

            if gnss_reader is not None:
                fix = gnss_reader.poll()
                if fix is not None:
                    if is_usable_for_navigation(fix, require_fixed=False):
                        origin_was_empty = nav.local_frame is None

                        corrected = nav.correct_with_gnss(
                            fix,
                            depth_m=nav.state.z,
                            require_fixed_origin=False,
                        )
                        local_pos = nav.gnss_to_local(
                            fix,
                            depth_m=corrected.z,
                        )

                        if origin_was_empty and nav.local_frame is not None:
                            logger.log_event(
                                fix.timestamp,
                                "INFO",
                                "set_origin",
                                "使用第一条有效GGA作为局部坐标原点",
                            )

                        logger.log_gnss(fix, local_pos)

                        if local_pos is not None:
                            logger.log_trajectory(
                                "trajectory_corrected.csv",
                                fix.timestamp,
                                corrected.local_point(),
                                corrected.source,
                            )
                            plotter.add_rtk(local_pos)
                            plotter.add_corrected(corrected.local_point())
                    else:
                        logger.log_gnss(fix, None)
                        logger.log_event(
                            fix.timestamp,
                            "WARN",
                            "gnss_lost",
                            f"GGA无效/无定位: sats={fix.satellites}, hdop={fix.hdop}",
                        )

            status = planner.update(nav.state.local_point())
            planner.append_history(nav.state.local_point())

            out = controller.make_command(
                nav.state,
                status,
                target_depth_m=cfg.target_depth_m,
                heading_hold=not getattr(args, "no_heading_hold", False),
                depth_hold=not getattr(args, "no_depth_hold", False),
            )
            client.send_channel(out.command)
            logger.log_control(out)

            plotter.add_heading(
                nav.state.timestamp if nav.state.timestamp else time.time(),
                status.target_heading_deg,
                nav.state.raw_yaw_deg,
                nav.state.yaw_deg,
                nav.state.gnss_course_deg,
            )
            plotter.show_once(nav.state.local_point(), nav.state.yaw_deg)
            plotter.show_heading_once()

            if status.arrived:
                client.send_channel(controller.emergency_stop())
                logger.log_event(time.time(), "INFO", "arrived", "到达终点，已停止")
                break

            time.sleep(cfg.command_period_s)
    finally:
        client.close()
        if gnss_reader is not None:
            gnss_reader.close()
        logger.close()
        plotter.save_png(str(folder / "trajectory.png"))
        print(f"日志已保存：{folder}")


def convert_geo_example() -> None:
    origin = GeoPoint(lat=34.000000, lon=108.000000, alt=0.0)
    ltp = LocalTangentPlane(origin)
    target_geo = GeoPoint(lat=34.000045, lon=108.000108, alt=0.0)
    local = ltp.geo_to_local(target_geo)
    print("经纬度转局部坐标：", local)
    print("局部坐标转经纬度：", ltp.local_to_geo(local))


def run_gnss_serial_test(args: argparse.Namespace) -> None:
    reader = GNSSSerialReader(
        GNSSSerialConfig(
            port=args.gnss_port,
            baudrate=args.gnss_baudrate,
            timeout_s=args.gnss_timeout_s,
            log_dir=Path(args.gnss_log_dir),
            print_raw_bytes=args.gnss_print_raw_bytes,
        ),
        validate_checksum=not args.no_gnss_checksum,
    )

    print(f"正在读取 GNSS 串口：{args.gnss_port}, baudrate={args.gnss_baudrate}")
    try:
        while True:
            fix = reader.poll()
            if fix is not None:
                if fix.lost:
                    print(f"{fix.timestamp:.3f} GNSS无效 sats={fix.satellites} hdop={fix.hdop}")
                else:
                    print(
                        f"{fix.timestamp:.3f} lat={fix.point.lat:.8f} lon={fix.point.lon:.8f} "
                        f"alt={fix.point.alt:.3f} status={fix.fix_status} "
                        f"sats={fix.satellites} hdop={fix.hdop}"
                    )
            time.sleep(0.1)
    finally:
        reader.close()


def run_gnss_net_test(args: argparse.Namespace) -> None:
    reader = GNSSNetReader(
        GNSSNetConfig(
            host=args.gnss_net_ip,
            port=args.gnss_net_port,
            connect_timeout_s=args.gnss_net_connect_timeout_s,
            recv_timeout_s=args.gnss_net_recv_timeout_s,
            log_dir=Path(args.gnss_net_log_dir),
            print_raw_bytes=args.gnss_print_raw_bytes,
        ),
        validate_checksum=not args.no_gnss_checksum,
    )

    print(f"正在读取 GNSS 网口：{args.gnss_net_ip}:{args.gnss_net_port}")
    try:
        while True:
            fix = reader.poll()
            if fix is not None:
                if fix.lost:
                    print(f"{fix.timestamp:.3f} GNSS无效 sats={fix.satellites} hdop={fix.hdop}")
                else:
                    print(
                        f"{fix.timestamp:.3f} lat={fix.point.lat:.8f} lon={fix.point.lon:.8f} "
                        f"alt={fix.point.alt:.3f} status={fix.fix_status} "
                        f"sats={fix.satellites} hdop={fix.hdop}"
                    )
            time.sleep(0.1)
    finally:
        reader.close()


def main() -> None:
    default_cfg = ROVConfig()
    parser = argparse.ArgumentParser(description="虎鲛 ROV RTK/IMU 自动航行测试程序")

    parser.add_argument(
        "--mode",
        default="auto_surface",
        choices=["convert", "auto_surface", "gnss_serial_test", "gnss_net_test"],
    )

    # 运行模式
    parser.add_argument(
        "--sim",
        action="store_true",
        help="启用软件在环仿真：默认连接 127.0.0.1，并从仿真 raw.txt 读取 GNSS",
    )

    # ROV 通信参数
    parser.add_argument(
        "--ip",
        default=None,
        help="ROV TCP IP；实机默认读取 config.ROVConfig.rov_ip，--sim 默认读取 config.ROVConfig.sim_rov_ip",
    )
    parser.add_argument("--port", type=int, default=default_cfg.rov_port)
    parser.add_argument("--speed", type=float, default=default_cfg.max_speed_mps, choices=[0.2, 0.5, 1.0])
    parser.add_argument("--log-dir", default="logs")

    # GNSS 参数
    parser.add_argument(
        "--gnss-source",
        default=None,
        choices=["net", "serial", "file"],
        help="GNSS来源：net=真实网口，serial=串口调试，file=仿真raw.txt；默认：实机 net，--sim file",
    )
    parser.add_argument(
        "--gnss-raw-path",
        default="",
        help="仿真 GNSS raw.txt 路径；不填时根据 --sim-gnss-base-dir 和 --sim-cid 自动生成",
    )
    parser.add_argument(
        "--sim-gnss-base-dir",
        default=default_cfg.sim_gnss_base_dir,
        help="仿真器 GNSS 日志根目录",
    )
    parser.add_argument(
        "--sim-cid",
        default=default_cfg.sim_cid,
        help="仿真器 CID，需与仿真器一致",
    )

    # GNSS 网口参数，实机 app 默认使用。
    parser.add_argument("--gnss-net-ip", default=default_cfg.gnss_net_ip, help="GNSS 网口 IP")
    parser.add_argument("--gnss-net-port", type=int, default=default_cfg.gnss_net_port, help="GNSS 网口端口")
    parser.add_argument("--gnss-net-connect-timeout-s", type=float, default=default_cfg.gnss_net_connect_timeout_s)
    parser.add_argument("--gnss-net-recv-timeout-s", type=float, default=default_cfg.gnss_net_recv_timeout_s)
    parser.add_argument("--gnss-net-log-dir", default=default_cfg.gnss_net_log_dir, help="GNSS 网口原始数据日志目录")

    # GNSS 串口参数：保留给串口调试和 NTRIP 配置。
    parser.add_argument(
        "--gnss-port",
        default=default_cfg.gnss_port,
        help="GNSS串口号，例如 Windows 下 COM4，Linux 下 /dev/ttyUSB0",
    )
    parser.add_argument("--gnss-baudrate", type=int, default=default_cfg.gnss_baudrate, help="GNSS串口波特率")
    parser.add_argument("--gnss-timeout-s", type=float, default=default_cfg.gnss_timeout_s, help="GNSS串口读取超时时间")
    parser.add_argument("--gnss-log-dir", default=default_cfg.gnss_log_dir, help="GNSS原始串口数据日志目录")
    parser.add_argument("--gnss-print-raw-bytes", action="store_true", help="调试时打印 GNSS 原始字节")
    parser.add_argument("--no-gnss", action="store_true", help="不启用GNSS读取，仅使用IMU/仿真数据")
    parser.add_argument("--no-gnss-checksum", action="store_true", help="跳过NMEA校验和，适合调试不完整/混杂输出")

    # DVL-A50 参数：默认启用，失效/无数据时自动退回 GNSS/IMU，并在窗口显示状态。
    parser.add_argument("--no-dvl", action="store_true", help="关闭 DVL 融合；默认启用。--sim 下默认连接本机模拟 DVL，实机默认连接真实 DVL")
    parser.add_argument("--sim-dvl", action="store_true", help="兼容旧参数；现在 app --sim 默认就会尝试连接本机模拟 DVL")
    parser.add_argument("--dvl-ip", default=None, help="DVL-A50 TCP IP；实机默认真实 DVL，--sim 默认 127.0.0.1")
    parser.add_argument("--dvl-port", type=int, default=None, help="DVL-A50 TCP 端口；实机默认真实 DVL 端口，--sim 默认模拟 DVL 端口")
    parser.add_argument("--dvl-connect-timeout-s", type=float, default=default_cfg.dvl_connect_timeout_s, help="DVL TCP 连接超时")
    parser.add_argument("--dvl-recv-timeout-s", type=float, default=default_cfg.dvl_recv_timeout_s, help="DVL TCP 非阻塞读取超时")
    parser.add_argument("--dvl-stale-timeout-s", type=float, default=default_cfg.dvl_stale_timeout_s, help="超过该秒数无新 DVL 数据即显示无新数据")
    parser.add_argument("--dvl-log-dir", default=default_cfg.dvl_log_dir, help="DVL 原始 JSON 日志目录")
    parser.add_argument("--dvl-print-raw", action="store_true", help="调试时在终端打印 DVL 原始 JSON")
    parser.add_argument("--dvl-position-gain", type=float, default=default_cfg.dvl_position_gain, help="DVL 位置积分融合增益 0~1")
    parser.add_argument("--dvl-velocity-gain", type=float, default=default_cfg.dvl_velocity_gain, help="DVL 速度融合增益 0~1")
    parser.add_argument("--map-scale", type=float, default=default_cfg.map_scale_m_per_unit, help="鼠标规划地图比例：1 图上单位对应多少米，默认 1")
    parser.add_argument("--rov-heading-offset-deg", type=float, default=default_cfg.rov_heading_offset_deg, help="ROV艏向修正角：尾部被当头/航向反180°时用180；方向正常用0")

    # NTRIP 一键配置参数：默认走网口，不再依赖 COM4；串口参数仅兼容调试。
    parser.add_argument("--no-ntrip-config", action="store_true", help="实机启动时跳过 NTRIP 一键配置")
    parser.add_argument("--force-ntrip-config", action="store_true", help="忽略状态文件，强制重新下发 NTRIP AT 命令")
    parser.add_argument("--ntrip-transport", default=default_cfg.ntrip_config_transport, choices=["net", "serial"], help="NTRIP 配置通道：默认 net 网口；serial 为兼容串口调试")
    parser.add_argument("--ntrip-net-ip", default=default_cfg.ntrip_net_ip, help="NTRIP 配置网口 IP")
    parser.add_argument("--ntrip-net-port", type=int, default=default_cfg.ntrip_net_port, help="NTRIP 配置网口端口")
    parser.add_argument("--ntrip-net-connect-timeout-s", type=float, default=default_cfg.ntrip_net_connect_timeout_s, help="NTRIP 配置网口连接超时")
    parser.add_argument("--ntrip-net-recv-timeout-s", type=float, default=default_cfg.ntrip_net_recv_timeout_s, help="NTRIP 配置网口读取响应超时")
    parser.add_argument("--ntrip-port", default=default_cfg.ntrip_serial_port, help="NTRIP 配置串口，仅 --ntrip-transport serial 时使用")
    parser.add_argument("--ntrip-baudrate", type=int, default=default_cfg.ntrip_serial_baudrate, help="NTRIP 配置串口波特率，仅串口模式使用")
    parser.add_argument("--ntrip-timeout-s", type=float, default=default_cfg.ntrip_serial_timeout_s, help="NTRIP 配置串口超时，仅串口模式使用")
    parser.add_argument("--ntrip-state-file", default=default_cfg.ntrip_state_file, help="NTRIP 已配置状态文件")

    # 交互 App 参数：默认启用 Tkinter 界面；如需旧版纯循环/Matplotlib 轨迹窗口，使用 --no-ui。
    parser.add_argument("--no-ui", action="store_true", help="关闭交互控制面板，使用旧版自动巡航循环")
    parser.add_argument("--start-paused", action="store_true", help="交互 App 启动后不立即自动巡航，等待鼠标点击开启")
    parser.add_argument("--no-heading-hold", action="store_true", help="启动时关闭定向/航向保持")
    parser.add_argument("--no-depth-hold", action="store_true", help="启动时关闭定深保持")
    parser.add_argument("--plot-update-s", type=float, default=0.25, help="交互 App 中实时轨迹/航向图刷新周期，单位秒，默认 0.25；GPU 图可更流畅，调大可减少卡顿")
    parser.add_argument("--display-update-s", type=float, default=0.25, help="交互 App 状态文字/PWM 刷新周期，单位秒，默认 0.25；控制指令频率不受影响")
    parser.add_argument("--max-live-plot-points", type=int, default=300, help="交互 App 每条轨迹最多绘制的点数，CSV 日志不受影响，默认 300")
    parser.add_argument(
        "--plot-backend",
        default="auto",
        choices=["auto", "gpu", "opengl", "pyqtgraph", "matplotlib"],
        help="实时绘图后端：auto 默认优先 GPU/OpenGL(pyqtgraph)，不可用则回退 Matplotlib；gpu/opengl/pyqtgraph 强制显卡绘图；matplotlib 使用旧内嵌图",
    )
    parser.add_argument("--no-live-plots", action="store_true", help="关闭交互 App 内嵌实时轨迹/航向图，仅记录 CSV 并退出时保存 PNG")

    args = parser.parse_args()

    # 运行方式保持分离：
    #   实际运行：python -m rov_control.app
    #   仿真运行：python -m rov_control.app --sim
    if args.sim:
        # 模拟调试：app 连接本机 sim_env_simulator/real_env_simulator，并从仿真 raw.txt 读取 GNSS。
        # DVL 默认也尝试连接本机模拟端；若模拟器未加 --dvl，窗口会显示未连接/无 DVL 数据。
        args.ip = args.ip or default_cfg.sim_rov_ip
        args.gnss_source = args.gnss_source or default_cfg.sim_gnss_source
        args.dvl_ip = args.dvl_ip or default_cfg.sim_dvl_ip
        args.dvl_port = args.dvl_port or default_cfg.sim_dvl_port
    else:
        # 现实调试：app 默认连接真实 ROV、真实 GNSS 网口，并启用真实 DVL。
        args.ip = args.ip or default_cfg.rov_ip
        args.gnss_source = args.gnss_source or default_cfg.real_gnss_source
        args.dvl_ip = args.dvl_ip or default_cfg.dvl_ip
        args.dvl_port = args.dvl_port or default_cfg.dvl_port

    if args.mode == "convert":
        convert_geo_example()
    elif args.mode == "auto_surface":
        if args.no_ui:
            run_auto_surface_demo(args)
        else:
            from rov_control.interactive_app import run_interactive_auto_surface

            run_interactive_auto_surface(args)
    elif args.mode == "gnss_serial_test":
        run_gnss_serial_test(args)
    elif args.mode == "gnss_net_test":
        run_gnss_net_test(args)


if __name__ == "__main__":
    main()
