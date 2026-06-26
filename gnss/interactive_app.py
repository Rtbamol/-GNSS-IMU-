from __future__ import annotations

import math
import time
import tkinter as tk
from dataclasses import dataclass
from datetime import date
from pathlib import Path
from tkinter import messagebox, ttk

from rov_control.config import ROVConfig, create_test_folder, save_config
from rov_control.controller import ControlOutput, HighLevelController
from rov_control.geo import LocalPoint
from rov_control.gnss import is_usable_for_navigation
from rov_control.gnss_file import GNSSLogTailReader, GNSSSerialConfig, GNSSSerialReader
from rov_control.gnss_collect_net import GNSSNetConfig, GNSSNetReader
from rov_control.dvl import DVLConfig, DVLTCPReader
from rov_control.ntrip_config import configure_ntrip_if_needed
from rov_control.imu_nav import SixAxisIMUNavigator
from rov_control.logger import CSVLogger
from rov_control.network import ROVSocketClient
from rov_control.planner import PathStatus, WaypointPlanner
from rov_control.protocol import ChannelCommand, clamp_int
from rov_control.thruster_mixer import ThrusterLayout, preview_thruster_pwms
from rov_control.plotter import TrajectoryPlotter
from rov_control.gpu_plotter import GpuRealtimePlotter

# Matplotlib 仅在需要回退到内嵌绘图时再导入。
# 避免 --plot-backend gpu 启动时，因为 NumPy 2.x / 旧 matplotlib ABI 不兼容而直接崩溃。
FigureCanvasTkAgg = None
Figure = None

def _lazy_import_matplotlib_tk():
    global FigureCanvasTkAgg, Figure
    if FigureCanvasTkAgg is not None and Figure is not None:
        return True
    try:
        from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg as _FigureCanvasTkAgg
        from matplotlib.figure import Figure as _Figure
        FigureCanvasTkAgg = _FigureCanvasTkAgg
        Figure = _Figure
        return True
    except Exception:
        FigureCanvasTkAgg = None
        Figure = None
        return False


@dataclass
class ManualAxes:
    forward: float = 0.0
    yaw: float = 0.0
    lateral: float = 0.0
    vertical: float = 0.0

    def active(self, deadband: float = 0.05) -> bool:
        return any(abs(v) >= deadband for v in (self.forward, self.yaw, self.lateral, self.vertical))


def build_demo_waypoints() -> list[LocalPoint]:
    return [
        LocalPoint(0, 0, 0),
        LocalPoint(5, 0, 0),
        LocalPoint(10, 0, 0),
        LocalPoint(15, 3, 0),
        LocalPoint(15, 8, 0),
        LocalPoint(10, 12, 0),
        LocalPoint(5, 12, 0),
        LocalPoint(0, 8, 0),
        LocalPoint(0, 3, 0),
        LocalPoint(5, 0, 0),
        LocalPoint(10, 2, 0),
    ]


def default_sim_gnss_raw_path(base_dir: str | Path, cid: str) -> Path:
    return Path(base_dir) / cid / cid / date.today().strftime("%Y-%m-%d") / "raw.txt"


def _axis_to_pwm(axis: float, cfg: ROVConfig) -> int:
    axis = max(-1.0, min(1.0, axis))
    span = max(cfg.pwm_max - cfg.pwm_mid, cfg.pwm_mid - cfg.pwm_min, 1)
    return clamp_int(round(cfg.pwm_mid + axis * span), cfg.pwm_min, cfg.pwm_max)


def make_manual_channel_command(
    axes: ManualAxes,
    cfg: ROVConfig,
    heading_hold: bool,
    depth_hold: bool,
    gear: int = 2,
) -> ChannelCommand:
    return ChannelCommand(
        forward=_axis_to_pwm(axes.forward, cfg),
        yaw=_axis_to_pwm(axes.yaw, cfg),
        lateral=_axis_to_pwm(axes.lateral, cfg),
        vertical=_axis_to_pwm(axes.vertical, cfg),
        light=0,
        heading_hold=1 if heading_hold else 0,
        depth_hold=1 if depth_hold else 0,
        gear=gear,
        arm=1,
    )


class ROVInteractiveApp:
    """Tkinter control panel for real-world and --sim runs.

    The app still sends ChannelCommand frames to the ROV/simulator. Per-thruster
    PWM values shown on screen are software-mixer previews for debugging and logs.
    """

    def __init__(self, args):
        self.args = args
        self.cfg = ROVConfig()
        self.cfg.rov_ip = args.ip
        self.cfg.rov_port = args.port
        self.cfg.max_speed_mps = args.speed
        self.cfg.gnss_validate_checksum = not args.no_gnss_checksum
        self.cfg.rov_heading_offset_deg = getattr(args, "rov_heading_offset_deg", self.cfg.rov_heading_offset_deg)
        self.cfg.gnss_net_ip = args.gnss_net_ip
        self.cfg.gnss_net_port = args.gnss_net_port
        self.cfg.gnss_net_connect_timeout_s = args.gnss_net_connect_timeout_s
        self.cfg.gnss_net_recv_timeout_s = args.gnss_net_recv_timeout_s
        self.cfg.gnss_net_log_dir = args.gnss_net_log_dir
        self.cfg.gnss_net_print_raw_bytes = args.gnss_print_raw_bytes
        self.cfg.gnss_port = args.gnss_port
        self.cfg.gnss_baudrate = args.gnss_baudrate
        self.cfg.gnss_timeout_s = args.gnss_timeout_s
        self.cfg.gnss_log_dir = args.gnss_log_dir
        self.cfg.gnss_print_raw_bytes = args.gnss_print_raw_bytes
        self.cfg.sim_gnss_base_dir = args.sim_gnss_base_dir
        self.cfg.sim_cid = args.sim_cid
        self.cfg.ntrip_config_enabled = self.cfg.ntrip_config_enabled and not args.no_ntrip_config
        self.cfg.ntrip_config_transport = args.ntrip_transport
        self.cfg.ntrip_net_ip = args.ntrip_net_ip
        self.cfg.ntrip_net_port = args.ntrip_net_port
        self.cfg.ntrip_net_connect_timeout_s = args.ntrip_net_connect_timeout_s
        self.cfg.ntrip_net_recv_timeout_s = args.ntrip_net_recv_timeout_s
        self.cfg.ntrip_serial_port = args.ntrip_port
        self.cfg.ntrip_serial_baudrate = args.ntrip_baudrate
        self.cfg.ntrip_serial_timeout_s = args.ntrip_timeout_s
        self.cfg.ntrip_state_file = args.ntrip_state_file
        self.cfg.dvl_enabled = not getattr(args, "no_dvl", False)
        self.cfg.dvl_ip = getattr(args, "dvl_ip", self.cfg.dvl_ip)
        self.cfg.dvl_port = getattr(args, "dvl_port", self.cfg.dvl_port)
        self.cfg.dvl_connect_timeout_s = getattr(args, "dvl_connect_timeout_s", self.cfg.dvl_connect_timeout_s)
        self.cfg.dvl_recv_timeout_s = getattr(args, "dvl_recv_timeout_s", self.cfg.dvl_recv_timeout_s)
        self.cfg.dvl_stale_timeout_s = getattr(args, "dvl_stale_timeout_s", self.cfg.dvl_stale_timeout_s)
        self.cfg.dvl_log_dir = getattr(args, "dvl_log_dir", self.cfg.dvl_log_dir)
        self.cfg.dvl_print_raw = getattr(args, "dvl_print_raw", self.cfg.dvl_print_raw)
        self.cfg.dvl_position_gain = getattr(args, "dvl_position_gain", self.cfg.dvl_position_gain)
        self.cfg.dvl_velocity_gain = getattr(args, "dvl_velocity_gain", self.cfg.dvl_velocity_gain)
        self.cfg.map_scale_m_per_unit = getattr(args, "map_scale", self.cfg.map_scale_m_per_unit)

        self.folder = create_test_folder(args.log_dir)
        save_config(self.cfg, self.folder)
        self.logger = CSVLogger(self.folder)

        self._maybe_configure_ntrip()
        self.gnss_reader = self._create_gnss_reader()
        self.dvl_reader = self._create_dvl_reader()
        self.planner = WaypointPlanner(
            max_speed_mps=self.cfg.max_speed_mps,
            arrival_radius_m=self.cfg.arrival_radius_m,
        )
        self.original_waypoints = build_demo_waypoints()
        self.waypoints = list(self.original_waypoints)
        self.manual_waypoints: list[LocalPoint] = []
        self.planner.set_path(self.waypoints)
        self.logger.log_waypoints(self.waypoints)
        self.plotter = TrajectoryPlotter(planned=self.waypoints)

        self.nav = SixAxisIMUNavigator(self.cfg)
        self.nav.initialize(self.cfg.manual_initial_yaw_deg, self.waypoints[0], time.time())

        self.controller = HighLevelController(self.cfg)
        self.controller.arm()

        self.client = ROVSocketClient(self.cfg.rov_ip, self.cfg.rov_port, recv_timeout_s=0.005)
        self.client.connect()
        self.logger.log_event(time.time(), "INFO", "connected", f"ROV socket {self.cfg.rov_ip}:{self.cfg.rov_port}")

        self.root = tk.Tk()
        self.root.title("ROV 自动巡航交互控制 App" + (" --sim" if args.sim else ""))
        # 放大默认窗口，避免导航图被下方区域挤得过小。
        self.root.geometry("1500x900")
        self.root.minsize(1180, 720)
        self.root.protocol("WM_DELETE_WINDOW", self.close)

        self.auto_enabled = tk.BooleanVar(value=not getattr(args, "start_paused", False))
        self.heading_hold_enabled = tk.BooleanVar(value=not getattr(args, "no_heading_hold", False))
        self.depth_hold_enabled = tk.BooleanVar(value=(not getattr(args, "no_depth_hold", False)) and abs(self.cfg.target_depth_m) > 0.05)
        self.manual_override_enabled = tk.BooleanVar(value=False)
        # 鼠标点击航点默认关闭，避免误点轨迹图就改写航线。需要时在窗口中手动打开。
        self.mouse_waypoint_enabled = tk.BooleanVar(value=False)
        self.estopped = tk.BooleanVar(value=False)
        # DVL 接收开关：打开才连接/读取/融合 DVL；关闭表示当前没有 DVL 数据参与导航。
        self.dvl_receive_enabled = tk.BooleanVar(value=bool(self.dvl_reader is not None))

        self.show_gnss_raw_enabled = tk.BooleanVar(value=False)
        self.gnss_raw_text = None

        self.axis_vars = {
            "forward": tk.DoubleVar(value=0.0),
            "yaw": tk.DoubleVar(value=0.0),
            "lateral": tk.DoubleVar(value=0.0),
            "vertical": tk.DoubleVar(value=0.0),
        }
        self.info_vars: dict[str, tk.StringVar] = {}
        self.pwm_vars: dict[str, tk.StringVar] = {}
        self.thruster_vars: dict[str, tk.StringVar] = {}
        self.pressed_keys: set[str] = set()
        self.closed = False
        self.last_status: PathStatus | None = None
        self.last_out: ControlOutput | None = None
        self.gnss_status_text = "未启用" if self.gnss_reader is None else "等待 GNSS"
        self.dvl_status_text = "未启用" if self.dvl_reader is None else "等待 DVL"
        self.last_imu = None
        self.live_plots_enabled = not getattr(args, "no_live_plots", False)
        # 图表刷新是 UI 卡顿的主要来源；控制指令仍按 command_period_s 高频发送。
        self.plot_update_s = max(0.1, float(getattr(args, "plot_update_s", 0.25)))
        self.display_update_s = max(0.1, float(getattr(args, "display_update_s", 0.25)))
        self.max_live_plot_points = int(max(50, getattr(args, "max_live_plot_points", 300)))
        self.plot_backend = str(getattr(args, "plot_backend", "auto") or "auto").lower()
        self.gpu_plotter = None
        self.gpu_plot_enabled = False
        self.last_plot_update = 0.0
        self.last_display_update = 0.0
        self.fig_traj = None
        self.ax_traj = None
        self.canvas_traj = None
        self.fig_heading = None
        self.ax_heading = None
        self.canvas_heading = None
        self.plot_notebook = None
        self.map_scale_var = tk.DoubleVar(value=float(self.cfg.map_scale_m_per_unit))
        self.traj_view_locked = False
        self.traj_locked_xlim = None
        self.traj_locked_ylim = None

        self._build_ui()
        self._bind_keys()
        self._refresh_button_text()


    def _maybe_configure_ntrip(self) -> None:
        if self.args.sim or self.args.no_gnss:
            return
        try:
            configure_ntrip_if_needed(
                self.cfg,
                force=self.args.force_ntrip_config,
                logger=self.logger,
            )
        except Exception as exc:
            # 如果 GNSS 模块已经配置过但状态文件丢失，不阻塞主流程，只写日志提醒。
            self.logger.log_event(time.time(), "WARN", "ntrip_config_failed", str(exc))
            print(f"NTRIP 一键配置失败，主程序继续启动：{exc}")

    def _create_gnss_reader(self):
        if self.args.no_gnss:
            return None

        if self.args.gnss_source == "file":
            raw_path = (
                Path(self.args.gnss_raw_path)
                if self.args.gnss_raw_path
                else default_sim_gnss_raw_path(self.args.sim_gnss_base_dir, self.args.sim_cid)
            )
            self.logger.log_event(time.time(), "INFO", "gnss_file_reader", f"GNSS raw file={raw_path}")
            return GNSSLogTailReader(
                raw_path,
                validate_checksum=self.cfg.gnss_validate_checksum,
                read_from_beginning=False,
            )

        if self.args.gnss_source == "net":
            self.logger.log_event(
                time.time(),
                "INFO",
                "gnss_net_reader",
                f"GNSS net={self.cfg.gnss_net_ip}:{self.cfg.gnss_net_port}",
            )
            return GNSSNetReader(
                GNSSNetConfig(
                    host=self.cfg.gnss_net_ip,
                    port=self.cfg.gnss_net_port,
                    connect_timeout_s=self.cfg.gnss_net_connect_timeout_s,
                    recv_timeout_s=self.cfg.gnss_net_recv_timeout_s,
                    log_dir=Path(self.cfg.gnss_net_log_dir),
                    print_raw_bytes=self.cfg.gnss_net_print_raw_bytes,
                ),
                validate_checksum=self.cfg.gnss_validate_checksum,
            )

        # 串口读取作为调试/兼容保留，实机 app 默认不再使用。
        gnss_config = GNSSSerialConfig(
            port=self.args.gnss_port,
            baudrate=self.args.gnss_baudrate,
            timeout_s=self.args.gnss_timeout_s,
            log_dir=Path(self.args.gnss_log_dir),
            print_raw_bytes=self.args.gnss_print_raw_bytes,
        )
        self.logger.log_event(
            time.time(),
            "INFO",
            "gnss_serial_reader",
            f"GNSS serial={self.args.gnss_port}, baudrate={self.args.gnss_baudrate}",
        )
        return GNSSSerialReader(gnss_config, validate_checksum=self.cfg.gnss_validate_checksum)

    def _create_dvl_reader(self):
        if not getattr(self.cfg, "dvl_enabled", True):
            return None
        self.logger.log_event(
            time.time(),
            "INFO",
            "dvl_reader",
            f"DVL net={self.cfg.dvl_ip}:{self.cfg.dvl_port}",
        )
        return DVLTCPReader(
            DVLConfig(
                host=self.cfg.dvl_ip,
                port=self.cfg.dvl_port,
                connect_timeout_s=self.cfg.dvl_connect_timeout_s,
                recv_timeout_s=self.cfg.dvl_recv_timeout_s,
                stale_timeout_s=self.cfg.dvl_stale_timeout_s,
                log_dir=Path(self.cfg.dvl_log_dir),
                print_raw=self.cfg.dvl_print_raw,
            )
        )

    def toggle_dvl_receive(self) -> None:
        """UI switch for DVL receiving/fusion.

        打开：创建 TCP 读数器并开始接收，但只有 DVL 数据 fresh 且 velocity_valid=true 时才参与融合。
        关闭：关闭 TCP 连接，不读取、不融合，状态明确显示“未接收”。
        """
        enabled = bool(self.dvl_receive_enabled.get())
        self.cfg.dvl_enabled = enabled
        if enabled:
            if self.dvl_reader is None:
                try:
                    self.dvl_reader = self._create_dvl_reader()
                    self.dvl_status_text = "已打开，等待 DVL 数据" if self.dvl_reader is not None else "打开失败"
                    self.logger.log_event(time.time(), "INFO", "dvl_receive_toggle", "ON")
                except Exception as exc:
                    self.dvl_reader = None
                    self.dvl_receive_enabled.set(False)
                    self.dvl_status_text = f"打开失败: {exc}"
                    self.logger.log_event(time.time(), "WARN", "dvl_receive_toggle_failed", str(exc))
            else:
                self.dvl_status_text = "已打开，等待 DVL 数据"
        else:
            if self.dvl_reader is not None:
                try:
                    self.dvl_reader.close()
                except Exception:
                    pass
            self.dvl_reader = None
            self.dvl_status_text = "未接收：DVL 开关关闭"
            self.logger.log_event(time.time(), "INFO", "dvl_receive_toggle", "OFF")

        if "dvl" in self.info_vars:
            self.info_vars["dvl"].set(self.dvl_status_text)

    def _build_ui(self) -> None:
        main = ttk.Frame(self.root, padding=10)
        main.grid(row=0, column=0, sticky="nsew")
        self.root.columnconfigure(0, weight=1)
        self.root.rowconfigure(0, weight=1)

        control = ttk.LabelFrame(main, text="运行控制")
        control.grid(row=0, column=0, sticky="nsew", padx=4, pady=4)

        self.auto_button = ttk.Checkbutton(
            control,
            text="自动巡航",
            variable=self.auto_enabled,
            command=lambda: self._log_toggle("auto_cruise", self.auto_enabled.get()),
        )
        self.auto_button.grid(row=0, column=0, sticky="w", padx=4, pady=3)

        self.heading_button = ttk.Checkbutton(
            control,
            text="定向/航向保持",
            variable=self.heading_hold_enabled,
            command=lambda: self._log_toggle("heading_hold", self.heading_hold_enabled.get()),
        )
        self.heading_button.grid(row=0, column=1, sticky="w", padx=4, pady=3)

        self.depth_button = ttk.Checkbutton(
            control,
            text="定深保持",
            variable=self.depth_hold_enabled,
            command=lambda: self._log_toggle("depth_hold", self.depth_hold_enabled.get()),
        )
        self.depth_button.grid(row=0, column=2, sticky="w", padx=4, pady=3)

        self.dvl_button = ttk.Checkbutton(
            control,
            text="接收 DVL 数据",
            variable=self.dvl_receive_enabled,
            command=self.toggle_dvl_receive,
        )
        self.dvl_button.grid(row=0, column=3, sticky="w", padx=4, pady=3)

        self.manual_button = ttk.Checkbutton(
            control,
            text="手柄干预自动巡航",
            variable=self.manual_override_enabled,
            command=lambda: self._log_toggle("manual_override", self.manual_override_enabled.get()),
        )
        self.manual_button.grid(row=1, column=0, sticky="w", padx=4, pady=3)

        ttk.Button(control, text="清零手柄", command=self.zero_manual_axes).grid(row=1, column=1, sticky="ew", padx=4, pady=3)
        ttk.Button(control, text="急停/解除急停", command=self.toggle_estop).grid(row=1, column=2, sticky="ew", padx=4, pady=3)
        ttk.Button(control, text="退出", command=self.close).grid(row=1, column=3, sticky="ew", padx=4, pady=3)

        manual = ttk.LabelFrame(main, text="手柄/鼠标干预轴，范围 -100% ~ +100%")
        manual.grid(row=1, column=0, sticky="nsew", padx=4, pady=4)
        axis_names = [
            ("forward", "前进/后退 W/S"),
            ("yaw", "转向 A/D"),
            ("lateral", "横移 Q/E"),
            ("vertical", "升降 R/F"),
        ]
        for row, (key, label) in enumerate(axis_names):
            ttk.Label(manual, text=label).grid(row=row, column=0, sticky="w", padx=4, pady=2)
            scale = ttk.Scale(manual, from_=-100, to=100, orient="horizontal", variable=self.axis_vars[key])
            scale.grid(row=row, column=1, sticky="ew", padx=4, pady=2)
            val = ttk.Label(manual, textvariable=self._var(f"axis_{key}", "0%"), width=8)
            val.grid(row=row, column=2, sticky="e", padx=4, pady=2)
        manual.columnconfigure(1, weight=1)

        mouse_plan = ttk.LabelFrame(main, text="路径规划鼠标干预（最高优先级）")
        mouse_plan.grid(row=2, column=0, sticky="nsew", padx=4, pady=4)
        ttk.Checkbutton(
            mouse_plan,
            text="启用鼠标点击添加航点",
            variable=self.mouse_waypoint_enabled,
            command=lambda: self._log_toggle("mouse_waypoint_click", self.mouse_waypoint_enabled.get()),
        ).grid(row=0, column=0, columnspan=2, sticky="w", padx=4, pady=2)
        ttk.Label(mouse_plan, text="地图比例：1 图上单位 =").grid(row=1, column=0, sticky="w", padx=4, pady=2)
        ttk.Spinbox(mouse_plan, from_=0.05, to=20.0, increment=0.05, textvariable=self.map_scale_var, width=8).grid(row=1, column=1, sticky="w", padx=4, pady=2)
        ttk.Label(mouse_plan, text="m，默认 1 m").grid(row=1, column=2, sticky="w", padx=4, pady=2)
        ttk.Button(mouse_plan, text="清除鼠标规划点/恢复默认航线", command=self.clear_manual_waypoints).grid(row=1, column=3, sticky="ew", padx=4, pady=2)
        ttk.Label(mouse_plan, text="开关打开后，在右侧“实时轨迹跟踪”图中左键点击，可连续添加规划点；这些点会作为最高优先级航线，但不会自动启动巡航。关闭开关后点击图形不会改航线。").grid(row=2, column=0, columnspan=4, sticky="w", padx=4, pady=2)
        mouse_plan.columnconfigure(3, weight=1)

        telemetry = ttk.LabelFrame(main, text="导航与状态")
        telemetry.grid(row=0, column=1, rowspan=2, sticky="nsew", padx=4, pady=4)
        labels = [
            ("mode", "控制模式"),
            ("socket", "通信"),
            ("gnss", "GNSS"),
            ("dvl", "DVL"),
            ("wp", "航点"),
            ("manual_wp", "鼠标规划点"),
            ("pos", "位置 x/y/z"),
            ("heading", "航向"),
            ("target", "目标航向/距离"),
            ("battery", "电池/漏水"),
            ("log", "日志目录"),
        ]
        for row, (key, label) in enumerate(labels):
            ttk.Label(telemetry, text=label).grid(row=row, column=0, sticky="w", padx=4, pady=2)
            ttk.Label(telemetry, textvariable=self._var(key, "--"), width=38).grid(row=row, column=1, sticky="w", padx=4, pady=2)

        ttk.Checkbutton(
            telemetry,
            text="窗口显示 GNSS 原始数据",
            variable=self.show_gnss_raw_enabled,
        ).grid(row=len(labels), column=0, columnspan=2, sticky="w", padx=4, pady=3)

        self.gnss_raw_text = tk.Text(telemetry, height=8, width=58)
        self.gnss_raw_text.grid(row=len(labels) + 1, column=0, columnspan=2, sticky="nsew", padx=4, pady=3)
        self.gnss_raw_text.insert("end", "未开启 GNSS 原始数据显示\n")
        self.gnss_raw_text.configure(state="disabled")
        telemetry.rowconfigure(len(labels) + 1, weight=1)

        pwm = ttk.LabelFrame(main, text="通道 PWM")
        pwm.grid(row=3, column=0, sticky="nsew", padx=4, pady=4)
        for col, key in enumerate(("forward", "yaw", "lateral", "vertical")):
            ttk.Label(pwm, text=key).grid(row=0, column=col, padx=4, pady=2)
            self.pwm_vars[key] = tk.StringVar(value="1500")
            ttk.Label(pwm, textvariable=self.pwm_vars[key], width=10).grid(row=1, column=col, padx=4, pady=2)
        for col, key in enumerate(("heading_hold", "depth_hold", "gear", "arm")):
            ttk.Label(pwm, text=key).grid(row=2, column=col, padx=4, pady=2)
            self.pwm_vars[key] = tk.StringVar(value="--")
            ttk.Label(pwm, textvariable=self.pwm_vars[key], width=10).grid(row=3, column=col, padx=4, pady=2)

        thrusters = ttk.LabelFrame(main, text="各推进器 PWM 预览")
        thrusters.grid(row=3, column=1, sticky="nsew", padx=4, pady=4)
        layout = ThrusterLayout()
        names = ["l1", "l2", "l3", "l4", "r1", "r2", "r3", "r4"]
        labels_by_name = layout.__dict__
        for i, name in enumerate(names):
            row = i // 2
            col = (i % 2) * 2
            ttk.Label(thrusters, text=labels_by_name[name]).grid(row=row, column=col, sticky="w", padx=4, pady=2)
            self.thruster_vars[name] = tk.StringVar(value="1500")
            ttk.Label(thrusters, textvariable=self.thruster_vars[name], width=8).grid(row=row, column=col + 1, sticky="e", padx=4, pady=2)

        self._build_plot_panel(main)

        main.columnconfigure(0, weight=1)
        main.columnconfigure(1, weight=1)
        # 第 3 列专门留给导航图，权重更高，使“实时轨迹跟踪”明显放大。
        main.columnconfigure(2, weight=4)
        main.rowconfigure(0, weight=0)
        main.rowconfigure(1, weight=1)
        main.rowconfigure(2, weight=0)
        main.rowconfigure(3, weight=0)
        main.rowconfigure(4, weight=0)

        self.info_vars["socket"].set(f"已连接 {self.cfg.rov_ip}:{self.cfg.rov_port}")
        self.info_vars["log"].set(str(self.folder))
    def _update_gnss_raw_window(self) -> None:
        if self.gnss_raw_text is None:
            return

        if not self.show_gnss_raw_enabled.get():
            return

        if self.gnss_reader is None:
            return

        pop_func = getattr(self.gnss_reader, "pop_recent_raw_lines", None)
        if pop_func is None:
            return

        lines = pop_func()
        if not lines:
            return

        self.gnss_raw_text.configure(state="normal")

        for line in lines[-20:]:
            self.gnss_raw_text.insert("end", line + "\n")

        # 限制窗口内最多保留约 200 行，防止 Text 控件越来越卡
        current_lines = int(self.gnss_raw_text.index("end-1c").split(".")[0])
        if current_lines > 200:
            self.gnss_raw_text.delete("1.0", f"{current_lines - 200}.0")

        self.gnss_raw_text.see("end")
        self.gnss_raw_text.configure(state="disabled")
    def _build_plot_panel(self, parent: ttk.Frame) -> None:
        plots = ttk.LabelFrame(parent, text="实时轨迹与航向校正效果图")
        # 绘图区移到右侧独立大列，占满主界面高度；左侧其余控制区域保持原功能不变。
        plots.grid(row=0, column=2, rowspan=5, sticky="nsew", padx=4, pady=4)
        plots.columnconfigure(0, weight=1)
        plots.rowconfigure(0, weight=1)

        if not self.live_plots_enabled:
            ttk.Label(
                plots,
                text="实时绘图已通过 --no-live-plots 关闭；轨迹与航向数据仍写入 CSV，退出时保存 PNG。",
            ).grid(row=0, column=0, sticky="nsew", padx=8, pady=8)
            return

        # 优先使用 pyqtgraph + OpenGL/GPU 的独立绘图进程。
        # 这样主 Tk 控制窗口不会被绘图刷新阻塞；若现场电脑未安装依赖，则自动回退到原 Matplotlib。
        if self.plot_backend in ("auto", "gpu", "opengl", "pyqtgraph"):
            self.gpu_plotter = GpuRealtimePlotter(title="ROV GPU/OpenGL 实时轨迹与航向图")
            if self.gpu_plotter.start():
                self.gpu_plot_enabled = True
                ttk.Label(
                    plots,
                    text=(
                        "已启用 GPU/OpenGL 绘图：轨迹/航向图会在独立窗口显示。\n"
                        "主控制窗口不再承担 Matplotlib 重绘，控制、通信、日志和手动航点功能保持运行。\n"
                        "鼠标航点：在 GPU 轨迹窗口左键点击；仍受窗口里的“鼠标点击航点”开关控制。"
                    ),
                    justify="left",
                ).grid(row=0, column=0, sticky="nsew", padx=10, pady=10)
                self._update_live_plots(force=True)
                return
            if self.plot_backend in ("gpu", "opengl", "pyqtgraph"):
                ttk.Label(
                    plots,
                    text=(
                        f"GPU/OpenGL 绘图不可用：{self.gpu_plotter.reason}。\n"
                        "请安装 pyqtgraph 和 PyQt6/PySide6/PyQt5 后重试，或改用 --plot-backend matplotlib。"
                    ),
                    justify="left",
                ).grid(row=0, column=0, sticky="nsew", padx=8, pady=8)
                self.live_plots_enabled = False
                return

        if not _lazy_import_matplotlib_tk():
            ttk.Label(
                plots,
                text=(
                    "未能加载 matplotlib Tk 后端；轨迹与航向数据仍写入 CSV。\n"
                    "通常是 numpy 被升级到 2.x 导致旧 matplotlib 不兼容，"
                    "可执行：python -m pip install --force-reinstall numpy==1.24.4"
                ),
            ).grid(row=0, column=0, sticky="nsew", padx=8, pady=8)
            self.live_plots_enabled = False
            return

        notebook = ttk.Notebook(plots)
        self.plot_notebook = notebook
        notebook.grid(row=0, column=0, sticky="nsew")

        traj_tab = ttk.Frame(notebook)
        heading_tab = ttk.Frame(notebook)
        notebook.add(traj_tab, text="实时轨迹跟踪")
        notebook.add(heading_tab, text="航向校正效果")

        self.fig_traj = Figure(figsize=(8.8, 7.2), dpi=100)
        self.ax_traj = self.fig_traj.add_subplot(111)
        self.canvas_traj = FigureCanvasTkAgg(self.fig_traj, master=traj_tab)
        self.canvas_traj.get_tk_widget().pack(fill="both", expand=True)
        self.canvas_traj.mpl_connect("button_press_event", self._on_traj_click)

        self.fig_heading = Figure(figsize=(8.8, 7.2), dpi=100)
        self.ax_heading = self.fig_heading.add_subplot(111)
        self.canvas_heading = FigureCanvasTkAgg(self.fig_heading, master=heading_tab)
        self.canvas_heading.get_tk_widget().pack(fill="both", expand=True)

        self._draw_trajectory_plot()
        self._draw_heading_plot()


    def _sample_live_points(self, points: list[LocalPoint]) -> list[LocalPoint]:
        """Live UI only: down-sample long tracks so Tk/matplotlib does not freeze.

        CSV logs and saved data remain complete; this only affects the on-screen drawing.
        """
        if len(points) <= self.max_live_plot_points:
            return points
        step = max(1, len(points) // self.max_live_plot_points)
        sampled = points[::step]
        if sampled[-1] is not points[-1]:
            sampled.append(points[-1])
        return sampled

    def _sample_series(self, *series):
        """Live UI only: down-sample equal-length time series for faster heading plots."""
        if not series or not series[0]:
            return series
        n = len(series[0])
        if n <= self.max_live_plot_points:
            return series
        step = max(1, n // self.max_live_plot_points)
        idx = list(range(0, n, step))
        if idx[-1] != n - 1:
            idx.append(n - 1)
        return tuple([seq[i] for i in idx] for seq in series)

    def _draw_trajectory_plot(self, preserve_view: bool = False) -> None:
        if self.ax_traj is None or self.canvas_traj is None:
            return

        ax = self.ax_traj
        old_xlim = old_ylim = None
        if preserve_view:
            if self.traj_locked_xlim is not None and self.traj_locked_ylim is not None:
                old_xlim, old_ylim = self.traj_locked_xlim, self.traj_locked_ylim
            elif ax.has_data():
                old_xlim, old_ylim = ax.get_xlim(), ax.get_ylim()

        ax.clear()
        rtk_track = self._sample_live_points(self.plotter.rtk_track)
        imu_track = self._sample_live_points(self.plotter.imu_track)
        corrected_track = self._sample_live_points(self.plotter.corrected_track)
        if self.plotter.planned:
            ax.plot(
                [p.x for p in self.plotter.planned],
                [p.y for p in self.plotter.planned],
                marker="o",
                label="planned waypoints",
            )
        if self.manual_waypoints:
            ax.scatter(
                [p.x for p in self.manual_waypoints],
                [p.y for p in self.manual_waypoints],
                marker="s",
                s=60,
                label="manual priority waypoints",
            )
            ax.plot([p.x for p in self.manual_waypoints], [p.y for p in self.manual_waypoints], linestyle=":")
        if rtk_track:
            ax.plot([p.x for p in rtk_track], [p.y for p in rtk_track], label="RTK track")
        if imu_track:
            ax.plot(
                [p.x for p in imu_track],
                [p.y for p in imu_track],
                label="IMU dead reckoning",
            )
        if corrected_track:
            ax.plot(
                [p.x for p in corrected_track],
                [p.y for p in corrected_track],
                label="GNSS/DVL corrected track",
            )

        current = self.nav.state.local_point()
        ax.scatter([current.x], [current.y], marker="x", s=70, label="current ROV")
        heading_deg = self.nav.state.yaw_deg
        dx = 0.8 * math.sin(math.radians(heading_deg))
        dy = 0.8 * math.cos(math.radians(heading_deg))
        ax.arrow(current.x, current.y, dx, dy, head_width=0.18, length_includes_head=True)

        if self.last_status is not None and self.last_status.target is not None:
            target = self.last_status.target
            ax.scatter([target.x], [target.y], marker="*", s=95, label="active target")
            ax.plot([current.x, target.x], [current.y, target.y], linestyle="--", label="current target line")

        ax.set_title("ROV local X-Y trajectory")
        ax.set_xlabel("X East / m")
        ax.set_ylabel("Y North / m")
        ax.axis("equal")
        ax.grid(True)
        ax.legend(loc="best", fontsize=8)
        if old_xlim is not None and old_ylim is not None:
            ax.set_xlim(old_xlim)
            ax.set_ylim(old_ylim)
        # 不在每次刷新都 tight_layout，避免 matplotlib 布局计算拖慢 Tk 窗口。
        self.canvas_traj.draw_idle()

    def _draw_heading_plot(self) -> None:
        if self.ax_heading is None or self.canvas_heading is None:
            return

        ax = self.ax_heading
        ax.clear()
        if self.plotter.heading_time_s:
            t0 = self.plotter.heading_time_s[0]
            time_s, target_h, raw_h, corrected_h, gnss_h = self._sample_series(
                self.plotter.heading_time_s,
                self.plotter.target_heading_deg,
                self.plotter.raw_heading_deg,
                self.plotter.corrected_heading_deg,
                self.plotter.gnss_course_deg,
            )
            t = [x - t0 for x in time_s]
            ax.plot(t, target_h, label="target heading")
            ax.plot(t, raw_h, label="raw IMU yaw")
            ax.plot(t, corrected_h, label="corrected heading")
            if any(x is not None for x in gnss_h):
                tg = [ti for ti, g in zip(t, gnss_h) if g is not None]
                gg = [g for g in gnss_h if g is not None]
                ax.scatter(tg, gg, s=12, label="GNSS course observation")
        else:
            ax.text(0.5, 0.5, "waiting for heading data", ha="center", va="center", transform=ax.transAxes)

        ax.set_title("Heading correction effect")
        ax.set_xlabel("time / s")
        ax.set_ylabel("heading / deg")
        ax.set_ylim(-5, 365)
        ax.grid(True)
        ax.legend(loc="best", fontsize=8)
        # 不在每次刷新都 tight_layout，避免 matplotlib 布局计算拖慢 Tk 窗口。
        self.canvas_heading.draw_idle()

    def _points_to_xy_list(self, points: list[LocalPoint]) -> list[tuple[float, float]]:
        return [(float(p.x), float(p.y)) for p in points]

    def _build_plot_snapshot(self) -> dict:
        current = self.nav.state.local_point()
        target = None
        if self.last_status is not None and self.last_status.target is not None:
            target = (float(self.last_status.target.x), float(self.last_status.target.y))

        heading_payload = {"t": [], "target": [], "raw": [], "corrected": [], "gnss": []}
        if self.plotter.heading_time_s:
            t0 = self.plotter.heading_time_s[0]
            time_s, target_h, raw_h, corrected_h, gnss_h = self._sample_series(
                self.plotter.heading_time_s,
                self.plotter.target_heading_deg,
                self.plotter.raw_heading_deg,
                self.plotter.corrected_heading_deg,
                self.plotter.gnss_course_deg,
            )
            heading_payload = {
                "t": [float(x - t0) for x in time_s],
                "target": [float(x) for x in target_h],
                "raw": [float(x) for x in raw_h],
                "corrected": [float(x) for x in corrected_h],
                "gnss": [None if x is None else float(x) for x in gnss_h],
            }

        return {
            "planned": self._points_to_xy_list(self.plotter.planned),
            "manual": self._points_to_xy_list(self.manual_waypoints),
            "rtk": self._points_to_xy_list(self._sample_live_points(self.plotter.rtk_track)),
            "imu": self._points_to_xy_list(self._sample_live_points(self.plotter.imu_track)),
            "corrected": self._points_to_xy_list(self._sample_live_points(self.plotter.corrected_track)),
            "current": (float(current.x), float(current.y)),
            "target": target,
            "heading_deg": float(self.nav.state.yaw_deg),
            "raw_heading_deg": float(self.nav.state.raw_yaw_deg),
            "target_heading_deg": None if self.last_status is None else float(self.last_status.target_heading_deg),
            "gnss_course_deg": None if self.nav.state.gnss_course_deg is None else float(self.nav.state.gnss_course_deg),
            "dvl_status": self.dvl_status_text,
            "heading": heading_payload,
            "lock_traj_view": bool(self.traj_view_locked),
        }

    def _poll_gpu_plot_events(self) -> None:
        if self.gpu_plotter is None:
            return
        for evt in self.gpu_plotter.pop_events():
            typ = evt.get("type")
            if typ == "ready":
                self.info_vars["manual_wp"].set(evt.get("message", "GPU/OpenGL 绘图窗口已启动"))
            elif typ == "error":
                self.info_vars["manual_wp"].set(evt.get("message", "GPU/OpenGL 绘图异常"))
            elif typ == "traj_click":
                if not self.mouse_waypoint_enabled.get():
                    self.info_vars["manual_wp"].set("鼠标航点开关未打开，GPU窗口点击已忽略")
                    continue
                try:
                    scale = max(0.01, float(self.map_scale_var.get()))
                except Exception:
                    scale = 1.0
                    self.map_scale_var.set(1.0)
                p = LocalPoint(float(evt.get("x", 0.0)) * scale, float(evt.get("y", 0.0)) * scale, self.nav.state.z)
                self.manual_waypoints.append(p)
                self._activate_manual_waypoints()
                self.info_vars["manual_wp"].set(f"{len(self.manual_waypoints)} 个，当前最高优先级")
                self.logger.log_event(time.time(), "INFO", "manual_waypoint_add_gpu", f"x={p.x:.3f}, y={p.y:.3f}, scale={scale:.3f}")

    def _update_live_plots(self, force: bool = False, preserve_traj_view: bool = False) -> None:
        if not self.live_plots_enabled:
            return
        now = time.time()
        if not force and now - self.last_plot_update < self.plot_update_s:
            return
        self.last_plot_update = now

        if self.gpu_plot_enabled and self.gpu_plotter is not None:
            self.gpu_plotter.update(self._build_plot_snapshot())
            return

        # Matplotlib 回退模式：只刷新当前可见页，隐藏页不重绘，降低 Tk 卡顿。
        if self.plot_notebook is not None:
            try:
                selected = self.plot_notebook.tab(self.plot_notebook.select(), "text")
            except Exception:
                selected = ""
            if "航向" in selected:
                self._draw_heading_plot()
            else:
                self._draw_trajectory_plot(preserve_view=preserve_traj_view or self.traj_view_locked)
        else:
            self._draw_trajectory_plot(preserve_view=preserve_traj_view or self.traj_view_locked)

    def _save_plots(self) -> None:
        trajectory_path = self.folder / "trajectory.png"
        heading_path = self.folder / "heading_correction.png"
        try:
            if self.fig_traj is not None and self.fig_heading is not None:
                self._draw_trajectory_plot()
                self._draw_heading_plot()
                self.fig_traj.savefig(trajectory_path, dpi=160)
                self.fig_heading.savefig(heading_path, dpi=160)
            else:
                self.plotter.save_png(str(trajectory_path))
                self.plotter.save_heading_png(str(heading_path))
            self.logger.log_event(time.time(), "INFO", "save_plots", f"{trajectory_path}; {heading_path}")
        except Exception as exc:
            self.logger.log_event(time.time(), "WARN", "save_plots_failed", str(exc))

    def _activate_manual_waypoints(self) -> None:
        if not self.manual_waypoints:
            return

        # 鼠标航点只提升路径优先级，不自动勾选“自动巡航”。
        # 同时锁定当前轨迹图视野，避免新航点距离较远时导航图自动缩放，影响观察。
        self.traj_view_locked = True
        if self.ax_traj is not None and self.ax_traj.has_data():
            self.traj_locked_xlim = self.ax_traj.get_xlim()
            self.traj_locked_ylim = self.ax_traj.get_ylim()

        self.waypoints = list(self.manual_waypoints)
        self.planner.set_path(self.waypoints)
        self.plotter.planned = self.waypoints
        self.manual_override_enabled.set(False)
        self.logger.log_waypoints(self.waypoints)
        self.logger.log_event(time.time(), "INFO", "manual_waypoints_active", f"count={len(self.manual_waypoints)}, auto={self.auto_enabled.get()}")
        self._refresh_button_text()
        self._update_live_plots(force=True, preserve_traj_view=True)

    def clear_manual_waypoints(self) -> None:
        self.manual_waypoints.clear()
        self.waypoints = list(self.original_waypoints)
        self.planner.set_path(self.waypoints)
        self.plotter.planned = self.waypoints
        self.traj_view_locked = False
        self.traj_locked_xlim = None
        self.traj_locked_ylim = None
        self.info_vars["manual_wp"].set("未设置，使用默认航线")
        self.logger.log_event(time.time(), "INFO", "manual_waypoints_clear", "恢复默认航线")
        self._update_live_plots(force=True)

    def _on_traj_click(self, event) -> None:
        if event.inaxes != self.ax_traj or event.xdata is None or event.ydata is None:
            return
        if event.button != 1:
            return
        if not self.mouse_waypoint_enabled.get():
            self.info_vars["manual_wp"].set("鼠标航点开关未打开，点击已忽略")
            return
        try:
            scale = max(0.01, float(self.map_scale_var.get()))
        except Exception:
            scale = 1.0
            self.map_scale_var.set(1.0)
        p = LocalPoint(float(event.xdata) * scale, float(event.ydata) * scale, self.nav.state.z)
        self.manual_waypoints.append(p)
        self._activate_manual_waypoints()
        self.info_vars["manual_wp"].set(f"{len(self.manual_waypoints)} 个，当前最高优先级")
        self.logger.log_event(time.time(), "INFO", "manual_waypoint_add", f"x={p.x:.3f}, y={p.y:.3f}, scale={scale:.3f}")

    def _bind_keys(self) -> None:
        self.root.bind("<KeyPress>", self._on_key_press)
        self.root.bind("<KeyRelease>", self._on_key_release)
        self.root.focus_set()

    def _var(self, key: str, default: str) -> tk.StringVar:
        if key not in self.info_vars:
            self.info_vars[key] = tk.StringVar(value=default)
        return self.info_vars[key]

    def _log_toggle(self, name: str, enabled: bool) -> None:
        self.logger.log_event(time.time(), "INFO", name, "on" if enabled else "off")
        self._refresh_button_text()

    def _refresh_button_text(self) -> None:
        self.info_vars.get("mode", tk.StringVar()).set("急停" if self.estopped.get() else ("自动巡航" if self.auto_enabled.get() else "待机/手动"))

    def toggle_estop(self) -> None:
        if self.estopped.get():
            self.estopped.set(False)
            self.controller.arm()
            self.logger.log_event(time.time(), "INFO", "estop_reset", "解除急停并重新解锁")
        else:
            self.estopped.set(True)
            try:
                self.client.send_channel(self.controller.emergency_stop())
            except Exception as exc:
                self.logger.log_event(time.time(), "ERROR", "estop_send_failed", str(exc))
            self.logger.log_event(time.time(), "WARN", "estop", "急停触发")
        self._refresh_button_text()

    def zero_manual_axes(self) -> None:
        for var in self.axis_vars.values():
            var.set(0.0)
        self.pressed_keys.clear()
        self.logger.log_event(time.time(), "INFO", "manual_zero", "手柄轴已清零")

    def _on_key_press(self, event: tk.Event) -> None:
        key = (event.keysym or "").lower()
        if key == "space":
            self.toggle_estop()
            return
        if key == "z":
            self.zero_manual_axes()
            return
        if key in {"w", "s", "a", "d", "q", "e", "r", "f"}:
            self.pressed_keys.add(key)
            self.manual_override_enabled.set(True)
            self._update_axes_from_keyboard()

    def _on_key_release(self, event: tk.Event) -> None:
        key = (event.keysym or "").lower()
        if key in self.pressed_keys:
            self.pressed_keys.remove(key)
            self._update_axes_from_keyboard()

    def _update_axes_from_keyboard(self) -> None:
        forward = (1.0 if "w" in self.pressed_keys else 0.0) + (-1.0 if "s" in self.pressed_keys else 0.0)
        yaw = (1.0 if "d" in self.pressed_keys else 0.0) + (-1.0 if "a" in self.pressed_keys else 0.0)
        lateral = (1.0 if "e" in self.pressed_keys else 0.0) + (-1.0 if "q" in self.pressed_keys else 0.0)
        vertical = (1.0 if "r" in self.pressed_keys else 0.0) + (-1.0 if "f" in self.pressed_keys else 0.0)
        self.axis_vars["forward"].set(forward * 100.0)
        self.axis_vars["yaw"].set(yaw * 100.0)
        self.axis_vars["lateral"].set(lateral * 100.0)
        self.axis_vars["vertical"].set(vertical * 100.0)
        self._update_axis_display(self._manual_axes())

    def _manual_axes(self) -> ManualAxes:
        axes = ManualAxes(
            forward=float(self.axis_vars["forward"].get()) / 100.0,
            yaw=float(self.axis_vars["yaw"].get()) / 100.0,
            lateral=float(self.axis_vars["lateral"].get()) / 100.0,
            vertical=float(self.axis_vars["vertical"].get()) / 100.0,
        )
        return axes

    def _update_axis_display(self, axes: ManualAxes) -> None:
        for key, value in axes.__dict__.items():
            self.info_vars[f"axis_{key}"].set(f"{value * 100:+.0f}%")

    def _poll_navigation(self) -> None:
        frames = self.client.poll_sensors()
        for imu in frames:
            self.last_imu = imu
            state = self.nav.update_with_imu(imu, commanded_speed_mps=self.cfg.max_speed_mps)
            self.logger.log_imu(imu)
            self.logger.log_trajectory("trajectory_raw.csv", imu.timestamp, state.local_point(), state.source)
            self.plotter.add_imu(state.local_point())

        if not self.dvl_receive_enabled.get():
            self.dvl_status_text = "未接收：DVL 开关关闭"
        elif self.dvl_reader is not None:
            dvl = self.dvl_reader.poll()
            raw_status = self.dvl_reader.status_text
            if dvl is not None and self.dvl_reader.is_fresh_valid():
                self.dvl_status_text = "接收中：有效，已参与融合"
                dvl_state = self.nav.correct_with_dvl(dvl)
                self.logger.log_trajectory("trajectory_dvl.csv", dvl.timestamp, dvl_state.local_point(), dvl_state.source)
                self.plotter.add_corrected(dvl_state.local_point())
            else:
                self.dvl_status_text = f"接收中：{raw_status}，未参与融合"

        if self.gnss_reader is None:
            return

        fix = self.gnss_reader.poll()
        if fix is None:
            return

        if is_usable_for_navigation(fix, require_fixed=False):
            origin_was_empty = self.nav.local_frame is None
            corrected = self.nav.correct_with_gnss(fix, depth_m=self.nav.state.z, require_fixed_origin=False)
            local_pos = self.nav.gnss_to_local(fix, depth_m=corrected.z)
            if origin_was_empty and self.nav.local_frame is not None:
                self.logger.log_event(fix.timestamp, "INFO", "set_origin", "使用第一条有效GGA作为局部坐标原点")
            self.logger.log_gnss(fix, local_pos)
            if local_pos is not None:
                self.logger.log_trajectory("trajectory_corrected.csv", fix.timestamp, corrected.local_point(), corrected.source)
                self.plotter.add_rtk(local_pos)
                self.plotter.add_corrected(corrected.local_point())
            self.gnss_status_text = f"{fix.fix_status} sats={fix.satellites} hdop={fix.hdop:.1f}"
        else:
            self.logger.log_gnss(fix, None)
            self.logger.log_event(fix.timestamp, "WARN", "gnss_lost", f"GGA无效/无定位: sats={fix.satellites}, hdop={fix.hdop}")
            self.gnss_status_text = f"无效 sats={fix.satellites} hdop={fix.hdop:.1f}"

    def _choose_command(self, status: PathStatus) -> ControlOutput:
        now = time.time()
        axes = self._manual_axes()
        manual_engaged = self.manual_override_enabled.get() and axes.active()

        if self.estopped.get():
            cmd = ChannelCommand.neutral(arm=0)
            return ControlOutput(cmd, "estop", status.target_heading_deg, self.nav.state.yaw_deg, 0.0, self.cfg.target_depth_m, self.nav.state.z, 0.0, now)

        if manual_engaged:
            cmd = make_manual_channel_command(
                axes,
                self.cfg,
                heading_hold=self.heading_hold_enabled.get(),
                depth_hold=self.depth_hold_enabled.get(),
                gear=2,
            )
            return ControlOutput(cmd, "manual_override", status.target_heading_deg, self.nav.state.yaw_deg, 0.0, self.cfg.target_depth_m, self.nav.state.z, 0.0, now)

        if self.auto_enabled.get():
            if not self.controller.enabled:
                self.controller.arm()
            return self.controller.make_command(
                self.nav.state,
                status,
                target_depth_m=self.cfg.target_depth_m,
                heading_hold=self.heading_hold_enabled.get(),
                depth_hold=self.depth_hold_enabled.get(),
            )

        cmd = ChannelCommand.neutral(arm=1 if self.controller.enabled else 0)
        cmd.heading_hold = 1 if self.heading_hold_enabled.get() else 0
        cmd.depth_hold = 1 if self.depth_hold_enabled.get() else 0
        cmd.gear = 2
        return ControlOutput(cmd, "standby", status.target_heading_deg, self.nav.state.yaw_deg, 0.0, self.cfg.target_depth_m, self.nav.state.z, 0.0, now)

    def _update_display(self, out: ControlOutput, status: PathStatus) -> None:
        cmd = out.command
        self._update_axis_display(self._manual_axes())
        self.info_vars["mode"].set(out.mode)
        self.info_vars["gnss"].set(self.gnss_status_text)
        self.info_vars["dvl"].set(self.dvl_status_text)
        self.info_vars["wp"].set(f"{status.waypoint_index}/{len(self.waypoints)} arrived={status.arrived}")
        if self.manual_waypoints:
            self.info_vars["manual_wp"].set(f"{len(self.manual_waypoints)} 个，当前最高优先级")
        else:
            self.info_vars["manual_wp"].set("未设置，使用默认航线")
        self.info_vars["pos"].set(f"x={self.nav.state.x:.2f} y={self.nav.state.y:.2f} z={self.nav.state.z:.2f} m")
        self.info_vars["heading"].set(
            f"yaw={self.nav.state.yaw_deg:.1f}° raw={self.nav.state.raw_yaw_deg:.1f}° offset={self.nav.state.yaw_offset_deg:.1f}°"
        )
        self.info_vars["target"].set(
            f"target={status.target_heading_deg:.1f}° remain={status.remaining_distance_m:.2f}m xte={status.cross_track_error_m:.2f}m"
        )
        if self.last_imu is None:
            self.info_vars["battery"].set("等待传感器帧")
        else:
            leak = "正常" if self.last_imu.no_leak else "漏水报警"
            self.info_vars["battery"].set(f"{self.last_imu.battery_percent}% / {leak} / depth={self.last_imu.depth_m:.2f}m")

        self.pwm_vars["forward"].set(str(cmd.forward))
        self.pwm_vars["yaw"].set(str(cmd.yaw))
        self.pwm_vars["lateral"].set(str(cmd.lateral))
        self.pwm_vars["vertical"].set(str(cmd.vertical))
        self.pwm_vars["heading_hold"].set("ON" if cmd.heading_hold else "OFF")
        self.pwm_vars["depth_hold"].set("ON" if cmd.depth_hold else "OFF")
        self.pwm_vars["gear"].set(str(cmd.gear))
        self.pwm_vars["arm"].set(str(cmd.arm))

        for name, value in preview_thruster_pwms(cmd, self.cfg).items():
            self.thruster_vars[name].set(str(value))

    def _tick(self) -> None:
        if self.closed:
            return

        try:
            self._poll_navigation()
            status = self.planner.update(self.nav.state.local_point())
            self.planner.append_history(self.nav.state.local_point())
            out = self._choose_command(status)

            if status.arrived and out.mode == "auto":
                self.auto_enabled.set(False)
                out = ControlOutput(
                    ChannelCommand.neutral(arm=0),
                    "arrived_stop",
                    status.target_heading_deg,
                    self.nav.state.yaw_deg,
                    0.0,
                    self.cfg.target_depth_m,
                    self.nav.state.z,
                    0.0,
                    time.time(),
                )
                self.logger.log_event(time.time(), "INFO", "arrived", "到达终点，已停止")

            self.client.send_channel(out.command)
            self.logger.log_control(out)
            self.plotter.add_heading(
                self.nav.state.timestamp if self.nav.state.timestamp else time.time(),
                status.target_heading_deg,
                self.nav.state.raw_yaw_deg,
                self.nav.state.yaw_deg,
                self.nav.state.gnss_course_deg,
            )
            self.last_status = status
            self.last_out = out
            now = time.time()
            if now - self.last_display_update >= self.display_update_s:
                self.last_display_update = now
                self._update_display(out, status)
                self._update_gnss_raw_window()
                self._poll_gpu_plot_events()
                self._update_live_plots()
        except Exception as exc:
            self.info_vars["socket"].set(f"错误：{exc}")
            self.logger.log_event(time.time(), "ERROR", "interactive_loop", str(exc))
            try:
                self.client.send_channel(ChannelCommand.neutral(arm=0))
            except Exception:
                pass

        self.root.after(max(20, int(self.cfg.command_period_s * 1000)), self._tick)

    def run(self) -> None:
        self.root.after(50, self._tick)
        self.root.mainloop()

    def close(self) -> None:
        if self.closed:
            return
        self.closed = True
        try:
            self.client.send_channel(ChannelCommand.neutral(arm=0))
        except Exception:
            pass
        try:
            self.client.close()
        finally:
            if self.gnss_reader is not None:
                self.gnss_reader.close()
            if self.dvl_reader is not None:
                self.dvl_reader.close()
            if self.gpu_plotter is not None:
                self.gpu_plotter.close()
            self._save_plots()
            self.logger.close()
            print(f"日志已保存：{self.folder}")
            try:
                self.root.destroy()
            except Exception:
                pass


def run_interactive_auto_surface(args) -> None:
    try:
        app = ROVInteractiveApp(args)
    except Exception as exc:
        # If Tk has not started yet, also print to terminal for field debugging.
        print(f"交互 App 启动失败：{exc}")
        try:
            messagebox.showerror("ROV App 启动失败", str(exc))
        except Exception:
            pass
        raise
    app.run()
