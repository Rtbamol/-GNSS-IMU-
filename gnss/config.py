from dataclasses import dataclass, asdict, field
from pathlib import Path
from datetime import datetime
import json


@dataclass
class ROVConfig:
    """ROV 测试、通信、GNSS 与 NTRIP 配置。

    常用现场参数尽量集中放在这里；命令行参数仅用于临时覆盖。
    运行方式保持不变：
    - 实机：python -m rov_control.app
    - 仿真：python -m rov_control.app --sim
    """

    # ROV 通信参数
    rov_ip: str = "192.168.1.7"
    sim_rov_ip: str = "127.0.0.1"
    rov_port: int = 8887
    recv_period_s: float = 0.10
    command_period_s: float = 0.10

    # GNSS 来源：实机默认网口，仿真默认文件。
    real_gnss_source: str = "net"      # net / serial / file
    sim_gnss_source: str = "file"      # file / net / serial
    gnss_validate_checksum: bool = True

    # 实机 GNSS 网口采集参数，默认对应 gnss_collect_net.py。
    gnss_net_ip: str = "192.168.7.7"
    gnss_net_port: int = 8848
    gnss_net_connect_timeout_s: float = 5.0
    gnss_net_recv_timeout_s: float = 0.02
    gnss_net_log_dir: str = "gnss_logs_net"
    gnss_net_print_raw_bytes: bool = False

    # 仿真 GNSS raw.txt 参数。
    gnss_base_dir: str = "D:/RTK_LOG"
    gnss_cid: str = "864865086384444"
    gnss_filename: str = "raw.txt"
    gnss_read_from_beginning: bool = False
    sim_gnss_base_dir: str = "sim_rtk_log"
    sim_cid: str = "864865086384444"

    # 保留串口采集/调试参数：实机主流程默认不再使用串口读取 GNSS，
    # 但串口仍用于首次下发 NTRIP 配置和必要时的串口调试。
    gnss_port: str = "COM4"
    gnss_baudrate: int = 115200
    gnss_timeout_s: float = 0.1
    gnss_log_dir: str = "gnss_logs"
    gnss_print_raw_bytes: bool = False

    # NTRIP 一键配置：首次启动实机 app 时默认通过 GNSS 网口 TCP 下发，
    # 成功后写入状态文件，后续跳过。串口方式仅作为兼容/调试保留。
    ntrip_config_enabled: bool = True
    ntrip_config_transport: str = "net"  # net / serial
    ntrip_net_ip: str = "192.168.7.7"
    ntrip_net_port: int = 8848
    ntrip_net_connect_timeout_s: float = 5.0
    ntrip_net_recv_timeout_s: float = 1.0
    ntrip_serial_port: str = "COM4"
    ntrip_serial_baudrate: int = 115200
    ntrip_serial_timeout_s: float = 1.0
    ntrip_command_interval_s: float = 0.5
    ntrip_state_file: str = "gnss_ntrip_configured.json"
    ntrip_commands: tuple[str, ...] = field(
        default_factory=lambda: (
            "AT+NTRIPEN=1",
            "AT+ADDR=120.253.226.97,8002",
            "AT+CORS=cxuj1939,eep79kh3,RTCM33_GRCEJ",
        )
    )

    # 控制通道 PWM 范围，协议中 0x05DC=1500，0x03E8=1000，0x07D0=2000
    pwm_min: int = 1200
    pwm_mid: int = 1500
    pwm_max: int = 1800

    # 自动航行参数
    max_speed_mps: float = 0.5
    arrival_radius_m: float = 0.8
    target_depth_m: float = 0.0
    underwater_depth_m: float = 1.0
    heartbeat_timeout_s: float = 2.5

    # 六轴 IMU 参数：无磁力计，不允许把 yaw 当绝对磁航向
    imu_yaw_init_mode: str = "rtk_course"  # rtk_course / manual / rov_heading
    manual_initial_yaw_deg: float = 0.0
    yaw_gyro_bias_deg_s: float = 0.0
    # True：使用 IMU 自身输出的相对 yaw；False：上位机用 gz_dps 短时积分。
    # 六轴 IMU 无磁力计时，heading/yaw 仍是开机零位下的相对角，不是绝对磁航向。
    use_imu_reported_yaw: bool = True
    imu_alpha_velocity: float = 0.08
    rtk_position_correction_gain: float = 0.85
    # GNSS 航迹角修正 IMU yaw 零偏的低通增益。
    rtk_heading_correction_gain: float = 0.25
    # 单天线 GNSS 只能在运动中由连续位置计算航迹角，低速/位移太小不修 yaw。
    gnss_heading_min_speed_mps: float = 0.05
    gnss_heading_min_distance_m: float = 0.50
    gnss_heading_max_dt_s: float = 5.0

    # ROV 艏向修正角，单位 deg。
    # 单天线 GNSS/RMC 航迹角表示“运动方向”，不一定等于艇体艏向。
    # 若现场表现为“尾部被当作头部、航向整体反 180°”，设为 180.0；若方向正常则设为 0.0。
    rov_heading_offset_deg: float = 0.0

    # RTK 天线相对 ROV 中心点安装偏移，单位 m：x 东、y 北、z 深度向下
    antenna_offset_x_m: float = 0.0
    antenna_offset_y_m: float = 0.0
    antenna_offset_z_m: float = 0.0


    # DVL-A50 对底速度融合参数：默认启用；无数据/失效时自动退回 IMU/GNSS。
    dvl_enabled: bool = True
    dvl_ip: str = "192.168.194.95"
    dvl_port: int = 16171
    # 仿真 DVL 默认走本机；sim_env_simulator --dvl 会在该端口输出 JSON。
    sim_dvl_ip: str = "127.0.0.1"
    sim_dvl_port: int = 16171
    dvl_connect_timeout_s: float = 3.0
    dvl_recv_timeout_s: float = 0.01
    dvl_stale_timeout_s: float = 1.0
    dvl_log_dir: str = "dvl_logs"
    dvl_print_raw: bool = False
    dvl_position_gain: float = 0.75
    dvl_velocity_gain: float = 0.80

    # 鼠标手动规划参数：图上 1 个坐标单位默认等于 1 m，可在窗口或命令行调整。
    map_scale_m_per_unit: float = 1.0

    # 安全阈值
    max_heading_jump_deg: float = 45.0
    max_rtk_jump_m: float = 3.0
    low_battery_percent: int = 25


def create_test_folder(base_dir: str = "logs", prefix: str = "RTK_ROV_Test") -> Path:
    root = Path(base_dir)
    root.mkdir(parents=True, exist_ok=True)
    ts = datetime.now().strftime("%Y_%m_%d_%H%M%S")
    folder = root / f"{prefix}_{ts}"
    folder.mkdir(parents=True, exist_ok=True)
    return folder


def save_config(config: ROVConfig, folder: Path) -> None:
    with open(folder / "config.json", "w", encoding="utf-8") as f:
        json.dump(asdict(config), f, ensure_ascii=False, indent=2)
