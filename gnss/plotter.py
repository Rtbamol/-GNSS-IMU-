from __future__ import annotations
from dataclasses import dataclass, field
import math
from rov_control.geo import LocalPoint


@dataclass
class TrajectoryPlotter:
    planned: list[LocalPoint] = field(default_factory=list)
    rtk_track: list[LocalPoint] = field(default_factory=list)
    imu_track: list[LocalPoint] = field(default_factory=list)
    corrected_track: list[LocalPoint] = field(default_factory=list)
    heading_time_s: list[float] = field(default_factory=list)
    target_heading_deg: list[float] = field(default_factory=list)
    raw_heading_deg: list[float] = field(default_factory=list)
    corrected_heading_deg: list[float] = field(default_factory=list)
    gnss_course_deg: list[float | None] = field(default_factory=list)

    def add_rtk(self, p: LocalPoint) -> None:
        self.rtk_track.append(p)

    def add_imu(self, p: LocalPoint) -> None:
        self.imu_track.append(p)

    def add_corrected(self, p: LocalPoint) -> None:
        self.corrected_track.append(p)

    def add_heading(
        self,
        timestamp: float,
        target_heading_deg: float,
        raw_heading_deg: float,
        corrected_heading_deg: float,
        gnss_course_deg: float | None = None,
    ) -> None:
        if not self.heading_time_s:
            t0 = timestamp
        else:
            t0 = self.heading_time_s[0]
        # heading_time_s[0] 存的是真实首时间戳，绘图时统一转换成相对秒。
        self.heading_time_s.append(timestamp if len(self.heading_time_s) == 0 else timestamp)
        self.target_heading_deg.append(target_heading_deg % 360.0)
        self.raw_heading_deg.append(raw_heading_deg % 360.0)
        self.corrected_heading_deg.append(corrected_heading_deg % 360.0)
        self.gnss_course_deg.append(None if gnss_course_deg is None else gnss_course_deg % 360.0)

    def _import_pyplot(self):
        """Lazy import matplotlib only when the matplotlib backend / PNG export is actually used.

        This keeps --plot-backend gpu from importing matplotlib at startup, which is
        important on field machines where NumPy 2.x may have broken old compiled
        matplotlib/scipy wheels.
        """
        try:
            import matplotlib.pyplot as plt
            return plt
        except Exception as exc:
            raise RuntimeError(
                "matplotlib 当前不可用。若需要使用 matplotlib 后端或退出时保存 PNG，"
                "请先执行：python -m pip install --force-reinstall numpy==1.24.4"
            ) from exc

    def show_heading_once(self) -> None:
        if not self.heading_time_s:
            return
        plt = self._import_pyplot()
        t0 = self.heading_time_s[0]
        t = [x - t0 for x in self.heading_time_s]
        plt.figure("ROV heading correction")
        plt.clf()
        plt.plot(t, self.target_heading_deg, label="target heading")
        plt.plot(t, self.raw_heading_deg, label="initial/raw IMU yaw")
        plt.plot(t, self.corrected_heading_deg, label="GNSS corrected heading")
        if any(x is not None for x in self.gnss_course_deg):
            tg = [ti for ti, g in zip(t, self.gnss_course_deg) if g is not None]
            gg = [g for g in self.gnss_course_deg if g is not None]
            plt.scatter(tg, gg, s=12, label="GNSS course observation")
        plt.xlabel("time / s")
        plt.ylabel("heading / deg")
        plt.ylim(-5, 365)
        plt.grid(True)
        plt.legend()
        plt.pause(0.001)

    def show_once(self, current: LocalPoint | None = None, heading_deg: float | None = None) -> None:
        plt = self._import_pyplot()
        plt.figure("ROV local X-Y trajectory")
        plt.clf()
        if self.planned:
            plt.plot([p.x for p in self.planned], [p.y for p in self.planned], marker="o", label="planned")
        if self.rtk_track:
            plt.plot([p.x for p in self.rtk_track], [p.y for p in self.rtk_track], label="rtk")
        if self.imu_track:
            plt.plot([p.x for p in self.imu_track], [p.y for p in self.imu_track], label="imu dead reckoning")
        if self.corrected_track:
            plt.plot([p.x for p in self.corrected_track], [p.y for p in self.corrected_track], label="corrected")
        if current is not None:
            plt.scatter([current.x], [current.y], marker="x", s=80, label="current")
            if heading_deg is not None:
                dx = 1.0 * math.sin(math.radians(heading_deg))
                dy = 1.0 * math.cos(math.radians(heading_deg))
                plt.arrow(current.x, current.y, dx, dy, head_width=0.2, length_includes_head=True)
        plt.xlabel("X East / m")
        plt.ylabel("Y North / m")
        plt.axis("equal")
        plt.grid(True)
        plt.legend()
        plt.pause(0.001)

    def save_png(self, path: str) -> None:
        self.show_once()
        plt = self._import_pyplot()
        plt.savefig(path, dpi=160)

    def save_heading_png(self, path: str) -> None:
        self.show_heading_once()
        plt = self._import_pyplot()
        plt.savefig(path, dpi=160)
