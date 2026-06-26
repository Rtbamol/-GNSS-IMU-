from __future__ import annotations

import importlib.util
import multiprocessing as mp
import queue
import time
from dataclasses import dataclass
from typing import Any


def _has_qt_graph_stack() -> tuple[bool, str]:
    if importlib.util.find_spec("pyqtgraph") is None:
        return False, "缺少 pyqtgraph"
    qt_ok = any(importlib.util.find_spec(name) is not None for name in ("PyQt6", "PySide6", "PyQt5", "PySide2"))
    if not qt_ok:
        return False, "缺少 Qt 绑定：PyQt6/PySide6/PyQt5/PySide2"
    return True, "OK"


def _xy(points: list[tuple[float, float]]) -> tuple[list[float], list[float]]:
    if not points:
        return [], []
    return [p[0] for p in points], [p[1] for p in points]


def _gpu_plot_worker(data_q: mp.Queue, event_q: mp.Queue, title: str) -> None:
    """Run pyqtgraph in a separate process so Tk control UI is never blocked by plotting."""
    try:
        import pyqtgraph as pg
        from pyqtgraph.Qt import QtCore, QtWidgets
    except Exception as exc:  # pragma: no cover - depends on field environment
        try:
            event_q.put({"type": "error", "message": f"GPU绘图启动失败：{exc}"})
        except Exception:
            pass
        return

    # pyqtgraph uses OpenGL when available; if GPU/OpenGL is unavailable it falls back internally.
    # 现场调试时黑底不易看清，统一改为浅色背景和深色坐标轴。
    pg.setConfigOptions(useOpenGL=True, antialias=False, background="w", foreground="k")

    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    win = QtWidgets.QMainWindow()
    win.setWindowTitle(title)
    win.resize(1200, 860)

    central = QtWidgets.QWidget()
    layout = QtWidgets.QVBoxLayout(central)
    status_label = QtWidgets.QLabel("航向信息：等待数据")
    status_label.setStyleSheet("font-size: 15px; color: #111; background: #f2f2f2; padding: 6px;")
    layout.addWidget(status_label)

    tabs = QtWidgets.QTabWidget()
    layout.addWidget(tabs)
    win.setCentralWidget(central)

    traj_plot = pg.PlotWidget(title="ROV local X-Y trajectory  GPU/OpenGL")
    traj_plot.setLabel("bottom", "X East", units="m")
    traj_plot.setLabel("left", "Y North", units="m")
    traj_plot.showGrid(x=True, y=True, alpha=0.25)
    traj_plot.setAspectLocked(False)
    traj_plot.addLegend(offset=(10, 10))
    tabs.addTab(traj_plot, "实时轨迹跟踪")

    heading_plot = pg.PlotWidget(title="Heading correction effect  GPU/OpenGL")
    heading_plot.setLabel("bottom", "time", units="s")
    heading_plot.setLabel("left", "heading", units="deg")
    heading_plot.setYRange(-5, 365)
    heading_plot.showGrid(x=True, y=True, alpha=0.25)
    heading_plot.addLegend(offset=(10, 10))
    tabs.addTab(heading_plot, "航向校正效果")

    dot_line = getattr(getattr(QtCore.Qt, "PenStyle", QtCore.Qt), "DotLine")
    dash_line = getattr(getattr(QtCore.Qt, "PenStyle", QtCore.Qt), "DashLine")
    left_button = getattr(getattr(QtCore.Qt, "MouseButton", QtCore.Qt), "LeftButton")

    curves: dict[str, Any] = {
        "planned": traj_plot.plot([], [], pen=pg.mkPen("#1f77b4", width=2), symbol="o", symbolSize=7, symbolBrush="#1f77b4", name="planned waypoints"),
        "manual": traj_plot.plot([], [], pen=pg.mkPen("#d62728", style=dot_line, width=2), symbol="s", symbolSize=8, symbolBrush="#d62728", name="manual priority waypoints"),
        "rtk": traj_plot.plot([], [], pen=pg.mkPen("#2ca02c", width=1), name="RTK track"),
        "imu": traj_plot.plot([], [], pen=pg.mkPen("#7f7f7f", width=1), name="IMU dead reckoning"),
        "corrected": traj_plot.plot([], [], pen=pg.mkPen("#ff7f0e", width=2), name="GNSS/DVL corrected track"),
        "current": traj_plot.plot([], [], pen=None, symbol="x", symbolSize=13, symbolPen=pg.mkPen("#000000", width=2), name="current ROV"),
        "target": traj_plot.plot([], [], pen=None, symbol="star", symbolSize=14, symbolBrush="#9467bd", name="active target"),
        "target_line": traj_plot.plot([], [], pen=pg.mkPen("#9467bd", style=dash_line), name="current target line"),
        "heading_arrow": traj_plot.plot([], [], pen=pg.mkPen("#000000", width=3), name="current heading arrow"),
        "target_h": heading_plot.plot([], [], pen=pg.mkPen("#1f77b4", width=1), name="target heading"),
        "raw_h": heading_plot.plot([], [], pen=pg.mkPen("#7f7f7f", width=1), name="raw IMU yaw"),
        "corrected_h": heading_plot.plot([], [], pen=pg.mkPen("#ff7f0e", width=2), name="corrected heading"),
        "gnss_h": heading_plot.plot([], [], pen=None, symbol="o", symbolSize=5, symbolBrush="#2ca02c", name="GNSS course observation"),
    }
    heading_text = pg.TextItem(text="", color="#111111", anchor=(0, 1), fill=pg.mkBrush(255, 255, 255, 210))
    traj_plot.addItem(heading_text)

    def on_click(evt) -> None:
        if evt.button() != left_button:
            return
        try:
            vb = traj_plot.plotItem.vb
            p = vb.mapSceneToView(evt.scenePos())
            event_q.put({"type": "traj_click", "x": float(p.x()), "y": float(p.y())})
        except Exception:
            pass

    traj_plot.scene().sigMouseClicked.connect(on_click)

    last_autorange = 0.0
    first_data = True

    def apply_snapshot(snap: dict[str, Any]) -> None:
        nonlocal last_autorange, first_data
        for key in ("planned", "manual", "rtk", "imu", "corrected"):
            x, y = _xy(snap.get(key, []))
            curves[key].setData(x, y)

        current = snap.get("current")
        heading = float(snap.get("heading_deg") or 0.0)
        target_heading = snap.get("target_heading_deg")
        raw_heading = snap.get("raw_heading_deg")
        gnss_course = snap.get("gnss_course_deg")
        dvl_status = snap.get("dvl_status") or "--"
        target_text = "--" if target_heading is None else f"{float(target_heading):.1f}°"
        raw_text = "--" if raw_heading is None else f"{float(raw_heading):.1f}°"
        gnss_text = "--" if gnss_course is None else f"{float(gnss_course):.1f}°"
        status_label.setText(
            f"当前航向: {heading:.1f}°    目标航向: {target_text}    IMU原始: {raw_text}    GNSS航迹角: {gnss_text}    DVL: {dvl_status}"
        )
        if current is not None:
            curves["current"].setData([current[0]], [current[1]])
            import math
            arrow_len = max(0.8, float(snap.get("arrow_len", 1.2) or 1.2))
            dx = arrow_len * math.sin(math.radians(heading))
            dy = arrow_len * math.cos(math.radians(heading))
            curves["heading_arrow"].setData([current[0], current[0] + dx], [current[1], current[1] + dy])
            heading_text.setText(f"航向 {heading:.1f}°\n目标 {target_text}")
            heading_text.setPos(current[0], current[1])
        else:
            curves["current"].setData([], [])
            curves["heading_arrow"].setData([], [])
            heading_text.setText("")

        target = snap.get("target")
        if target is not None:
            curves["target"].setData([target[0]], [target[1]])
            if current is not None:
                curves["target_line"].setData([current[0], target[0]], [current[1], target[1]])
            else:
                curves["target_line"].setData([], [])
        else:
            curves["target"].setData([], [])
            curves["target_line"].setData([], [])

        hs = snap.get("heading", {})
        t = hs.get("t", [])
        curves["target_h"].setData(t, hs.get("target", []))
        curves["raw_h"].setData(t, hs.get("raw", []))
        curves["corrected_h"].setData(t, hs.get("corrected", []))
        gnss = hs.get("gnss", [])
        tg = [ti for ti, gi in zip(t, gnss) if gi is not None]
        gg = [gi for gi in gnss if gi is not None]
        curves["gnss_h"].setData(tg, gg)

        now = time.time()
        lock_traj_view = bool(snap.get("lock_traj_view", False))
        if first_data or (not lock_traj_view and now - last_autorange > 5.0):
            first_data = False
            last_autorange = now
            try:
                if not lock_traj_view:
                    traj_plot.enableAutoRange(axis="xy", enable=True)
                heading_plot.enableAutoRange(axis="x", enable=True)
            except Exception:
                pass

    def poll_queue() -> None:
        latest = None
        while True:
            try:
                msg = data_q.get_nowait()
            except queue.Empty:
                break
            if isinstance(msg, dict) and msg.get("type") == "close":
                win.close()
                app.quit()
                return
            latest = msg
        if isinstance(latest, dict):
            apply_snapshot(latest)

    timer = QtCore.QTimer()
    timer.timeout.connect(poll_queue)
    timer.start(50)
    win.show()
    try:
        event_q.put({"type": "ready", "message": "GPU/OpenGL 绘图窗口已启动"})
    except Exception:
        pass
    app.exec()


@dataclass
class GpuRealtimePlotter:
    title: str = "ROV GPU/OpenGL 实时绘图"
    data_queue_size: int = 3

    def __post_init__(self) -> None:
        self._ctx = mp.get_context("spawn")
        self._data_q: mp.Queue | None = None
        self._event_q: mp.Queue | None = None
        self._proc: mp.Process | None = None
        self.available, self.reason = _has_qt_graph_stack()

    def start(self) -> bool:
        if not self.available:
            return False
        if self._proc is not None and self._proc.is_alive():
            return True
        self._data_q = self._ctx.Queue(maxsize=self.data_queue_size)
        self._event_q = self._ctx.Queue(maxsize=20)
        self._proc = self._ctx.Process(
            target=_gpu_plot_worker,
            args=(self._data_q, self._event_q, self.title),
            daemon=True,
        )
        self._proc.start()
        return True

    def update(self, snapshot: dict[str, Any]) -> None:
        if self._data_q is None:
            return
        # Keep only the newest frame; plotting should never block control commands.
        try:
            while self._data_q.full():
                self._data_q.get_nowait()
        except Exception:
            pass
        try:
            self._data_q.put_nowait(snapshot)
        except Exception:
            pass

    def pop_events(self) -> list[dict[str, Any]]:
        if self._event_q is None:
            return []
        events: list[dict[str, Any]] = []
        while True:
            try:
                events.append(self._event_q.get_nowait())
            except Exception:
                break
        return events

    def close(self) -> None:
        if self._data_q is not None:
            try:
                self._data_q.put_nowait({"type": "close"})
            except Exception:
                pass
        if self._proc is not None and self._proc.is_alive():
            self._proc.join(timeout=1.0)
            if self._proc.is_alive():
                self._proc.terminate()
