from __future__ import annotations

import math
from collections import deque
from typing import Callable, Dict, Iterable, Optional, Sequence, Tuple

from PyQt5.QtCore import QPointF, QRectF, Qt, pyqtSignal
from PyQt5.QtGui import QColor, QFont, QPainter, QPainterPath, QPen, QPolygonF
from PyQt5.QtWidgets import QLabel, QSizePolicy, QWidget

from .models import GroundSnapshot, Point3, VehicleState

BG = QColor('#111722')
PANEL = QColor('#171f2c')
GRID = QColor('#2b3545')
TEXT = QColor('#dfe7f1')
MUTED = QColor('#8995a7')
CYAN = QColor('#38bdf8')
GREEN = QColor('#4ade80')
AMBER = QColor('#fbbf24')
RED = QColor('#fb7185')
PURPLE = QColor('#a78bfa')
WHITE = QColor('#f8fafc')


class StatusChip(QLabel):
    def __init__(self, text: str = 'OFFLINE') -> None:
        super().__init__(text)
        self.setAlignment(Qt.AlignCenter)
        self.setMinimumWidth(88)
        self.set_state(text)

    def set_state(self, text: str, ok: bool = False, warning: bool = False) -> None:
        color = '#22c55e' if ok else ('#f59e0b' if warning else '#ef4444')
        self.setText(text)
        self.setStyleSheet(
            f'QLabel {{ background:{color}; color:#071017; border-radius:10px; padding:4px 10px; font-weight:700; }}'
        )


class RpmWidget(QWidget):
    def __init__(self) -> None:
        super().__init__()
        self.values = [0.0] * 4
        self.setMinimumHeight(145)

    def set_values(self, values: Sequence[float]) -> None:
        self.values = list(values[:4]) + [0.0] * max(0, 4 - len(values))
        self.update()

    def paintEvent(self, event) -> None:
        p = QPainter(self); p.setRenderHint(QPainter.Antialiasing)
        p.fillRect(self.rect(), PANEL)
        margin, top, bottom = 14, 18, 28
        width = max(1, self.width() - 2 * margin)
        bar_w = width / 4.0
        maximum = max(10000.0, max(self.values) * 1.05)
        labels = ['FL', 'FR', 'RR', 'RL']
        for i, value in enumerate(self.values):
            x = margin + i * bar_w + bar_w * 0.22
            w = bar_w * 0.56
            hmax = max(1.0, self.height() - top - bottom)
            h = hmax * min(max(value / maximum, 0.0), 1.0)
            p.setPen(Qt.NoPen); p.setBrush(QColor('#243044'))
            p.drawRoundedRect(QRectF(x, top, w, hmax), 5, 5)
            p.setBrush(CYAN if value < 9800 else RED)
            p.drawRoundedRect(QRectF(x, top + hmax - h, w, h), 5, 5)
            p.setPen(TEXT); p.setFont(QFont('Sans', 8, QFont.Bold))
            p.drawText(QRectF(x - 8, self.height() - bottom + 4, w + 16, 18), Qt.AlignCenter, labels[i])
            p.setPen(MUTED); p.setFont(QFont('Monospace', 7))
            p.drawText(QRectF(x - 12, 2, w + 24, 14), Qt.AlignCenter, f'{value:.0f}')


class MiniChart(QWidget):
    def __init__(self, title: str, unit: str, color: QColor = CYAN, limit: int = 360) -> None:
        super().__init__()
        self.title, self.unit, self.color = title, unit, color
        self.values = deque(maxlen=limit)
        self.setMinimumHeight(125)
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)

    def clear(self) -> None:
        self.values.clear(); self.update()

    def append(self, value: Optional[float]) -> None:
        if value is not None and math.isfinite(float(value)): self.values.append(float(value))
        self.update()

    def paintEvent(self, event) -> None:
        p = QPainter(self); p.setRenderHint(QPainter.Antialiasing)
        p.fillRect(self.rect(), PANEL)
        p.setPen(TEXT); p.setFont(QFont('Sans', 9, QFont.Bold))
        p.drawText(12, 18, self.title)
        if not self.values:
            p.setPen(MUTED); p.drawText(self.rect(), Qt.AlignCenter, 'NO DATA'); return
        values = list(self.values)
        lo, hi = min(values), max(values)
        if abs(hi - lo) < 1e-6: lo -= 0.5; hi += 0.5
        pad = (hi - lo) * 0.12; lo -= pad; hi += pad
        area = QRectF(12, 28, max(1, self.width() - 24), max(1, self.height() - 42))
        p.setPen(QPen(GRID, 1))
        for i in range(4):
            y = area.top() + i * area.height() / 3.0
            p.drawLine(QPointF(area.left(), y), QPointF(area.right(), y))
        path = QPainterPath()
        for i, v in enumerate(values):
            x = area.left() + (i / max(1, len(values) - 1)) * area.width()
            y = area.bottom() - (v - lo) / (hi - lo) * area.height()
            if i == 0: path.moveTo(x, y)
            else: path.lineTo(x, y)
        p.setPen(QPen(self.color, 2)); p.drawPath(path)
        p.setPen(TEXT); p.setFont(QFont('Monospace', 8, QFont.Bold))
        p.drawText(QRectF(area.right() - 95, 4, 95, 18), Qt.AlignRight | Qt.AlignVCenter, f'{values[-1]:.3f} {self.unit}')


class MissionMapWidget(QWidget):
    targetPicked = pyqtSignal(float, float)

    def __init__(self) -> None:
        super().__init__()
        self.snapshot: Optional[GroundSnapshot] = None
        self.center = [0.0, 0.0]
        self.meters_per_screen = 12.0
        self.drag_last = None
        self.auto_fit_pending = True
        self.setMinimumSize(600, 420)
        self.setMouseTracking(True)
        self.setToolTip('Double-click to select goal X/Y. Wheel: zoom. Drag: pan.')

    def set_snapshot(self, snapshot: GroundSnapshot) -> None:
        self.snapshot = snapshot
        if self.auto_fit_pending:
            self.fit_to_data()
            self.auto_fit_pending = False
        self.update()

    def request_auto_fit(self) -> None:
        self.auto_fit_pending = True

    def fit_to_data(self) -> None:
        if self.snapshot is None: return
        xs, ys = [], []
        occ = self.snapshot.occupancy
        if occ.width > 0 and occ.height > 0:
            xs += [occ.origin_x, occ.origin_x + occ.width * occ.resolution]
            ys += [occ.origin_y, occ.origin_y + occ.height * occ.resolution]
        for state in self.snapshot.vehicles.values():
            if state.connected(self.snapshot.monotonic_time):
                xs.append(state.position[0]); ys.append(state.position[1])
                for point in state.actual_path[-500:]: xs.append(point[0]); ys.append(point[1])
                if state.has_goal: xs.append(state.goal[0]); ys.append(state.goal[1])
        for path in [self.snapshot.planned_path, self.snapshot.planned_path_3d, self.snapshot.trajectory_reference]:
            for point in path: xs.append(point[0]); ys.append(point[1])
        dyn = self.snapshot.dynamic_obstacle
        if dyn.position: xs.append(dyn.position[0]); ys.append(dyn.position[1])
        for point in dyn.predictions: xs.append(point[0]); ys.append(point[1])
        if not xs: xs, ys = [-5, 5], [-5, 5]
        minx, maxx, miny, maxy = min(xs), max(xs), min(ys), max(ys)
        self.center = [(minx + maxx) / 2.0, (miny + maxy) / 2.0]
        span = max(maxx - minx, maxy - miny, 4.0)
        self.meters_per_screen = span * 1.25

    def _scale(self) -> float:
        return min(self.width(), self.height()) / max(1e-6, self.meters_per_screen)

    def world_to_screen(self, x: float, y: float) -> QPointF:
        s = self._scale()
        return QPointF(self.width() / 2 + (x - self.center[0]) * s, self.height() / 2 - (y - self.center[1]) * s)

    def screen_to_world(self, point: QPointF) -> Tuple[float, float]:
        s = self._scale()
        return self.center[0] + (point.x() - self.width() / 2) / s, self.center[1] - (point.y() - self.height() / 2) / s

    def _draw_path(self, p: QPainter, points: Iterable[Point3], color: QColor, width: float, dashed: bool = False) -> None:
        points = list(points)
        if len(points) < 2: return
        pen = QPen(color, width)
        if dashed: pen.setStyle(Qt.DashLine)
        p.setPen(pen); p.setBrush(Qt.NoBrush)
        path = QPainterPath()
        first = self.world_to_screen(points[0][0], points[0][1]); path.moveTo(first)
        for point in points[1:]: path.lineTo(self.world_to_screen(point[0], point[1]))
        p.drawPath(path)

    def _nice_step(self) -> float:
        target = self.meters_per_screen / 8.0
        power = 10 ** math.floor(math.log10(max(target, 1e-6)))
        for m in (1, 2, 5, 10):
            if m * power >= target: return m * power
        return 10 * power

    def paintEvent(self, event) -> None:
        p = QPainter(self); p.setRenderHint(QPainter.Antialiasing)
        p.fillRect(self.rect(), BG)
        step = self._nice_step(); half = self.meters_per_screen * 0.75
        minx, maxx = self.center[0] - half, self.center[0] + half
        miny, maxy = self.center[1] - half, self.center[1] + half
        x = math.floor(minx / step) * step
        p.setFont(QFont('Monospace', 7)); p.setPen(QPen(GRID, 1))
        while x <= maxx:
            a, b = self.world_to_screen(x, miny), self.world_to_screen(x, maxy)
            p.drawLine(a, b); p.setPen(MUTED); p.drawText(QPointF(a.x() + 2, self.height() - 5), f'{x:g}'); p.setPen(QPen(GRID, 1)); x += step
        y = math.floor(miny / step) * step
        while y <= maxy:
            a, b = self.world_to_screen(minx, y), self.world_to_screen(maxx, y)
            p.drawLine(a, b); p.setPen(MUTED); p.drawText(QPointF(4, a.y() - 2), f'{y:g}'); p.setPen(QPen(GRID, 1)); y += step

        if self.snapshot is None:
            p.setPen(TEXT); p.drawText(self.rect(), Qt.AlignCenter, 'WAITING FOR ROS 2 DATA'); return
        snap = self.snapshot
        # occupancy
        p.setPen(Qt.NoPen); p.setBrush(QColor(82, 96, 117, 145))
        cell = max(2.0, snap.occupancy.resolution * self._scale())
        for ox, oy in snap.occupancy.occupied_points:
            q = self.world_to_screen(ox, oy); p.drawRect(QRectF(q.x() - cell/2, q.y() - cell/2, cell, cell))
        # route layers
        self._draw_path(p, snap.planned_path, AMBER, 2.0, True)
        self._draw_path(p, snap.planned_path_3d, PURPLE, 2.4, True)
        self._draw_path(p, snap.trajectory_reference, GREEN, 2.2, False)
        colors = {'single': CYAN, 'uav1': GREEN, 'uav2': PURPLE}
        for key, state in snap.vehicles.items():
            self._draw_path(p, state.actual_path, colors[key], 2.0, False)
            if state.has_goal:
                g = self.world_to_screen(state.goal[0], state.goal[1])
                p.setPen(QPen(colors[key], 2)); p.setBrush(Qt.NoBrush)
                p.drawEllipse(g, 8, 8); p.drawLine(g + QPointF(-12, 0), g + QPointF(12, 0)); p.drawLine(g + QPointF(0, -12), g + QPointF(0, 12))
        # dynamic obstacle and prediction
        dyn = snap.dynamic_obstacle
        if dyn.position and snap.monotonic_time - dyn.last_update_monotonic < 2.0:
            q = self.world_to_screen(dyn.position[0], dyn.position[1]); r = max(5.0, dyn.radius_m * self._scale())
            p.setBrush(QColor(251, 113, 133, 80)); p.setPen(QPen(RED, 2)); p.drawEllipse(q, r, r)
            self._draw_path(p, dyn.predictions, RED, 1.6, True)
            p.setPen(RED); p.drawText(q + QPointF(10, -10), dyn.phase)
        # inter-UAV line
        s1, s2 = snap.vehicles['uav1'], snap.vehicles['uav2']
        if s1.connected(snap.monotonic_time) and s2.connected(snap.monotonic_time):
            a, b = self.world_to_screen(s1.position[0], s1.position[1]), self.world_to_screen(s2.position[0], s2.position[1])
            color = RED if snap.system.safety_active else QColor('#64748b')
            p.setPen(QPen(color, 1.5, Qt.DashLine)); p.drawLine(a, b)
            if snap.system.current_inter_uav_distance_m is not None:
                mid = QPointF((a.x() + b.x()) / 2.0, (a.y() + b.y()) / 2.0); p.setPen(color); p.drawText(mid + QPointF(5,-5), f'{snap.system.current_inter_uav_distance_m:.2f} m')
        # vehicles last
        for key, state in snap.vehicles.items():
            if not state.connected(snap.monotonic_time): continue
            q = self.world_to_screen(state.position[0], state.position[1])
            yaw = math.radians(state.attitude_deg[2])
            forward = QPointF(math.cos(yaw), -math.sin(yaw))
            side = QPointF(-forward.y(), forward.x())
            poly = QPolygonF([q + forward*13, q - forward*9 + side*8, q - forward*5, q - forward*9 - side*8])
            p.setPen(QPen(WHITE, 1)); p.setBrush(colors[key]); p.drawPolygon(poly)
            p.setPen(TEXT); p.setFont(QFont('Sans', 8, QFont.Bold)); p.drawText(q + QPointF(12, 18), f'{state.label}  z={state.position[2]:.2f}')
        # legend
        p.setFont(QFont('Sans', 8)); p.setPen(TEXT)
        legend = [('Actual', CYAN), ('Trajectory', GREEN), ('3D A*', PURPLE), ('Planned', AMBER), ('Dynamic obstacle', RED)]
        x0, y0 = 12, 14
        for text, color in legend:
            p.setPen(QPen(color, 3)); p.drawLine(x0, y0, x0+18, y0); p.setPen(TEXT); p.drawText(x0+24, y0+4, text); x0 += 95

    def wheelEvent(self, event) -> None:
        factor = 0.82 if event.angleDelta().y() > 0 else 1.22
        self.meters_per_screen = min(max(self.meters_per_screen * factor, 2.0), 500.0); self.update()

    def mousePressEvent(self, event) -> None:
        if event.button() == Qt.LeftButton: self.drag_last = event.pos()

    def mouseMoveEvent(self, event) -> None:
        if self.drag_last is not None and event.buttons() & Qt.LeftButton:
            dx, dy = event.x() - self.drag_last.x(), event.y() - self.drag_last.y()
            scale = self._scale(); self.center[0] -= dx / scale; self.center[1] += dy / scale
            self.drag_last = event.pos(); self.update()

    def mouseReleaseEvent(self, event) -> None:
        self.drag_last = None

    def mouseDoubleClickEvent(self, event) -> None:
        x, y = self.screen_to_world(QPointF(event.pos())); self.targetPicked.emit(x, y)
