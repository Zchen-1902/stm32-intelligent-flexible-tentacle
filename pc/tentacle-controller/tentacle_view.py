from __future__ import annotations

import math
from pathlib import Path

from PySide6.QtCore import QPoint, QPointF, QRect, Qt, Signal
from PySide6.QtGui import (
    QColor, QLinearGradient, QPainter, QPainterPath, QPen,
)
from PySide6.QtWidgets import QWidget


class DirectionDial(QWidget):
    angle_changed = Signal(float)
    bend_wheel = Signal(int, bool)

    def __init__(self, parent: QWidget | None = None) -> None:
        super().__init__(parent)
        self.angle = 0.0
        self.locked = False
        self.dragging = False
        self.setMinimumSize(120, 120)
        self.setFocusPolicy(Qt.StrongFocus)

    def set_angle(self, angle: float) -> None:
        self.angle = angle % 360.0
        self.update()

    def _angle_at(self, point: QPoint) -> float:
        center = self.rect().center()
        dx = point.x() - center.x()
        dy = center.y() - point.y()
        return math.degrees(math.atan2(dy, dx)) % 360.0

    def mousePressEvent(self, event) -> None:
        if event.button() == Qt.LeftButton and not self.locked:
            self.dragging = True
            self.set_angle(self._angle_at(event.position().toPoint()))
            self.angle_changed.emit(self.angle)
            event.accept()

    def mouseMoveEvent(self, event) -> None:
        if self.dragging and not self.locked:
            self.set_angle(self._angle_at(event.position().toPoint()))
            self.angle_changed.emit(self.angle)
            event.accept()

    def mouseReleaseEvent(self, event) -> None:
        self.dragging = False
        super().mouseReleaseEvent(event)

    def wheelEvent(self, event) -> None:
        if not self.locked:
            steps = 1 if event.angleDelta().y() > 0 else -1
            self.bend_wheel.emit(steps, bool(event.modifiers() & Qt.ShiftModifier))
        event.accept()

    def paintEvent(self, _event) -> None:
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)
        rect = self.rect().adjusted(12, 12, -12, -12)
        center = rect.center()
        radius = min(rect.width(), rect.height()) // 2
        painter.setPen(QPen(QColor("#425466"), 2))
        painter.setBrush(QColor("#101820"))
        painter.drawEllipse(center, radius, radius)
        painter.setPen(QPen(QColor("#8ea7bd"), 1))
        for tick in range(0, 360, 30):
            radians = math.radians(tick)
            outer = QPoint(
                int(center.x() + math.cos(radians) * radius),
                int(center.y() - math.sin(radians) * radius),
            )
            inner = QPoint(
                int(center.x() + math.cos(radians) * (radius - (9 if tick % 90 else 15))),
                int(center.y() - math.sin(radians) * (radius - (9 if tick % 90 else 15))),
            )
            painter.drawLine(inner, outer)
        painter.setPen(QColor("#d9e5ef"))
        painter.drawText(QRect(center.x() - 15, rect.top() - 2, 30, 18), Qt.AlignCenter, "90")
        painter.drawText(QRect(rect.right() - 24, center.y() - 10, 24, 20), Qt.AlignCenter, "0")
        painter.drawText(QRect(center.x() - 20, rect.bottom() - 14, 40, 18), Qt.AlignCenter, "270")
        painter.drawText(QRect(rect.left() - 4, center.y() - 10, 28, 20), Qt.AlignCenter, "180")
        radians = math.radians(self.angle)
        point = QPoint(
            int(center.x() + math.cos(radians) * (radius - 25)),
            int(center.y() - math.sin(radians) * (radius - 25)),
        )
        painter.setPen(QPen(QColor("#66d9ef"), 3))
        painter.drawLine(center, point)
        painter.setPen(Qt.NoPen)
        painter.setBrush(QColor("#66d9ef"))
        painter.drawEllipse(point, 8, 8)
        painter.setBrush(QColor("#f1f5f9"))
        painter.drawEllipse(center, 5, 5)


class TentacleViewport(QWidget):
    """Lightweight pseudo-3D posture view without a physics renderer."""

    selected = Signal()

    # 屏幕角度沿用方向圆盘：0°向右、90°向上、270°向下。
    # 四个根部从整机中心分别指向四个外侧45°方向。
    _MOUNT_SCREEN_ANGLES = (135.0, 45.0, 225.0, 315.0)
    _BASE_ANCHORS = ((0.68, 0.68), (0.32, 0.68), (0.68, 0.32), (0.32, 0.32))
    # 实机当前中心对称姿态下，T1/T3为252°、T2/T4为72°；这些局部方向均映射为整机向下。
    _DOWN_REFERENCE_ANGLES = (252.0, 72.0, 252.0, 72.0)
    _DOWNWARD_DISPLAY_GAIN = 1.75

    def __init__(
        self,
        _model_path: Path | None = None,
        parent: QWidget | None = None,
        *,
        inverted: bool = False,
        tentacle_index: int = 0,
    ) -> None:
        super().__init__(parent)
        # 保留 inverted 参数以兼容旧调用，但不再翻转整个画布。
        self.inverted = False
        self.tentacle_index = max(0, min(3, int(tentacle_index)))
        self.angle = 0.0
        self.bend = 0.0
        self.online = True
        self.active = False
        self.view_azimuth = 0.0
        self.view_elevation = 20.0
        self.zoom = 0.92
        self._drag_pos: QPoint | None = None
        self.setMinimumSize(170, 170)
        self.setStyleSheet("background:#101820; border:1px solid #263746;")
        self.setToolTip("左键选择，滚轮缩放")

    def set_tentacle(self, index: int) -> None:
        """切换触手安装坐标，仅影响姿态绘制，不改变控制角度。"""
        self.tentacle_index = max(0, min(3, int(index)))
        self.update()

    def set_state(self, angle: float, bend: float, online: bool, active: bool) -> None:
        self.angle = angle % 360.0
        self.bend = max(0.0, min(100.0, bend))
        self.online = online
        self.active = active
        self.update()

    def _centerline(self) -> list[tuple[float, float, float]]:
        # Distal joints receive more curvature so motion begins at the head.
        segment_count = 25
        total_turn = math.radians(720.0) * (self.bend / 100.0) ** 1.08
        weights = [((index + 1) / segment_count) ** 1.55 for index in range(segment_count)]
        weight_sum = sum(weights)
        direction = math.radians(self.angle)
        radial = (math.cos(direction), math.sin(direction), 0.0)
        position = [0.0, 0.0, 0.0]
        points = [tuple(position)]
        tangent_turn = 0.0
        for index, weight in enumerate(weights):
            tangent_turn += total_turn * weight / weight_sum
            # Exponentially shrinking links preserve the logarithmic-spiral character.
            length = 1.0 * (0.975 ** index)
            horizontal = math.sin(tangent_turn)
            vertical = math.cos(tangent_turn)
            position[0] += radial[0] * horizontal * length
            position[1] += radial[1] * horizontal * length
            position[2] += vertical * length
            points.append(tuple(position))
        return points

    def _screen_points(self) -> list[tuple[QPointF, float]]:
        centerline = self._centerline()
        index = self.tentacle_index
        mount_angle = math.radians(self._MOUNT_SCREEN_ANGLES[index])
        mount_x = math.cos(mount_angle)
        mount_y = -math.sin(mount_angle)
        side_x, side_y = -mount_y, mount_x
        reference = math.radians(self._DOWN_REFERENCE_ANGLES[index])
        reference_x, reference_y = math.cos(reference), math.sin(reference)
        anchor_x, anchor_y = self._BASE_ANCHORS[index]

        # 使用固定标称长度缩放，姿态变化不能推动根部或改变整幅图的缩放。
        nominal_length = sum(0.975 ** index for index in range(25))
        available_length = max(90.0, min(self.width(), self.height()) * 0.60)
        scale = available_length / nominal_length * self.zoom
        screen_base_x = self.width() * anchor_x
        screen_base_y = self.height() * anchor_y

        points: list[tuple[QPointF, float]] = []
        for x, y, z in centerline:
            # 局部参考弯曲分量映射到屏幕向下；其正交分量只保留少量透视偏移。
            downward = x * reference_x + y * reference_y
            depth = -x * reference_y + y * reference_x
            screen_x = screen_base_x + (z * mount_x + depth * side_x * 0.18) * scale
            screen_y = screen_base_y + (
                z * mount_y
                + downward * self._DOWNWARD_DISPLAY_GAIN
                + depth * side_y * 0.18
            ) * scale
            points.append((QPointF(screen_x, screen_y), depth))
        return points

    def paintEvent(self, _event) -> None:
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)
        painter.fillRect(self.rect(), QColor("#101820"))
        points = self._screen_points()
        if len(points) < 2:
            return

        base = points[0][0]
        painter.setPen(Qt.NoPen)
        painter.setBrush(QColor(5, 10, 14, 115))
        painter.drawEllipse(QPointF(base.x() + 8, base.y() + 7), 27, 10)
        base_gradient = QLinearGradient(-25, -10, 25, 10)
        base_gradient.setColorAt(0.0, QColor("#263847"))
        base_gradient.setColorAt(0.55, QColor("#7891a4"))
        base_gradient.setColorAt(1.0, QColor("#1b2934"))
        painter.save()
        painter.translate(base)
        painter.rotate(90.0 - self._MOUNT_SCREEN_ANGLES[self.tentacle_index])
        painter.setBrush(base_gradient)
        painter.setPen(QPen(QColor("#8aa1b2"), 1))
        painter.drawEllipse(QPointF(0.0, 0.0), 27, 11)
        painter.restore()

        base_color = QColor("#39caa5") if self.active else QColor("#687b8d")
        if not self.online:
            base_color = QColor("#4d5862")
        segment_count = len(points) - 1
        for index in range(segment_count):
            start = points[index][0]
            end = points[index + 1][0]
            ratio = index / max(1, segment_count - 1)
            width = 17.0 - 10.5 * ratio
            light = 108 + int(22 * math.sin(ratio * math.pi))
            color = base_color.lighter(light)
            if not self.online:
                color.setAlpha(150)
            painter.setPen(QPen(QColor(0, 0, 0, 90), width + 4, Qt.SolidLine, Qt.RoundCap))
            painter.drawLine(QPointF(start.x() + 3, start.y() + 4), QPointF(end.x() + 3, end.y() + 4))
            painter.setPen(QPen(color, width, Qt.SolidLine, Qt.RoundCap))
            painter.drawLine(start, end)
            highlight = QColor(220, 255, 248, 80 if self.active else 45)
            painter.setPen(QPen(highlight, max(1.5, width * 0.18), Qt.SolidLine, Qt.RoundCap))
            painter.drawLine(QPointF(start.x() - 1.5, start.y() - 1.5), QPointF(end.x() - 1.5, end.y() - 1.5))

        painter.setPen(Qt.NoPen)
        for index in range(2, len(points), 3):
            point = points[index][0]
            ratio = index / (len(points) - 1)
            radius = 4.8 - 2.2 * ratio
            painter.setBrush(base_color.lighter(125))
            painter.drawEllipse(point, radius, radius)

        tip = points[-1][0]
        painter.setBrush(QColor("#83f2d5") if self.active else QColor("#9aabba"))
        painter.drawEllipse(tip, 5.0, 5.0)
        painter.setPen(QColor("#8fa2b2"))
        status = "实际姿态" if self.online else "离线 · 保留最后姿态"
        painter.drawText(self.rect().adjusted(10, 8, -10, -8), Qt.AlignLeft | Qt.AlignTop, status)

    def mousePressEvent(self, event) -> None:
        if event.button() == Qt.LeftButton:
            self.selected.emit()
        super().mousePressEvent(event)

    def mouseMoveEvent(self, event) -> None:
        super().mouseMoveEvent(event)

    def mouseReleaseEvent(self, event) -> None:
        self._drag_pos = None
        self.unsetCursor()
        super().mouseReleaseEvent(event)

    def wheelEvent(self, event) -> None:
        self.zoom = max(0.7, min(1.45, self.zoom + event.angleDelta().y() / 1200.0))
        self.update()
        event.accept()
