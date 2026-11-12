"""
2D and 3D canvases shared by the reachable set viewers. No ROS imports.

The canvases draw the frame (grid, ticks, axis names, 3D axes, legend) and the trails. The sets themselves are drawn by
the renderer, the checker's ReachViz class (see base_viz.ReachVizBase for the hooks points / draw_2d / draw_3d).

entries: list of (label, QColor, shape or None, stale, trail), shape as returned by renderer.project, trail a list of
2 or 3 coordinates (may be empty).
"""
import math

try:
    from python_qt_binding.QtCore import QPointF, Qt
    from python_qt_binding.QtGui import QColor, QFont, QPainter, QPen, QPolygonF
    from python_qt_binding.QtWidgets import QWidget
except ImportError:  # plain PyQt5 outside of rqt (offscreen tests)
    from PyQt5.QtCore import QPointF, Qt
    from PyQt5.QtGui import QColor, QFont, QPainter, QPen, QPolygonF
    from PyQt5.QtWidgets import QWidget

import numpy as np

MIN_SPAN = 10.0  # smallest visible extent per axis, so a single small set does not fill the screen
MARGIN = 40  # px


def nice_step(span, target_ticks=8):
    raw = span / target_ticks
    mag = 10 ** math.floor(math.log10(raw))
    for m in (1, 2, 5, 10):
        if raw <= m * mag:
            return m * mag
    return 10 * mag


def legend(p, entries, x, y, text_color):
    for label, color, shape, stale, _ in entries:
        p.setPen(QPen(color, 3))
        p.drawLine(x, y + 6, x + 18, y + 6)
        p.setPen(QPen(text_color))
        suffix = '  (no data)' if shape is None else '  (stale)' if stale else ''
        p.drawText(x + 24, y, 400, 14, Qt.AlignLeft | Qt.AlignVCenter, label + suffix)
        y += 16


def _fit_points(entries, renderer, k):
    """All points to fit the view on: the sets' points and the trails, (m, k), None if there is nothing"""
    pts = [np.asarray(renderer.points(e[2]), float).reshape(-1, k) for e in entries if e[2] is not None]
    pts += [np.asarray(e[4], float).reshape(-1, k) for e in entries if e[2] is not None and len(e[4])]
    return np.vstack(pts) if pts else None


class Canvas2D(QWidget):
    """Equal-aspect view, auto-fitted to the sets and trails. Horizontal axis to the right, vertical axis up."""

    def __init__(self, renderer, parent=None):
        super().__init__(parent)
        self.setMinimumSize(320, 320)
        self.renderer = renderer
        self.entries = []
        self.h_name, self.v_name = 'x', 'y'

    def set_entries(self, entries, h_name, v_name):
        self.entries, self.h_name, self.v_name = entries, h_name, v_name
        self.update()

    def _bounds(self):
        pts = _fit_points(self.entries, self.renderer, 2)
        if pts is None:
            return -MIN_SPAN / 2, MIN_SPAN / 2, -MIN_SPAN / 2, MIN_SPAN / 2
        (h0, v0), (h1, v1) = pts.min(axis=0), pts.max(axis=0)
        pad_h = max(max(MIN_SPAN - (h1 - h0), 0) / 2, 0.05 * (h1 - h0))
        pad_v = max(max(MIN_SPAN - (v1 - v0), 0) / 2, 0.05 * (v1 - v0))
        return h0 - pad_h, h1 + pad_h, v0 - pad_v, v1 + pad_v

    def paintEvent(self, _event):
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing)
        p.fillRect(self.rect(), self.palette().base())
        w, h = self.width() - 2 * MARGIN, self.height() - 2 * MARGIN
        if w <= 0 or h <= 0:
            return

        h0, h1, v0, v1 = self._bounds()
        scale = min(w / (h1 - h0), h / (v1 - v0))  # px per unit, equal aspect
        cx, cy = (h0 + h1) / 2, (v0 + v1) / 2
        ox, oy = MARGIN + w / 2, MARGIN + h / 2

        def to_px(a, b):
            return ox + (a - cx) * scale, oy - (b - cy) * scale

        # visible data range after the equal-aspect fit
        vis_h0, vis_h1 = cx - w / 2 / scale, cx + w / 2 / scale
        vis_v0, vis_v1 = cy - h / 2 / scale, cy + h / 2 / scale

        # grid + tick labels
        grid_pen = QPen(QColor(128, 128, 128, 70))
        text_pen = QPen(self.palette().text().color())
        p.setFont(QFont(self.font().family(), 8))
        step = nice_step(max(vis_h1 - vis_h0, vis_v1 - vis_v0))
        t = math.ceil(vis_h0 / step) * step
        while t <= vis_h1:
            x, _ = to_px(t, cy)
            p.setPen(grid_pen)
            p.drawLine(int(x), MARGIN, int(x), MARGIN + int(h))
            p.setPen(text_pen)
            p.drawText(int(x) - 25, self.height() - MARGIN + 14, 50, 14, Qt.AlignCenter, f'{t:g}')
            t += step
        t = math.ceil(vis_v0 / step) * step
        while t <= vis_v1:
            _, y = to_px(cx, t)
            p.setPen(grid_pen)
            p.drawLine(MARGIN, int(y), MARGIN + int(w), int(y))
            p.setPen(text_pen)
            p.drawText(2, int(y) - 7, MARGIN - 6, 14, Qt.AlignRight | Qt.AlignVCenter, f'{t:g}')
            t += step
        p.setPen(text_pen)
        p.drawText(0, self.height() - 16, self.width(), 14, Qt.AlignCenter, self.h_name)
        p.save()
        p.translate(12, self.height() / 2)
        p.rotate(-90)
        p.drawText(-30, -4, 60, 14, Qt.AlignCenter, self.v_name)
        p.restore()

        # sets + trails
        p.setClipRect(MARGIN, MARGIN, int(w), int(h))
        for _label, color, shape, stale, trail in self.entries:
            if shape is None:
                continue
            p.save()
            self.renderer.draw_2d(p, shape, color, stale, to_px)
            p.restore()
            if len(trail):
                p.setPen(QPen(color, 2))
                p.setBrush(Qt.NoBrush)
                p.drawPolyline(QPolygonF([QPointF(*to_px(a, b)) for a, b in trail]))
        p.setClipping(False)

        # frame + legend
        p.setPen(text_pen)
        p.setBrush(Qt.NoBrush)
        p.drawRect(MARGIN, MARGIN, int(w), int(h))
        legend(p, self.entries, MARGIN + 8, MARGIN + 6, self.palette().text().color())


class Canvas3D(QWidget):
    """
    Orthographic 3D view (x east, y north, z up, equal scale on all axes), auto-fitted to the sets and trails.
    Drag with the left mouse button to rotate, wheel to zoom, double click to reset the view.
    """

    DEFAULT_YAW = -60.0   # deg, rotation about z
    DEFAULT_PITCH = 25.0  # deg, elevation above the horizon

    def __init__(self, renderer, parent=None):
        super().__init__(parent)
        self.setMinimumSize(320, 320)
        self.renderer = renderer
        self.entries = []
        self.yaw, self.pitch, self.zoom = self.DEFAULT_YAW, self.DEFAULT_PITCH, 1.0
        self._last = None

    def set_entries(self, entries):
        self.entries = entries
        self.update()

    # ---- interaction ----
    def mousePressEvent(self, event):
        self._last = event.pos()

    def mouseMoveEvent(self, event):
        if self._last is None:
            return
        d = event.pos() - self._last
        self._last = event.pos()
        self.yaw += d.x() * 0.5
        self.pitch = max(-89.0, min(89.0, self.pitch + d.y() * 0.5))
        self.update()

    def mouseReleaseEvent(self, _event):
        self._last = None

    def wheelEvent(self, event):
        self.zoom = max(0.2, min(10.0, self.zoom * (1.1 if event.angleDelta().y() > 0 else 1 / 1.1)))
        self.update()

    def mouseDoubleClickEvent(self, _event):
        self.yaw, self.pitch, self.zoom = self.DEFAULT_YAW, self.DEFAULT_PITCH, 1.0
        self.update()

    # ---- drawing ----
    def paintEvent(self, _event):
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing)
        p.fillRect(self.rect(), self.palette().base())
        pts = _fit_points(self.entries, self.renderer, 3)
        lo, hi = (pts.min(axis=0), pts.max(axis=0)) if pts is not None else (np.zeros(3), np.zeros(3))
        pad = np.maximum(MIN_SPAN - (hi - lo), 0) / 2
        lo, hi = lo - pad, hi + pad
        ctr = (lo + hi) / 2
        extent = float(np.max(hi - lo))
        scale = self.zoom * 0.7 * min(self.width(), self.height()) / extent
        cx, cy = self.width() / 2, self.height() / 2
        ya, pa = math.radians(self.yaw), math.radians(self.pitch)
        cyaw, syaw, cp, sp = math.cos(ya), math.sin(ya), math.cos(pa), math.sin(pa)

        def project(pt):
            """-> (screen x, screen y, depth); larger depth = farther from the viewer"""
            x, y, z = pt[0] - ctr[0], pt[1] - ctr[1], pt[2] - ctr[2]
            xr, yr = cyaw * x - syaw * y, syaw * x + cyaw * y
            return cx + xr * scale, cy - (z * cp - yr * sp) * scale, yr * cp + z * sp

        text_color = self.palette().text().color()

        # ground grid at the lowest z, plus axis arrows from the min corner
        step = nice_step(extent)
        p.setPen(QPen(QColor(128, 128, 128, 70)))
        for axis, other in ((0, 1), (1, 0)):
            t = math.ceil(lo[axis] / step) * step
            while t <= hi[axis]:
                a, b = np.array(lo), np.array(lo)
                a[axis] = b[axis] = t
                b[other] = hi[other]
                pa_, pb_ = project(a), project(b)
                p.drawLine(int(pa_[0]), int(pa_[1]), int(pb_[0]), int(pb_[1]))
                t += step
        length = max(step, extent * 0.25)
        for name, d, color in (('x', (1, 0, 0), '#D55E00'), ('y', (0, 1, 0), '#009E73'), ('z', (0, 0, 1), '#0072B2')):
            a, b = project(lo), project(lo + length * np.array(d))
            p.setPen(QPen(QColor(color), 2))
            p.drawLine(int(a[0]), int(a[1]), int(b[0]), int(b[1]))
            p.drawText(int(b[0]) + 4, int(b[1]) - 4, f'{name} (+{length:g} m)')

        # sets farthest first (by the centroid of their points), each followed by its trail
        drawn = [e for e in self.entries if e[2] is not None]
        drawn.sort(key=lambda e: -project(np.asarray(self.renderer.points(e[2]), float).reshape(-1, 3).mean(axis=0))[2])
        for _label, color, shape, stale, trail in drawn:
            p.save()
            self.renderer.draw_3d(p, shape, color, stale, project)
            p.restore()
            if len(trail):
                p.setPen(QPen(color, 2))
                p.setBrush(Qt.NoBrush)
                p.drawPolyline(QPolygonF([QPointF(*project(v)[:2]) for v in trail]))

        p.setFont(QFont(self.font().family(), 8))
        legend(p, self.entries, 10, 8, text_color)
        p.setPen(QPen(text_color))
        p.drawText(10, self.height() - 18, 'drag: rotate   wheel: zoom   double click: reset')
