"""
rqt plugin: Face Lifting reachable sets (fl_reach/HyperRectangle), drawn as rectangles in a 2D
projection (default x-y, axes selectable) or as cuboids in 3D, with the trail of the agent's pose.
UI, topics and trails come from visualization.base_viz.ReachVizBase.

    rqt                                                     # Plugins -> Visualization -> Reachable Set Viewer (Face Lifting)
    ros2 run fl_reach reach_viz       # standalone
"""
import numpy as np
from python_qt_binding.QtCore import QPointF, QRectF, Qt
from python_qt_binding.QtGui import QColor, QPen, QPolygonF

from visualization.base_viz import AXES, ReachVizBase

# corner index = bit0 -> x, bit1 -> y, bit2 -> z (0 = min, 1 = max)
FACES = ((0, 1, 3, 2), (4, 5, 7, 6), (0, 1, 5, 4), (2, 3, 7, 6), (0, 2, 6, 4), (1, 3, 7, 5))
EDGES = ((0, 1), (2, 3), (4, 5), (6, 7), (0, 2), (1, 3), (4, 6), (5, 7), (0, 4), (1, 5), (2, 6), (3, 7))


def _pen(color, width, stale):
    pen = QPen(color, width)
    if stale:
        pen.setStyle(Qt.DashLine)
    return pen


class ReachViz(ReachVizBase):

    MSG_TYPE = 'fl_reach/msg/HyperRectangle'
    TITLE = 'Reachable Set Viewer (Face Lifting)'

    @staticmethod
    def to_data(msg):
        """-> (6, 2) array of [min, max] per state axis"""
        return np.array([[getattr(msg, a).min, getattr(msg, a).max] for a in AXES])

    @staticmethod
    def project(data, axes):
        """-> (k, 2) [min, max] of the box on the chosen axes"""
        return data[list(axes)]

    @staticmethod
    def points(shape):
        """the 2^k corners of the box; corner i takes max on axis j if bit j of i is set"""
        return np.array([[shape[j, (i >> j) & 1] for j in range(len(shape))] for i in range(2 ** len(shape))])

    @staticmethod
    def draw_2d(p, shape, color, stale, to_px):
        (h0, h1), (v0, v1) = shape
        x0, y1 = to_px(h0, v1)
        x1, y0 = to_px(h1, v0)
        fill = QColor(color)
        fill.setAlpha(25 if stale else 70)
        p.setPen(_pen(color, 2, stale))
        p.setBrush(fill)
        p.drawRect(QRectF(x0, y0, x1 - x0, y1 - y0))

    @staticmethod
    def draw_3d(p, shape, color, stale, project):
        corners = [project(c) for c in ReachViz.points(shape)]
        fill = QColor(color)
        fill.setAlpha(18 if stale else 45)
        p.setPen(Qt.NoPen)
        p.setBrush(fill)
        for face in sorted(FACES, key=lambda f: -sum(corners[i][2] for i in f)):  # farthest face first
            p.drawPolygon(QPolygonF([QPointF(corners[i][0], corners[i][1]) for i in face]))
        p.setPen(_pen(color, 2, stale))
        for a, b in EDGES:
            p.drawLine(int(corners[a][0]), int(corners[a][1]), int(corners[b][0]), int(corners[b][1]))
