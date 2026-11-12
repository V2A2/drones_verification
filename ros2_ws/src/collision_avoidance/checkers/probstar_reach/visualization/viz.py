"""
rqt plugin: ProbStar tubes (probstar_reach/ProbStar) as a 2D projection (default x-y, axes selectable) or in 3D, with
the trail of the agent's pose. Each tube shows the support (filled, later steps lighter) of every step and the 1/2/3
sigma contours of the last step's Gaussian. Every star is drawn exactly as received, no propagation on the viewer side.
UI, topics and trails come from visualization.base_viz.ReachVizBase.

    rqt                                     # Plugins -> Visualization -> Reachable Set Viewer (ProbStar)
    ros2 run probstar_reach reach_viz       # standalone
"""
import numpy as np
from python_qt_binding.QtCore import QPointF, Qt
from python_qt_binding.QtGui import QColor, QPen, QPolygonF

from visualization.base_viz import ReachVizBase

from .geometry import ellipse_2d, ellipsoid_circles, projection, zonotope_faces, zonotope_polygon

SIGMAS = (1, 2, 3)


def _fade(i, n):
    """Opacity factor of tube step i of n: 1 for the first step down to 0.3 for the last (later = lighter)"""
    return 1 - 0.7 * i / max(n - 1, 1)


def _support_style(p, color, stale, i, n, fill_alpha, width):
    fade = _fade(i, n)
    fill = QColor(color)
    fill.setAlpha(int(fill_alpha * fade))
    edge = QColor(color)
    edge.setAlpha(int(255 * fade))
    pen = QPen(edge, width)
    if stale:
        pen.setStyle(Qt.DashLine)
    p.setPen(pen)
    p.setBrush(fill)


class ReachViz(ReachVizBase):

    MSG_TYPE = 'probstar_reach/msg/ProbStar'
    TITLE = 'Reachable Set Viewer (ProbStar)'

    @staticmethod
    def to_data(msg):
        """-> (mean, std), each (N, 6): one pair of 6 values per tube step"""
        return np.reshape(msg.mean, (-1, 6)), np.reshape(msg.std, (-1, 6))

    @staticmethod
    def project(data, axes):
        """
        -> (steps, supports): steps [(center (k,), G (k, 6))] per tube step, supports the support of every step,
        a polygon (h, 2) in 2D or a list of faces (4+, 3) in 3D
        """
        steps = [projection(m, s, axes) for m, s in zip(*data)]
        support = zonotope_polygon if len(axes) == 2 else zonotope_faces
        return steps, [support(*step) for step in steps]

    @staticmethod
    def points(shape):
        steps, supports = shape
        if not steps:
            return np.empty((0, 2))
        k = len(steps[0][0])
        return np.vstack([np.reshape(s, (-1, k)) for s in supports])

    @staticmethod
    def draw_2d(p, shape, color, stale, to_px):
        steps, supports = shape
        if not steps:
            return

        def poly(pts):
            return QPolygonF([QPointF(*to_px(a, b)) for a, b in pts])

        for i, support in enumerate(supports):
            _support_style(p, color, stale, i, len(supports), 20 if stale else 45, 2)
            p.drawPolygon(poly(support))
        p.setBrush(Qt.NoBrush)
        center, G = steps[-1]
        for k in SIGMAS:  # inner rings are drawn heavier: more probability mass
            ring = QColor(color)
            ring.setAlpha(60 if stale else 255 - 60 * (k - 1))
            p.setPen(QPen(ring, 1))
            p.drawPolygon(poly(ellipse_2d(center, G, k)))
        p.setPen(QPen(color, 1))
        cxp, cyp = to_px(*center)
        p.drawLine(int(cxp) - 4, int(cyp), int(cxp) + 4, int(cyp))
        p.drawLine(int(cxp), int(cyp) - 4, int(cxp), int(cyp) + 4)

    @staticmethod
    def draw_3d(p, shape, color, stale, project):
        steps, supports = shape
        if not steps:
            return
        for i, faces in enumerate(supports):
            _support_style(p, color, stale, i, len(supports), 14 if stale else 35, 1.5)
            for face in sorted(faces, key=lambda f: -sum(project(v)[2] for v in f)):  # farthest face first
                p.drawPolygon(QPolygonF([QPointF(*project(v)[:2]) for v in face]))
        p.setBrush(Qt.NoBrush)
        center, G = steps[-1]
        for k in SIGMAS:
            ring = QColor(color)
            ring.setAlpha(50 if stale else 255 - 70 * (k - 1))
            p.setPen(QPen(ring, 1))
            for circle in ellipsoid_circles(center, G, k):
                p.drawPolyline(QPolygonF([QPointF(*project(v)[:2]) for v in circle]))
