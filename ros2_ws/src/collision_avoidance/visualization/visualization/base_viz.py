"""
Base rqt plugin of the reachable set viewers: user interface, topic selection, pose trails, settings.

A checker's viewer (<checker package>/visualization/viz.py, class ReachViz) derives from ReachVizBase, sets MSG_TYPE
and TITLE and implements how its reachable set is converted, projected and drawn:

    to_data(msg)                                    executor thread: message -> plain data (kept as the latest set)
    project(data, axes)                -> shape     the set on state axes (indices into AXES), 2 in 2D, 3 in 3D
    points(shape)                      -> (m, k)    points the view is fitted to
    draw_2d(p, shape, color, stale, to_px)          QPainter p, to_px(a, b) -> screen (x, y)
    draw_3d(p, shape, color, stale, project)        QPainter p, project(pt) -> screen (x, y, depth), larger = farther

All hooks are static (no plugin state), the canvases call them on the class.

Pick a reachable set topic (e.g. /drone1/my_rs or /drone2/received_rs, only topics of MSG_TYPE are listed) and a pose
topic (e.g. /drone1/self_localization/pose, or (none)) and press Add. Every set is drawn as received, with the trail of
its pose topic over the last TRAIL_SEC. Every added topic gets its own color.

    rqt                                     # Plugins -> Visualization -> Reachable Set Viewer (<checker>)
    ros2 run <checker package> reach_viz    # standalone
"""
import threading
import time
from collections import deque

from geometry_msgs.msg import PoseStamped
from python_qt_binding.QtCore import Qt, QTimer
from python_qt_binding.QtGui import QColor
from python_qt_binding.QtWidgets import (
    QComboBox, QHBoxLayout, QLabel, QListWidget, QListWidgetItem, QPushButton, QVBoxLayout, QWidget)
from qt_gui.plugin import Plugin
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy, qos_profile_sensor_data
from rosidl_runtime_py.utilities import get_message

from .canvas import Canvas2D, Canvas3D

AXES = ('x', 'y', 'z', 'vx', 'vy', 'vz')  # state axes, the order of the projection indices
POSE_TYPE = 'geometry_msgs/msg/PoseStamped'
NO_POSE = '(none)'
STALE_AFTER = 1.0  # s without a message before a set is drawn as stale
REDRAW_MS = 100
TRAIL_SEC = 10.0  # s of past positions drawn as the trail
# Color-blind friendly cycle (Okabe-Ito)
COLORS = ['#0072B2', '#D55E00', '#009E73', '#CC79A7', '#E69F00', '#56B4E9', '#F0E442', '#000000']


def _label(topic):
    """/drone1/my_rs (set + pose trail) -> 'drone1 reachability traj', other topics as they are"""
    ns, name = topic.rsplit('/', 1) if '/' in topic else ('', topic)
    return f'{ns.strip("/")} reachability traj' if name == 'my_rs' else topic


class ReachVizBase(Plugin):

    MSG_TYPE = None  # e.g. 'probstar_reach/msg/ProbStar'
    TITLE = 'Reachable Set Viewer'

    # ---- hooks, implemented by the checker ----
    @staticmethod
    def to_data(msg):
        raise NotImplementedError

    @staticmethod
    def project(data, axes):
        raise NotImplementedError

    @staticmethod
    def points(shape):
        raise NotImplementedError

    @staticmethod
    def draw_2d(p, shape, color, stale, to_px):
        raise NotImplementedError

    @staticmethod
    def draw_3d(p, shape, color, stale, project):
        raise NotImplementedError

    # ---- plugin ----
    def __init__(self, context):
        super().__init__(context)
        self.setObjectName(type(self).__name__)
        self._node = context.node
        self._msg_class = get_message(self.MSG_TYPE)
        self._lock = threading.Lock()
        self._subs, self._latest, self._colors = {}, {}, {}  # topic -> subscription, (data, time, id), QColor
        self._pose_subs, self._trails = {}, {}  # topic -> pose subscription, deque of (time, [x, y, z])
        self._next_color = 0

        self._widget = QWidget()
        self._widget.setObjectName(type(self).__name__ + 'Ui')
        self._widget.setWindowTitle(self.TITLE)

        self._topic_box, self._pose_box = QComboBox(), QComboBox()
        for box in (self._topic_box, self._pose_box):
            box.setEditable(True)  # lets you type a topic that is not discovered yet
            box.setMinimumWidth(240)
        refresh, add, remove = QPushButton('Refresh'), QPushButton('Add'), QPushButton('Remove')
        self._h_box, self._v_box = QComboBox(), QComboBox()
        for box, default in ((self._h_box, 'x'), (self._v_box, 'y')):
            box.addItems(AXES)
            box.setCurrentText(default)
        self._list = QListWidget()
        self._list.setMaximumHeight(110)
        self._view_box = QComboBox()
        self._view_box.addItems(['2D', '3D'])
        self._canvas, self._canvas3d = Canvas2D(type(self)), Canvas3D(type(self))
        self._canvas3d.hide()

        rs_row, pose_row, button_row = QHBoxLayout(), QHBoxLayout(), QHBoxLayout()
        rs_row.addWidget(QLabel('RS topic:'))
        rs_row.addWidget(self._topic_box, 1)
        pose_row.addWidget(QLabel('Pose topic:'))
        pose_row.addWidget(self._pose_box, 1)
        for b in (refresh, add, remove):
            button_row.addWidget(b)
        button_row.addStretch(1)  # keep the buttons at their natural size
        axes = QHBoxLayout()
        for text, w in (('View:', self._view_box), ('Horizontal:', self._h_box), ('Vertical:', self._v_box)):
            axes.addWidget(QLabel(text))
            axes.addWidget(w)
        axes.addStretch(1)
        layout = QVBoxLayout(self._widget)
        for row in (rs_row, pose_row, button_row):
            layout.addLayout(row)
        layout.addWidget(self._list)
        layout.addLayout(axes)
        layout.addWidget(self._canvas, 1)
        layout.addWidget(self._canvas3d, 1)

        refresh.clicked.connect(self._refresh_topics)
        self._topic_box.currentTextChanged.connect(self._match_pose)
        add.clicked.connect(self._add_current)
        remove.clicked.connect(self._remove_selected)
        self._view_box.currentTextChanged.connect(self._on_view_changed)
        self._on_view_changed('2D')

        context.add_widget(self._widget)
        self._refresh_topics()
        self._timer = QTimer(self._widget)
        self._timer.timeout.connect(self._redraw)
        self._timer.start(REDRAW_MS)

    def _on_view_changed(self, view):
        is_3d = view == '3D'
        self._canvas3d.setVisible(is_3d)
        self._canvas.setVisible(not is_3d)
        self._h_box.setEnabled(not is_3d)
        self._v_box.setEnabled(not is_3d)

    # ---- topics ----
    def _refresh_topics(self):
        names_types = self._node.get_topic_names_and_types()
        for box, msg_type, extra in ((self._topic_box, self.MSG_TYPE, []), (self._pose_box, POSE_TYPE, [NO_POSE])):
            current = box.currentText()
            box.blockSignals(True)  # keep the pose the user picked, no _match_pose on refresh
            box.clear()
            box.addItems(extra + sorted(n for n, types in names_types if msg_type in types))
            if current:
                box.setEditText(current)
            box.blockSignals(False)
        if self._pose_box.currentText() in ('', NO_POSE):  # nothing picked yet, e.g. the first RS topic on startup
            self._match_pose(self._topic_box.currentText())

    def _match_pose(self, topic):
        """Pick <namespace>/self_localization/pose for the chosen RS topic (the verifier takes its position from it)"""
        ns, name = topic.strip().rsplit('/', 1) if '/' in topic else ('', topic)
        # received_rs lives in the receiver's namespace, that pose is not the sender's, so leave it to the user
        pose = f'{ns}/self_localization/pose'
        if name != 'received_rs' and self._pose_box.findText(pose) >= 0:
            self._pose_box.setCurrentText(pose)
        else:
            self._pose_box.setCurrentText(NO_POSE)

    def _add_current(self):
        topic = self._topic_box.currentText().strip()
        pose_topic = self._pose_box.currentText().strip()
        if not topic or topic in self._subs:
            return
        # reliable + volatile matches both transient_local (my_rs) and default (received_rs) publishers
        qos = QoSProfile(depth=1, reliability=QoSReliabilityPolicy.RELIABLE, durability=QoSDurabilityPolicy.VOLATILE)
        self._subs[topic] = self._node.create_subscription(
            self._msg_class, topic, lambda msg, t=topic: self._on_msg(t, msg), qos)
        if pose_topic and pose_topic != NO_POSE:
            # self_localization/pose is published best effort
            self._pose_subs[topic] = self._node.create_subscription(
                PoseStamped, pose_topic, lambda msg, t=topic: self._on_pose(t, msg), qos_profile_sensor_data)
        self._colors[topic] = QColor(COLORS[self._next_color % len(COLORS)])
        self._next_color += 1
        item = QListWidgetItem(_label(topic))
        item.setData(Qt.UserRole, topic)
        item.setForeground(self._colors[topic])
        self._list.addItem(item)

    def _remove_selected(self):
        for item in self._list.selectedItems():
            topic = item.data(Qt.UserRole)
            self._node.destroy_subscription(self._subs.pop(topic))
            if topic in self._pose_subs:
                self._node.destroy_subscription(self._pose_subs.pop(topic))
            with self._lock:
                self._latest.pop(topic, None)
                self._trails.pop(topic, None)
            self._colors.pop(topic, None)
            self._list.takeItem(self._list.row(item))

    # ---- data ----
    def _on_msg(self, topic, msg):  # executor thread
        data = self.to_data(msg)
        with self._lock:
            self._latest[topic] = (data, time.monotonic(), msg.id)

    def _on_pose(self, topic, msg):  # executor thread
        p = msg.pose.position
        with self._lock:
            now = time.monotonic()
            trail = self._trails.setdefault(topic, deque())
            trail.append((now, [p.x, p.y, p.z]))
            while now - trail[0][0] > TRAIL_SEC:
                trail.popleft()

    def _redraw(self):  # GUI thread
        is_3d = self._view_box.currentText() == '3D'
        axes = (0, 1, 2) if is_3d else (AXES.index(self._h_box.currentText()), AXES.index(self._v_box.currentText()))
        now = time.monotonic()
        with self._lock:
            latest = dict(self._latest)
            trails = {t: [pos for _, pos in tr] for t, tr in self._trails.items()}
        entries = []
        for topic, color in self._colors.items():
            if topic in latest:
                data, stamp, agent_id = latest[topic]
                # the pose has no velocity, so there is no trail on the vx/vy/vz axes
                trail = [[pos[i] for i in axes] for pos in trails.get(topic, [])] if max(axes) < 3 else []
                entries.append((f'{_label(topic)} (id {agent_id})', color, self.project(data, axes),
                                now - stamp > STALE_AFTER, trail))
            else:
                entries.append((_label(topic), color, None, True, []))
        if is_3d:
            self._canvas3d.set_entries(entries)
        else:
            self._canvas.set_entries(entries, self._h_box.currentText(), self._v_box.currentText())

    # ---- plugin lifecycle ----
    def shutdown_plugin(self):
        self._timer.stop()
        for sub in [*self._subs.values(), *self._pose_subs.values()]:
            self._node.destroy_subscription(sub)
        self._subs.clear()
        self._pose_subs.clear()

    def save_settings(self, plugin_settings, instance_settings):
        # rs|pose pairs, pose empty for (none)
        instance_settings.set_value('topics', ','.join(
            f'{t}|{self._pose_subs[t].topic_name if t in self._pose_subs else ""}' for t in self._subs))
        instance_settings.set_value('view', self._view_box.currentText())
        instance_settings.set_value('h', self._h_box.currentText())
        instance_settings.set_value('v', self._v_box.currentText())

    def restore_settings(self, plugin_settings, instance_settings):
        if instance_settings.value('view') in ('3D', '2D'):
            self._view_box.setCurrentText(instance_settings.value('view'))
        for key, box in (('h', self._h_box), ('v', self._v_box)):
            if instance_settings.value(key) in AXES:
                box.setCurrentText(instance_settings.value(key))
        saved = instance_settings.value('topics')
        for pair in (saved.split(',') if saved else []):
            topic, _, pose_topic = pair.partition('|')
            if topic:
                self._topic_box.setEditText(topic)
                self._pose_box.setEditText(pose_topic or NO_POSE)
                self._add_current()
