"""
servo_panel.py — Unified servo control GUI.
Combines a JointState slider (like joint_state_publisher_gui) and
preset action buttons in one PyQt5 window. Publishes to /joint_states
and /servo_action so servo_node can drive the Arduino.
"""
import sys
import math
import threading
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import String, Header

from PyQt5.QtWidgets import (
    QApplication, QWidget, QVBoxLayout, QHBoxLayout,
    QPushButton, QLabel, QSlider, QGroupBox, QSizePolicy
)
from PyQt5.QtCore import Qt, QTimer, pyqtSignal, QObject
from PyQt5.QtGui import QFont


# ── ROS node ────────────────────────────────────────────────────────────────
class ServoPanelNode(Node):
    JOINT_NAME = 'servo_joint'
    MIN_RAD = 0.0
    MAX_RAD = math.pi  # 0–180°

    def __init__(self):
        super().__init__('servo_panel_node')
        self._js_pub = self.create_publisher(JointState, '/joint_states', 10)
        self._act_pub = self.create_publisher(String, '/servo_action', 10)
        self._angle_rad = math.pi / 2  # start at 90°
        # Publish joint state at 20 Hz so servo_node always has fresh data
        self.create_timer(0.05, self._publish_joint_state)
        self.get_logger().info('Servo Panel Node ready')

    def set_angle_rad(self, radians: float):
        self._angle_rad = max(self.MIN_RAD, min(self.MAX_RAD, radians))
        self._publish_joint_state()

    def _publish_joint_state(self):
        msg = JointState()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = [self.JOINT_NAME]
        msg.position = [self._angle_rad]
        self._js_pub.publish(msg)

    def send_action(self, action: str):
        msg = String()
        msg.data = action
        self._act_pub.publish(msg)
        self.get_logger().info(f'Action: {action}')


# ── Signals helper (thread → Qt main thread) ────────────────────────────────
class Signals(QObject):
    set_slider = pyqtSignal(int)   # degrees
    set_status = pyqtSignal(str, str)  # text, colour


# ── Main GUI window ──────────────────────────────────────────────────────────
class ServoPanelGUI(QWidget):
    def __init__(self, node: ServoPanelNode):
        super().__init__()
        self.node = node
        self._busy = False
        self._stop = False
        self.signals = Signals()
        self.signals.set_slider.connect(self._apply_slider)
        self.signals.set_status.connect(self._apply_status)
        self._setup_ui()

    # ── UI construction ──────────────────────────────────────────────────
    def _setup_ui(self):
        self.setWindowTitle('Servo Control Panel')
        self.setMinimumWidth(400)
        self.setStyleSheet("""
            QWidget {
                background-color: #1e1e2e;
                color: #cdd6f4;
                font-family: 'Segoe UI', Arial, sans-serif;
            }
            QGroupBox {
                border: 1px solid #45475a;
                border-radius: 8px;
                margin-top: 12px;
                padding: 10px 8px 8px 8px;
                font-weight: bold;
                color: #89b4fa;
                font-size: 12px;
            }
            QGroupBox::title {
                subcontrol-origin: margin;
                left: 12px;
                padding: 0 4px;
            }
            QPushButton {
                background-color: #313244;
                color: #cdd6f4;
                border: none;
                border-radius: 6px;
                padding: 9px 0;
                font-size: 12px;
            }
            QPushButton:hover   { background-color: #45475a; }
            QPushButton:pressed { background-color: #585b70; }
            QPushButton:disabled { background-color: #181825; color: #585b70; }
            QSlider::groove:horizontal {
                height: 6px;
                background: #45475a;
                border-radius: 3px;
            }
            QSlider::handle:horizontal {
                background: #89b4fa;
                width: 18px;
                height: 18px;
                border-radius: 9px;
                margin: -6px 0;
            }
            QSlider::sub-page:horizontal {
                background: #89b4fa;
                border-radius: 3px;
            }
        """)

        root = QVBoxLayout(self)
        root.setSpacing(12)
        root.setContentsMargins(16, 16, 16, 16)

        # Title
        title = QLabel('🎛  Servo Control Panel')
        title.setFont(QFont('Arial', 14, QFont.Bold))
        title.setAlignment(Qt.AlignCenter)
        title.setStyleSheet('color:#cdd6f4; margin-bottom:2px;')
        root.addWidget(title)

        # ── Slider group ──────────────────────────────────────────────
        slider_group = QGroupBox('Joint: servo_joint   (0° – 180°)')
        sg = QVBoxLayout(slider_group)
        sg.setSpacing(6)

        self.angle_label = QLabel('Angle: 90°')
        self.angle_label.setAlignment(Qt.AlignCenter)
        self.angle_label.setStyleSheet('color:#89b4fa; font-size:13px;')
        sg.addWidget(self.angle_label)

        self.slider = QSlider(Qt.Horizontal)
        self.slider.setMinimum(0)
        self.slider.setMaximum(180)
        self.slider.setValue(90)
        self.slider.setTickPosition(QSlider.TicksBelow)
        self.slider.setTickInterval(45)
        self.slider.valueChanged.connect(self._on_slider)
        sg.addWidget(self.slider)

        # Min / Max labels
        minmax = QHBoxLayout()
        minmax.addWidget(QLabel('0°'))
        minmax.addStretch()
        minmax.addWidget(QLabel('90°'))
        minmax.addStretch()
        minmax.addWidget(QLabel('180°'))
        sg.addLayout(minmax)

        root.addWidget(slider_group)

        # ── Quick-set buttons ─────────────────────────────────────────
        preset_group = QGroupBox('Quick Position')
        pg = QHBoxLayout(preset_group)
        pg.setSpacing(8)
        for label, deg in [('0°', 0), ('45°', 45), ('90°', 90), ('135°', 135), ('180°', 180)]:
            btn = QPushButton(label)
            btn.clicked.connect(lambda _, d=deg: self._goto(d))
            pg.addWidget(btn)
        root.addWidget(preset_group)

        # ── Preset motion buttons ─────────────────────────────────────
        actions_group = QGroupBox('Preset Motions')
        ag = QVBoxLayout(actions_group)
        ag.setSpacing(8)

        self._action_buttons = []
        motion_actions = [
            ('⚡  Zigzag  (0° ↔ 180°)',   'zigzag',    '#f9e2af'),
            ('🔄  Full Sweep',             'sweep',     '#89b4fa'),
            ('⚙️   Slow Sweep',             'slow_sweep','#cba6f7'),
            ('🌊  Wave  (45° ↔ 135°)',    'wave',      '#94e2d5'),
        ]

        # Two columns
        col_layout = QHBoxLayout()
        col_left  = QVBoxLayout()
        col_right = QVBoxLayout()
        for i, (lbl, act, col) in enumerate(motion_actions):
            btn = QPushButton(lbl)
            btn.setStyleSheet(
                f'QPushButton {{color:{col}; background:#313244;}}'
                f'QPushButton:hover {{background:#45475a;}}'
                f'QPushButton:disabled {{color:#585b70;}}'
            )
            btn.clicked.connect(lambda _, a=act: self._run_action(a))
            self._action_buttons.append(btn)
            (col_left if i % 2 == 0 else col_right).addWidget(btn)
        col_layout.addLayout(col_left)
        col_layout.addLayout(col_right)
        ag.addLayout(col_layout)

        root.addWidget(actions_group)

        # ── Status + Stop ─────────────────────────────────────────────
        bottom = QHBoxLayout()
        self.status_label = QLabel('Ready')
        self.status_label.setStyleSheet('color:#a6e3a1; font-size:11px;')
        bottom.addWidget(self.status_label)
        bottom.addStretch()

        stop_btn = QPushButton('■  STOP')
        stop_btn.setFixedWidth(90)
        stop_btn.setStyleSheet(
            'QPushButton {background:#f38ba8; color:#1e1e2e; font-weight:bold; border-radius:6px;}'
            'QPushButton:hover {background:#eba0ac;}'
        )
        stop_btn.clicked.connect(self._stop_action)
        bottom.addWidget(stop_btn)
        root.addLayout(bottom)

        self.adjustSize()

    # ── Slot helpers (called from main thread via signals) ───────────────
    def _apply_slider(self, deg: int):
        self.slider.setValue(deg)

    def _apply_status(self, text: str, colour: str):
        self.status_label.setText(text)
        self.status_label.setStyleSheet(f'color:{colour}; font-size:11px;')
        for btn in self._action_buttons:
            btn.setEnabled(colour != '#f38ba8')  # disable while busy

    # ── UI callbacks ─────────────────────────────────────────────────────
    def _on_slider(self, deg: int):
        self.angle_label.setText(f'Angle: {deg}°')
        self.node.set_angle_rad(math.radians(deg))

    def _goto(self, deg: int):
        self.slider.setValue(deg)  # triggers _on_slider

    def _stop_action(self):
        self._stop = True
        self.node.send_action('stop')
        self.signals.set_status.emit('Stopped', '#fab387')

    def _run_action(self, action: str):
        if self._busy:
            return
        self._stop = False
        self._busy = True
        self.signals.set_status.emit(f'▶ {action}…', '#f38ba8')
        threading.Thread(target=self._execute, args=(action,), daemon=True).start()

    def _execute(self, action: str):
        try:
            if action == 'zigzag':
                for _ in range(6):
                    if self._stop: break
                    self._move_to(0);   time.sleep(0.5)
                    if self._stop: break
                    self._move_to(180); time.sleep(0.5)

            elif action == 'sweep':
                for d in list(range(0, 181, 3)) + list(range(180, -1, -3)):
                    if self._stop: break
                    self._move_to(d); time.sleep(0.025)

            elif action == 'slow_sweep':
                for d in list(range(0, 181, 1)) + list(range(180, -1, -1)):
                    if self._stop: break
                    self._move_to(d); time.sleep(0.06)

            elif action == 'wave':
                for _ in range(5):
                    for d in list(range(45, 136, 3)) + list(range(135, 44, -3)):
                        if self._stop: break
                        self._move_to(d); time.sleep(0.03)
        finally:
            self._busy = False
            if not self._stop:
                self.signals.set_status.emit('Done ✓', '#a6e3a1')

    def _move_to(self, deg: int):
        """Set angle from background thread — update node immediately, slider via signal."""
        self.node.set_angle_rad(math.radians(deg))
        self.signals.set_slider.emit(deg)


# ── Entry point ──────────────────────────────────────────────────────────────
def main(args=None):
    rclpy.init(args=args)
    node = ServoPanelNode()

    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    app = QApplication(sys.argv)
    gui = ServoPanelGUI(node)
    gui.show()
    app.exec_()

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
