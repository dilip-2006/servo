import sys
import threading
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from PyQt5.QtWidgets import (
    QApplication, QWidget, QVBoxLayout, QHBoxLayout,
    QPushButton, QLabel, QProgressBar, QGroupBox
)
from PyQt5.QtCore import Qt, QTimer
from PyQt5.QtGui import QFont


class ServoActionsNode(Node):
    def __init__(self):
        super().__init__('servo_actions_node')
        self.pub = self.create_publisher(String, '/servo_action', 10)
        self.get_logger().info('Servo Actions Node ready — publishing to /servo_action')

    def send_action(self, action: str):
        msg = String()
        msg.data = action
        self.pub.publish(msg)
        self.get_logger().info(f'Sent action: {action}')


class ServoActionsGUI(QWidget):
    def __init__(self, node: ServoActionsNode):
        super().__init__()
        self.node = node
        self._busy = False
        self._stop = False
        self._setup_ui()

    def _setup_ui(self):
        self.setWindowTitle('Servo Action Panel')
        self.setFixedSize(340, 460)
        self.setStyleSheet("""
            QWidget {
                background-color: #1e1e2e;
                color: #cdd6f4;
                font-family: 'Segoe UI', Arial, sans-serif;
            }
            QGroupBox {
                border: 1px solid #45475a;
                border-radius: 8px;
                margin-top: 10px;
                padding: 8px;
                font-weight: bold;
                color: #89b4fa;
            }
            QGroupBox::title {
                subcontrol-origin: margin;
                left: 10px;
                padding: 0 4px;
            }
            QPushButton {
                background-color: #313244;
                color: #cdd6f4;
                border: none;
                border-radius: 6px;
                padding: 10px 0;
                font-size: 13px;
            }
            QPushButton:hover  { background-color: #45475a; }
            QPushButton:pressed { background-color: #585b70; }
            QPushButton:disabled { background-color: #181825; color: #585b70; }
            QPushButton#stop_btn {
                background-color: #f38ba8;
                color: #1e1e2e;
                font-weight: bold;
            }
            QPushButton#stop_btn:hover { background-color: #eba0ac; }
        """)

        root = QVBoxLayout(self)
        root.setSpacing(10)
        root.setContentsMargins(14, 14, 14, 14)

        # Title
        title = QLabel('🎛  Servo Action Panel')
        title.setFont(QFont('Arial', 13, QFont.Bold))
        title.setAlignment(Qt.AlignCenter)
        title.setStyleSheet('color: #cdd6f4; margin-bottom: 4px;')
        root.addWidget(title)

        # Status bar
        self.status_label = QLabel('Ready')
        self.status_label.setAlignment(Qt.AlignCenter)
        self.status_label.setStyleSheet('color: #a6e3a1; font-size: 11px;')
        root.addWidget(self.status_label)

        # ── Preset motions ──────────────────────────────────────────────
        motion_group = QGroupBox('Motion Presets')
        mg_layout = QVBoxLayout(motion_group)
        mg_layout.setSpacing(8)

        actions = [
            ('⚡ Zigzag  (0° ↔ 180°)',  'zigzag',  '#f9e2af'),
            ('🔄 Full Sweep',            'sweep',   '#89b4fa'),
            ('🌊 Wave  (45° ↔ 135°)',   'wave',    '#94e2d5'),
            ('⚙️  Slow Sweep',           'slow_sweep', '#cba6f7'),
            ('🎯 Center  (90°)',         'center',  '#a6e3a1'),
            ('📌 Home  (0°)',            'home',    '#fab387'),
        ]

        self._action_buttons = []
        for label, action, color in actions:
            btn = QPushButton(label)
            btn.setStyleSheet(
                f'QPushButton {{ color: {color}; background-color: #313244; }}'
                f'QPushButton:hover {{ background-color: #45475a; }}'
                f'QPushButton:disabled {{ color: #585b70; }}'
            )
            btn.clicked.connect(lambda checked, a=action: self._run_action(a))
            mg_layout.addWidget(btn)
            self._action_buttons.append(btn)

        root.addWidget(motion_group)

        # ── Stop button ─────────────────────────────────────────────────
        stop_btn = QPushButton('■  STOP')
        stop_btn.setObjectName('stop_btn')
        stop_btn.setFixedHeight(42)
        stop_btn.clicked.connect(self._stop_action)
        root.addWidget(stop_btn)

    def _set_busy(self, busy: bool, status: str = 'Ready'):
        self._busy = busy
        self.status_label.setText(status)
        self.status_label.setStyleSheet(
            f'color: {"#f38ba8" if busy else "#a6e3a1"}; font-size: 11px;'
        )
        for btn in self._action_buttons:
            btn.setEnabled(not busy)

    def _stop_action(self):
        self._stop = True
        self.node.send_action('stop')
        self._set_busy(False, 'Stopped')

    def _run_action(self, action: str):
        if self._busy:
            return
        self._stop = False
        self._set_busy(True, f'Running: {action}…')
        threading.Thread(target=self._execute, args=(action,), daemon=True).start()

    def _execute(self, action: str):
        try:
            if action == 'zigzag':
                for _ in range(6):
                    if self._stop: break
                    self.node.send_action('angle:0')
                    time.sleep(0.5)
                    if self._stop: break
                    self.node.send_action('angle:180')
                    time.sleep(0.5)

            elif action == 'sweep':
                for angle in list(range(0, 181, 5)) + list(range(180, -1, -5)):
                    if self._stop: break
                    self.node.send_action(f'angle:{angle}')
                    time.sleep(0.03)

            elif action == 'slow_sweep':
                for angle in list(range(0, 181, 2)) + list(range(180, -1, -2)):
                    if self._stop: break
                    self.node.send_action(f'angle:{angle}')
                    time.sleep(0.08)

            elif action == 'wave':
                for _ in range(4):
                    for angle in list(range(45, 136, 3)) + list(range(135, 44, -3)):
                        if self._stop: break
                        self.node.send_action(f'angle:{angle}')
                        time.sleep(0.04)

            elif action == 'center':
                self.node.send_action('angle:90')

            elif action == 'home':
                self.node.send_action('angle:0')

        finally:
            if not self._stop:
                self._set_ui_ready()

    def _set_ui_ready(self):
        # Safe call from background thread via QTimer
        QTimer.singleShot(0, lambda: self._set_busy(False, 'Done ✓'))


def main(args=None):
    rclpy.init(args=args)
    node = ServoActionsNode()

    # Spin ROS in background thread (no tkinter = no segfault)
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    app = QApplication(sys.argv)
    gui = ServoActionsGUI(node)
    gui.show()
    app.exec_()

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
