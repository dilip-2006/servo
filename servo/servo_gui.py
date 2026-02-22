import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
import tkinter as tk
import threading
import time


class ServoGUINode(Node):
    def __init__(self):
        super().__init__('servo_gui_node')
        self.publisher_ = self.create_publisher(Int32, '/servo_angle', 10)
        self.get_logger().info('Servo GUI Node started. Publishing to /servo_angle')

    def publish_angle(self, angle: int):
        msg = Int32()
        msg.data = angle
        self.publisher_.publish(msg)
        self.get_logger().info(f'Published angle: {angle}')


class ServoGUI:
    def __init__(self, node: ServoGUINode):
        self.node = node
        self._zigzag_running = False

        self.root = tk.Tk()
        self.root.title('Servo Controller GUI')
        self.root.geometry('420x240')
        self.root.resizable(False, False)
        self.root.configure(bg='#1e1e2e')

        # Title
        tk.Label(
            self.root,
            text='🎛  Servo Joint Controller',
            font=('Helvetica', 14, 'bold'),
            bg='#1e1e2e',
            fg='#cdd6f4'
        ).pack(pady=(18, 4))

        # Angle display
        self.angle_var = tk.IntVar(value=90)
        self.angle_label = tk.Label(
            self.root,
            text='Angle: 90°',
            font=('Helvetica', 12),
            bg='#1e1e2e',
            fg='#89b4fa'
        )
        self.angle_label.pack()

        # Slider — command fires on release to avoid rapid-fire serial writes
        self.slider = tk.Scale(
            self.root,
            from_=0,
            to=180,
            orient=tk.HORIZONTAL,
            variable=self.angle_var,
            length=360,
            tickinterval=45,
            resolution=1,
            bg='#313244',
            fg='#cdd6f4',
            troughcolor='#45475a',
            activebackground='#89b4fa',
            highlightthickness=0,
            command=self._on_slider_change
        )
        self.slider.pack(pady=8)

        # Buttons row
        btn_frame = tk.Frame(self.root, bg='#1e1e2e')
        btn_frame.pack(pady=6)

        for label, angle in [('0°', 0), ('90°', 90), ('180°', 180)]:
            tk.Button(
                btn_frame,
                text=label,
                width=7,
                bg='#45475a',
                fg='#cdd6f4',
                activebackground='#89b4fa',
                relief='flat',
                font=('Helvetica', 10),
                command=lambda a=angle: self._set_angle(a)
            ).pack(side=tk.LEFT, padx=6)

        self.zigzag_btn = tk.Button(
            btn_frame,
            text='Zigzag',
            width=7,
            bg='#f38ba8',
            fg='#1e1e2e',
            activebackground='#eba0ac',
            relief='flat',
            font=('Helvetica', 10, 'bold'),
            command=self._zigzag
        )
        self.zigzag_btn.pack(side=tk.LEFT, padx=6)

        self.root.protocol('WM_DELETE_WINDOW', self._on_close)

        # Poll rclpy on the main thread every 50 ms — avoids threading segfault
        self._ros_poll()

    def _ros_poll(self):
        """Drive ROS callbacks from the tkinter main thread."""
        rclpy.spin_once(self.node, timeout_sec=0)
        self.root.after(50, self._ros_poll)

    def _on_slider_change(self, value):
        angle = int(float(value))
        self.angle_label.config(text=f'Angle: {angle}°')
        self.node.publish_angle(angle)

    def _set_angle(self, angle: int):
        self.angle_var.set(angle)
        self.angle_label.config(text=f'Angle: {angle}°')
        self.node.publish_angle(angle)

    def _zigzag(self):
        if self._zigzag_running:
            return
        self._zigzag_running = True
        self.zigzag_btn.config(state='disabled')

        def run():
            for _ in range(5):
                self.root.after(0, lambda: self._set_angle(0))
                time.sleep(0.6)
                self.root.after(0, lambda: self._set_angle(180))
                time.sleep(0.6)
            self.root.after(0, self._zigzag_done)

        threading.Thread(target=run, daemon=True).start()

    def _zigzag_done(self):
        self._zigzag_running = False
        self.zigzag_btn.config(state='normal')

    def _on_close(self):
        self.root.destroy()

    def run(self):
        self.root.mainloop()


def main(args=None):
    rclpy.init(args=args)
    node = ServoGUINode()

    gui = ServoGUI(node)
    gui.run()  # tkinter mainloop runs on main thread; ROS polled via after()

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
