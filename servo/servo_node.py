import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import serial
import time
import math


class ServoNode(Node):
    def __init__(self):
        super().__init__('servo_node')

        # Configure the serial connection
        try:
            self.ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)
            self.get_logger().info('Serial connection established on /dev/ttyUSB0')
            time.sleep(2)
        except serial.SerialException as e:
            self.get_logger().error(f'Failed to connect to serial port: {e}')
            self.ser = None

        # Subscribe to joint_state_publisher_gui output
        self.subscription = self.create_subscription(
            JointState,
            '/joint_states',
            self._joint_state_callback,
            10
        )
        self.get_logger().info('Subscribed to /joint_states — move the GUI slider to control servo')

    def _joint_state_callback(self, msg: JointState):
        """Receive joint angle (radians) from joint_state_publisher_gui and forward to Arduino."""
        try:
            idx = msg.name.index('servo_joint')
        except ValueError:
            return  # joint not in this message

        angle_rad = msg.position[idx]
        # Clamp to URDF limits (0 to pi) then convert to degrees
        angle_rad = max(0.0, min(math.pi, angle_rad))
        angle_deg = int(math.degrees(angle_rad))
        self.send_angle(angle_deg)

    def send_angle(self, angle: int):
        if self.ser and self.ser.is_open:
            try:
                self.ser.write(f'{angle}\n'.encode())
                self.get_logger().info(f'Sent angle: {angle}°')
            except Exception as e:
                self.get_logger().error(f'Error writing to serial: {e}')
        else:
            self.get_logger().warn('Serial port not open, cannot send angle')


def main(args=None):
    rclpy.init(args=args)
    
    welcome_msg = """
\033[1;36m==================================================
                 S E R V O 
          Created by Dilip Kumar
==================================================\033[0m
"""
    print(welcome_msg)
    
    node = ServoNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node.ser and node.ser.is_open:
            node.ser.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
