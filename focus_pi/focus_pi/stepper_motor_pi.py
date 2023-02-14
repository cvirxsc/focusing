import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
import serial

class SerialNode(Node):
    def __init__(self):
        super().__init__('serial_node')
        self.subscription = self.create_subscription(
            Int32,
            'contour',
            self.listener_callback,
            10)
        self.serial_port = serial.Serial('/dev/ttyAMA0', 9600)

    def listener_callback(self, msg):
        self.serial_port.write(str(msg.data).encode())

def main(args=None):
    rclpy.init(args=args)
    serial_node = SerialNode("serial_pi")
    rclpy.spin(serial_node)
    serial_node.serial_port.close()
    serial_node.destroy_node()
    rclpy.shutdown()
