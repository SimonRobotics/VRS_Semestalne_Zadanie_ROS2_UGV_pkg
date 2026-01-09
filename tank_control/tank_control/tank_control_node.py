import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import serial
import struct

class CmdVelToSerial(Node):
    def __init__(self):

        super().__init__('tank_control_node')

        self.subscription = self.create_subscription(
            Twist,
            'cmd_vel',
            self.listener_callback,
            10)

        try:
            self.ser = serial.Serial('/dev/ttyACM0', 115200, timeout=1)
        except:
            print("error while creating serial connection")

    def listener_callback(self, msg):
        linear = msg.linear.x
        angular = msg.angular.z

        command = struct.pack('ff',linear, angular)

        self.ser.write(command)

def main(args=None):
    rclpy.init(args=args)
    node = CmdVelToSerial()

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()

if __name__ == '__main__':
    main()