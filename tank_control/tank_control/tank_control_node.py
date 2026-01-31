import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import serial
import struct
import time


class CmdVelToSerial(Node):
    def __init__(self):

        super().__init__('tank_control_node')

        while True:
            try:
                self.ser = serial.Serial('/dev/ttyACM0', 38400, timeout=1)
                break
            except serial.SerialException:
                print("Waiting for serial device...")
                time.sleep(1)
        
        self.subscription = self.create_subscription(
            Twist,
            'cmd_vel',
            self.listener_callback,
            10)

    def listener_callback(self, msg):
        linear = msg.linear.x
        angular = msg.angular.z

        self.send_cmd(linear, angular)


    def send_cmd(self, v_linear: float, v_angular: float):
        header = b'\xA5\x5A'
        payload_len = 8

        payload = struct.pack('<ff', v_linear, v_angular)

        frame_wo_crc = header + bytes([payload_len]) + payload
        crc = self.crc_ccitt_false(frame_wo_crc)

        crc_bytes = struct.pack('<H', crc)  # LSB first

        frame = frame_wo_crc + crc_bytes
        try:
            self.ser.write(frame)
        except:
            print("droping frame, lost connection")
            print("trying to reconncet")
            self.ser.close()

            try:
                self.ser = serial.Serial('/dev/ttyACM0', 38400, timeout=1)
            except:
                print("error while creating serial connection")


    def crc_ccitt_false(self, data: bytes) -> int:
        crc = 0xFFFF
        for b in data:
            crc ^= b << 8
            for _ in range(8):
                if crc & 0x8000:
                    crc = (crc << 1) ^ 0x1021
                else:
                    crc <<= 1
                crc &= 0xFFFF
        return crc

def main(args=None):
    rclpy.init(args=args)
    node = CmdVelToSerial()

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()

if __name__ == '__main__':
    main()