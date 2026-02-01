import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Imu

import serial
import struct
import time


SOF1 = 0xA1
SOF2 = 0x1A
FRAME_LEN = 9

class ImuReader(Node):

    def __init__(self, port='/dev/ttyACM0', baudrate=38400, timeout=1):
        super().__init__('imu_reader')

        self.baudrate = baudrate
        self.timeout = timeout
        self.port = port

        self.ser = None

        self.connect_serial()

        self.msg = Imu()

        self.publisher_ = self.create_publisher(Imu, 'imu_topic', 10)
        timer_period = 0.004
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
    def timer_callback(self):
        if self.ser is None or not self.ser.is_open:
            if not self.connect_serial():
                print("False")
                return

        value = self.read_float_frame(self.ser)
        if value is None:
            return

        self.msg.angular_velocity.z = value
        self.publisher_.publish(self.msg)


    def connect_serial(self):

        try:
            self.ser = serial.Serial(self.port, self.baudrate, timeout=self.timeout)
            self.ser.reset_input_buffer()
            return True
        except serial.SerialException as e:
            print(f"Serial port error: {e}")
            if self.ser is not None and self.ser.is_open:
                self.ser.close()
            time.sleep(1)
            return False

    def read_float_frame(self, ser):
        try:
            while ser.in_waiting > 0:
                if ser.read(1)[0] != SOF1:
                    continue
                if ser.read(1)[0] != SOF2:
                    continue

                payload_len = ser.read(1)[0]
                if payload_len != 4:
                    continue

                payload = ser.read(4)
                crc_rx = ser.read(2)
                crc_rx = crc_rx[0] | (crc_rx[1] << 8)

                crc_calc = self.crc16_ccitt_false(bytes([SOF1, SOF2, payload_len]) + payload)

                if crc_rx != crc_calc:
                    continue

                return struct.unpack('<f', payload)[0]
            return None
        except:
            print("Error while reading")
            try:
                self.ser.close()
            except:
                pass
            
            return None
        
    def crc16_ccitt_false(self, data: bytes) -> int:
        crc = 0xFFFF
        for byte in data:
            crc ^= byte << 8
            for _ in range(8):
                if crc & 0x8000:
                    crc = ((crc << 1) ^ 0x1021) & 0xFFFF
                else:
                    crc = (crc << 1) & 0xFFFF
        return crc

    def __del__(self):
        self.ser.reset_input_buffer()
        if self.ser.is_open:
            self.ser.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = ImuReader()

    rclpy.spin(minimal_publisher)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()