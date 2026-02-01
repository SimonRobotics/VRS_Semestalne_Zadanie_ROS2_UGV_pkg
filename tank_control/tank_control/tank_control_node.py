import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu, BatteryState
import serial
import struct
import time

SOF1_O = 0xA5
SOF2_O = 0x5A
PAYLOAD_LEN_O = 8
FRAME_LEN_O = (3 + PAYLOAD_LEN_O + 2)

SOF1_I = 0xA1
SOF2_I = 0x1A
PAYLOAD_LEN_I = 4
FRAME_LEN_I = (3 + PAYLOAD_LEN_I + 2)



class TankControl(Node):
    def __init__(self, port='/dev/ttyACM0', baudrate=38400, timeout=1):
        super().__init__('tank_control_node')

        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self.ser = None

        timer_period = 0.02

        self.connect_serial()
    
        self.msg_imu = Imu()
        #self.msg_battery = BatteryState()

        self.subscription = self.create_subscription(
            Twist,
            'cmd_vel',
            self.listener_callback,
            10)
        
        self.publisher_imu_ = self.create_publisher(Imu, 'imu_topic', 10)
        #self.publisher_battery_ = self.create_publisher(BatteryState, 'battery_topic', 10)
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        if self.ser is None or not self.ser.is_open:
            if not self.connect_serial():
                return

        value = self.read_float_frame(self.ser)
        if value is None:
            return

        self.msg_imu.angular_velocity.z = value
        self.publisher_imu_.publish(self.msg_imu)

    def listener_callback(self, msg):

        if self.ser is None or not self.ser.is_open:
            if not self.connect_serial():
                return
    
        linear = msg.linear.x
        angular = msg.angular.z
        try:
            self.ser.write(self.create_frame(linear, angular))
        except:
            print("Error while writing")
            try:
                if self.ser is not None and self.ser.is_open:
                    self.ser.close()
            except:
                pass


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

    def create_frame(self, v_linear: float, v_angular: float):

        payload = struct.pack('<ff', v_linear, v_angular)

        frame_wo_crc = bytes([SOF1_O,SOF2_O,PAYLOAD_LEN_O]) + payload
        crc = self.crc_ccitt_false(frame_wo_crc)

        crc_bytes = struct.pack('<H', crc) 

        return frame_wo_crc + crc_bytes
    
    def read_float_frame(self, ser):
        try:
            while ser.in_waiting > FRAME_LEN_I:
                b = self.ser.read(1)
                if b[0] != SOF1_I:
                    continue

                b = self.ser.read(1)
                if b[0] != SOF2_I:
                    continue

                payload_len = self.ser.read(1)
                if payload_len[0] != PAYLOAD_LEN_I:
                    continue

                payload = self.ser.read(4)
                if len(payload) != PAYLOAD_LEN_I:
                    continue

                crc_bytes = self.ser.read(2)
                if len(crc_bytes) != 2:
                    continue

                crc_rx = crc_bytes[0] | (crc_bytes[1] << 8)

                crc_calc = self.crc_ccitt_false(bytes([SOF1_I, SOF2_I, payload_len[0]]) + payload)

                if crc_rx != crc_calc:
                    continue

                return struct.unpack('<f', payload)[0]
            return None
        except Exception as e:
            print(f"error while reading: {e}")
            try:
                if self.ser is not None and self.ser.is_open:
                    self.ser.close()
            except:
                pass
            return None

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
    
    def __del__(self):
        self.ser.reset_input_buffer()
        if self.ser.is_open:
            self.ser.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = TankControl()

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()

if __name__ == '__main__':
    main()