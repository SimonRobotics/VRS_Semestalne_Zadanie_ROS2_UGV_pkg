import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
from mavros_msgs.srv import CommandBool, SetMode
import time

class TankControlNode(Node):

    def __init__(self):
        super().__init__('tank_control_node')

        self.ARM_BUTTON = 1
        self.armed = False
        self.last_button_state = 0

        self.subscription = self.create_subscription(
            Twist,
            'cmd_vel',
            self.velocity_callback,
            10)
        
        self.subscription = self.create_subscription(
            Joy,
            'joy',
            self.joy_callback,
            10)
        
        self.vel_pub = self.create_publisher(
            Twist,
            '/mavros/setpoint_velocity/cmd_vel_unstamped',
            10
        )

        self.arm_client = self.create_client(
            CommandBool,
            '/mavros/cmd/arming'
        )

        self.mode_client = self.create_client(
            SetMode,
            '/mavros/set_mode'
        )

        self.timer = self.create_timer(0.05, self.send_cmd)

        self.get_logger().info('Waiting for MAVROS services...')
        self.wait_for_services()
        self.get_logger().info('MAVROS services are ready!')

        self.set_mode('GUIDED')

        

    def wait_for_services(self):
        """Wait for all services to be available."""
        services = [
            self.arm_client,
            self.mode_client,
        ]
        for service in services:
            while not service.wait_for_service(timeout_sec=1.0):
                self.get_logger().info('Waiting for service...')

    def velocity_callback(self, msg):
        self.vel_pub(msg)
    
    def joy_callback(self, msg):
        current_state = msg.buttons[self.ARM_BUTTON]

        if current_state == 1 and self.last_button_state == 0:
            target_state = not self.armed
            if self.arm(target_state):
                self.armed = target_state                

        self.last_button_state = current_state

    def arm(self,  arm: bool) -> bool:
        req = CommandBool.Request()
        req.value = arm

        future = self.arming_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None:
            if future.result().success:
                self.get_logger().info("Vehicle armed successfully" if arm else "Vehicle disarmed successfully")
                return True
            else:
                self.get_logger().warn("Failed to arm vehicle" if arm else "Failed to disarm vehicle")
                return False
        else:
            self.get_logger().error("Arming/Disarmin service call failed")
            return False
    
    def set_mode(self, mode: str) -> bool:
        req = SetMode.Request()
        req.custom_mode = mode

        future = self.set_mode_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None:
            if future.result().mode_sent:
                self.get_logger().info(f'Mode set to {mode}')
                return True
            else:
                self.get_logger().warn(f'Failed to set mode to {mode}')
                return False
        else:
            self.get_logger().error('Set mode service call failed')
            return False

def main(args=None):
    rclpy.init(args=args)
    node = TankControlNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()