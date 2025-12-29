import rclpy
from rclpy.node import Node
from std_msgs.msg import Int16MultiArray

import time
import math
import smbus


class PCA9685:
    __MODE1 = 0x00
    __PRESCALE = 0xFE
    __LED0_ON_L = 0x06
    __LED0_ON_H = 0x07
    __LED0_OFF_L = 0x08
    __LED0_OFF_H = 0x09

    def __init__(self, address=0x40, bus=1):
        self.bus = smbus.SMBus(bus)
        self.address = address
        self.bus.write_byte_data(self.address, self.__MODE1, 0x00)

    def set_pwm_freq(self, freq):
        prescaleval = 25000000.0
        prescaleval /= 4096.0
        prescaleval /= float(freq)
        prescaleval -= 1.0
        prescale = int(math.floor(prescaleval + 0.5))

        oldmode = self.bus.read_byte_data(self.address, self.__MODE1)
        newmode = (oldmode & 0x7F) | 0x10
        self.bus.write_byte_data(self.address, self.__MODE1, newmode)
        self.bus.write_byte_data(self.address, self.__PRESCALE, prescale)
        self.bus.write_byte_data(self.address, self.__MODE1, oldmode)
        time.sleep(0.005)
        self.bus.write_byte_data(self.address, self.__MODE1, oldmode | 0x80)

    def set_pwm(self, channel, on, off):
        self.bus.write_byte_data(self.address, self.__LED0_ON_L + 4 * channel, on & 0xFF)
        self.bus.write_byte_data(self.address, self.__LED0_ON_H + 4 * channel, on >> 8)
        self.bus.write_byte_data(self.address, self.__LED0_OFF_L + 4 * channel, off & 0xFF)
        self.bus.write_byte_data(self.address, self.__LED0_OFF_H + 4 * channel, off >> 8)

    def set_servo_pulse(self, channel, pulse_us):
        pulse = int(pulse_us * 4096 / 20000)
        self.set_pwm(channel, 0, pulse)


class ServoControllerNode(Node):
    def __init__(self):
        super().__init__('servo_controller_node')
        self.get_logger().info("Waveshare Servo Driver HAT (PCA9685) ROS2 controller started")

        self.pwm = PCA9685(0x40)
        self.pwm.set_pwm_freq(50)

        # Servo limits (SAFE)
        self.MIN_PULSE = 500
        self.MAX_PULSE = 2500

        self.subscription = self.create_subscription(
            Int16MultiArray,
            '/servo_command',
            self.command_callback,
            10
        )

    def angle_to_pulse(self, angle):
        angle = max(0, min(180, angle))
        return int(self.MIN_PULSE + (angle / 180.0) * (self.MAX_PULSE - self.MIN_PULSE))

    def command_callback(self, msg):
        for channel, angle in enumerate(msg.data):
            if channel >= 16:
                break

            pulse = self.angle_to_pulse(angle)
            self.pwm.set_servo_pulse(channel, pulse)

            #self.get_logger().info(
            #    f"Servo {channel}: angle={angle}° pulse={pulse}us"
            #)


def main(args=None):
    rclpy.init(args=args)
    node = ServoControllerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

