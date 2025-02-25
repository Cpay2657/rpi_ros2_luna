# this code read pwm from rc receiver. using pigpio, use pin GPIO# 18

import rclpy
from rclpy.node import Node

from std_msgs.msg import String

import pigpio
from rpi_pubsub import read_PWM

class Luna(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        self.luna_info_publisher_ = self.create_publisher(String, 'Luna_Info', 10)
        self.switch_sub = self.create_subscription(
                String,
                'LunaSwitch',
                self.switch_listener_callback,
                10)
        self.cmd_sub = self.create_subscription(
                String,
                'LunaCmds',
                self.cmd_listener_callback,
                10)

        self.switch_sub # prevent unused variable warning
        self.cmd_sub # prevent unused variable warning
        timer_period = 0.5  # seconds
        self.luna_info_timer = self.create_timer(timer_period, self.luna_info_timer_callback)
        self.i = 0

        # Setting up the PWM Reader for GPIO 18 (BCM)
        self.PWM_GPIO = 18
        self.pi = pigpio.pi()
        self.p = read_PWM.reader(self.pi, self.PWM_GPIO)

    def luna_info_timer_callback(self):
        print("Luna Info:\n")

        # Reading the RC input information
        f = self.p.frequency()
        pw = self.p.pulse_width()
        dc = self.p.duty_cycle()

        print(f"RC Info:\nf(Hz)={f:.1f} pw(ms)={int(pw+0.5)} dc(%)={dc:.2f}")

        msg = String()
        msg.data = "f={:.1f} pw={} dc={:.2f}".format(f, int(pw+0.5), dc)
        self.luna_info_publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
        self.i += 1

    def switch_listener_callback(self, msg):
        self.get_logger().info('Switch Status: "%s"' % msg.data)

    def cmd_listener_callback(self, msg):
        self.get_logger().info('Base Cmds: "%s"' % msg.data)

def main(args=None):
    rclpy.init(args=args)

    luna = Luna()

    rclpy.spin(luna)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    luna.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
