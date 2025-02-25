# this code read pwm from rc receiver. using pigpio, use pin GPIO# 18

import rclpy
from rclpy.node import Node

from std_msgs.msg import String

import pigpio
from rpi_pubsub import read_PWM

cmds = ["1","2","3","0"]


class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(String, 'topic', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0
        self.PWM_GPIO = 18
        self.pi = pigpio.pi()
        self.p = read_PWM.reader(self.pi, self.PWM_GPIO)

    def timer_callback(self):
        print("PWM Values\n")

        f = self.p.frequency()
        pw = self.p.pulse_width()
        dc = self.p.duty_cycle()

        msg = String()
        msg.data = "f={:.1f} pw={} dc={:.2f}".format(f, int(pw+0.5), dc)
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
        self.i += 1

def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
