import rclpy
from rclpy.node import Node

from std_msgs.msg import String

import pigpio
import time

class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(
            String,
            'topic',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

        self.GPIO=[18]
        self.MAX_DC = 100
        self.MID_DC = 75
        self.MIN_DC = 50
        self.STOP_DC = self.MID_DC
        self.pi = pigpio.pi()
        if not self.pi.connected:
                exit(0)
        self.pi.set_PWM_frequency(self.GPIO[0], 50)
        self.pi.set_PWM_range(self.GPIO[0], 1000)
        self.pi.set_PWM_dutycycle(self.GPIO[0], 0)
        print(f"PWM DC is 0")
        time.sleep(1)

    def listener_callback(self, msg):
        self.get_logger().info('I heard: "%s"' % msg.data)
	#[print(i) for i in msg.data]
        try:
	        while True:
                        print("Running")
                        self.pi.set_PWM_frequency(self.GPIO[0], self.MAX_DC)
                        print(f"DC: {self.MAX_DC}")
                        time.sleep(3)

                        self.pi.set_PWM_frequency(self.GPIO[0], self.MID_DC)
                        print(f"DC: {self.MID_DC}")
                        time.sleep(3)

                        self.pi.set_PWM_frequency(self.GPIO[0], self.MIN_DC)
                        print(f"DC: {self.MIN_DC}")
                        time.sleep(3)
        except KeyboardInterrupt:
                pass

        print("\nStopping Motor")
        self.pi.set_PWM_frequency(self.GPIO[0], self.STOP_DC)
        print(f"STOP_DC: {self.STOP_DC}")
        time.sleep(1)
        print("\ntidying up")
        self.pi.set_PWM_frequency(self.GPIO[0], 0)
        self.pi.stop()


def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = MinimalSubscriber()

    rclpy.spin(minimal_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
