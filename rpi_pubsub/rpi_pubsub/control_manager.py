'''
Description:
This code receives Twist messages from teleop_keyboard and sends them to a hardware interface (Arduino) over serial

Written by: Christopher Payne
Date: 19 Mar. 2025
'''

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist # For teleop-keyboard commands

import serial
import time


class ControlManager(Node):

    def __init__(self):
        super().__init__('ControlManager')
        self.cmd_sub = self.create_subscription(
                Twist,
                'cmd_vel',
                self.cmd_listener_callback,
                10)

        self.cmd_sub # prevent unused variable warning

    def cmd_listener_callback(self, msg):
            self.get_logger().info(f'Base Cmd Vels:\nLinear: [{msg.linear.x},{msg.linear.y},{msg.linear.z}]\nAngular: [{msg.angular.x},{msg.angular.y},{msg.angular.z}]')
            cmd_vels = [[msg.linear.x,msg.linear.y,msg.linear.z], [msg.angular.x,msg.angular.y,msg.angular.z]]
            stop = [[0, 0, 0],[0, 0, 0]]
            fwd = [[0.5, 0, 0],[0, 0, 0]]
            bwd = [[-0.5, 0, 0],[0, 0, 0]]
            ptl = [[0, 0, 0],[0, 0, 1]]
            ptr = [[0.5, 0, 0],[0, 0, -1]]
            swtlf = [[0.5, 0, 0],[0, 0, 1]]
            swtrf = [[0.5, 0, 0],[0, 0, -1]]
            swtlb = [[-0.5, 0, 0],[0, 0, -1]]
            swtrb = [[-0.5, 0, 0],[0, 0, 1]]

            linear_s_d = msg.linear.x
            angular_s_d = msg.angular.z
            cmd = [linear_s_d, angular_s_d]

            if cmd[0] > 0:
                # Moving Forward
                if cmd[1] > 0:
                    print("Forward Swing Turn Left")
                    msg = [0,cmd[1]]
                elif cmd[1] < 0:
                    print("Forward Swing Turn Right")
                    msg = [-1*cmd[1],0]
                else:
                    print("Forward")
                    msg = [cmd[0],cmd[0]]
            elif cmd[0] < 0:
                # Moving Backward
                if cmd[1] < 0:
                    print("Backward Swing Turn Left")
                    msg = [0,cmd[1]]
                elif cmd[1] > 0:
                    print("Backward Swing Turn Right")
                    msg = [-1*cmd[1],0]
                else:
                    print("Backward")
                    msg = [cmd[0],cmd[0]]
            else:
                # No Linear Movement
                if cmd[1] > 0:
                    print("Point Turn Left")
                    msg = [-1*cmd[1],cmd[1]]
                elif cmd[1] < 0:
                    print("Point Turn Right")
                    msg = [-1*cmd[1],cmd[1]]
                else:
                    print("Stop")
                    msg = [cmd[0],cmd[0]]

            hardware_msg = f"L_M, R_M: [{msg}]"
            print(f"Sending: {hardware_msg}")

def main(args=None):
    rclpy.init(args=args)

    # Setting up the Hardware Interface Connection
    '''global ser
    ser = serial.Serial("/dev/ttyS0", 115200)
    ser.parity = serial.PARITY_NONE
    ser.stopbits = serial.STOPBITS_ONE
    '''


    control_manager = ControlManager()

    try:

        rclpy.spin(control_manager)

    except KeyboardInterrupt:

        print("Ctrl+C was pressed - Exiting...")

    finally:

        hardware_msg = [0,0]
        print(f"Sending: {hardware_msg}")
        print(f"Stopping All Motors:...")
        time.sleep(1)

        print(f"Turn Off All Motors:")
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    control_manager.destroy_node()
    rclpy.shutdown()

    print("Closing")
    print("Closing Program")

if __name__ == '__main__':
    main()
