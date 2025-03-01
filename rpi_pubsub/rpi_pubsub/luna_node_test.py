'''
Description:
This code is the primary node for running the Luna Bot's hardware.

Written by: Christopher Payne
Date: 28 Feb. 2025

This code currently supports:

	1.) Reading I-Bus protocol up to 6-channels
	2.) Setting up 6 PWMs formatted for RC control (50 Hz, DC Range: 5%-10%)
	3.) Controlling the duty cycle of up to 6 PWM outputs
	4.) Receiving commands to switch between RC I-Bus or Pi inputs
	5.) Receiving commands to change the speed of the Pi inputs
	6.) Stops All Motors and cleans output GPIOs on shutdown
	7.) Move Commands Implemmented:
		a.) Drive speed
		b.) Drive direction
		c.) Drill direction and speed

This code currently needs:

	1.) TODO: Receiving commands to change the speed of the Pi inputs
	2.) TODO: Add ability to accept input from Panda Computer
	3.) TODO: Move Commands to be implemmented:
		a.) Arm Tilt direction and speed (position?)
		b.) Arm Length direction and speed (position?)
		c.) Bin Tilt direction and speed (position?)



most recent update: 28 Feb. 2025
'''

import rclpy
from rclpy.node import Node

from std_msgs.msg import String

import pigpio
import serial
import time

class Luna(Node):

    def __init__(self):
        super().__init__('Luna')
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

        # Setting up the timers
        info_timer_period = 0.5  # seconds
        self.luna_info_timer = self.create_timer(info_timer_period, self.luna_info_timer_callback)

        output_timer_period = 0.0001  # seconds
        self.luna_output_timer = self.create_timer(output_timer_period, self.luna_output_timer_callback)
        self.i = 0


        # Setting up the pigpio instance
        self.pi = pigpio.pi()
        if not self.pi.connected:
            print("\npigpio is not connected\n")
            exit(0)

        # Setting up the RC Reader for GPIO 15 (BCM, UART Rx)
        self.RC_GPIO_IN = 15

        # Default control system output is the Pi's PWM
        self.switch_status = "Pi"

        # Setting up the Pi's PWM Generator with pigpio
        self.Pi_FREQ = 50 # Hz based RC Rx's PWM (50 Hz, DC Range: 5%-10%)
        self.Pi_PWM_RANGE = 1000 # 1000 is 100% DC, 500 is 50% DC, 0 is 0% DC
        self.Pi_MAX = 100 # 10%
        self.Pi_MID = 75 # 7.5%
        self.Pi_MIN = 50 # 5%
        self.Pi_STOP = self.Pi_MID # The MID DC stops the motor

        # Setting up the Cmd Channels and Duty Cycles
        self.DIRECTION_CH = 0 #ch1 is direction
        self.SPEED_CH = 1 #ch2 is speed
        self.DRILL_CH = 2 #ch3 is drill speed & direction
        self.ARM_LENGTH_CH = 3 #ch4 is arm length
        self.ARM_TILT_CH = 4 #ch5 is arm tilt
        self.BIN_TILT_CH = 5 #ch6 is bin tilt


        # Setting up Luna's Output GPIOs
        self.L_MOTOR = 26
        self.R_MOTOR = 19
        self.DRILL = 13
        self.ARM_TILT = 6
        self.ARM_LENGTH = 5
        self.BIN_TILT = 11
        self.LUNA_GPIO_OUT = {"L_M":[self.L_MOTOR,self.Pi_STOP],
                              "R_M":[self.R_MOTOR,self.Pi_STOP],
                              "DRILL":[self.DRILL,self.Pi_STOP],
                              "ARM_T":[self.ARM_TILT,self.Pi_STOP],
                              "ARM_L":[self.ARM_LENGTH,self.Pi_STOP],
                              "BIN":[self.BIN_TILT,self.Pi_STOP]
                             }

        # Setting up every Output GPIO. (Initialize them at the stopping duty cycle) 
        for joint in self.LUNA_GPIO_OUT:
            GPIO = self.LUNA_GPIO_OUT[joint][0]
            self.pi.set_PWM_frequency(GPIO, self.Pi_FREQ)
            self.pi.set_PWM_range(GPIO, self.Pi_PWM_RANGE)
            self.pi.set_PWM_dutycycle(GPIO, self.Pi_STOP) # Default is the motor being stopped
            print(f"Pi PWM is setup: GPIO (BCM): {GPIO}, F(Hz): {self.Pi_FREQ}, DCs(%): {self.Pi_STOP}")

    def update_info(self, DCs):
        for i,key in enumerate(self.LUNA_GPIO_OUT.keys()):
            self.LUNA_GPIO_OUT[key][1] = DCs[i]


    def luna_move(self,DCs):
        drive_speed = DCs[self.SPEED_CH]
        drive_direction = DCs[self.DIRECTION_CH]
        arm_length = DCs[self.ARM_LENGTH_CH]
        drill_speed = DCs[self.DRILL_CH]
        arm_tilt = DCs[self.ARM_TILT_CH]
        bin_tilt = DCs[self.BIN_TILT_CH]

        drive_speed_norm = (drive_speed-75)/25 # 75=0, 50=-1,100=+1
        drive_direction_norm = (drive_direction-75)/25 # 75=0, 50=-1,100=+1
        arm_length_norm = (arm_length-75)/25 # 75=0, 50=-1,100=+1
        drill_speed_norm = (drill_speed-75)/25 # 75=0, 50=-1,100=+1
        arm_tilt_norm = (arm_tilt-75)/25 # 75=0, 50=-1,100=+1
        bin_tilt_norm = (bin_tilt-75)/25 # 75=0, 50=-1,100=+1

        # Troubleshooting Printouts
        print(f"drive_speed,drive_speed_norm: {drive_speed},{drive_speed_norm}")
        print(f"drive_direction,drive_direction_norm: {drive_direction},{drive_direction_norm}")
        print(f"arm_length,arm_lenght_norm: {arm_length},{arm_length_norm}")
        print(f"drill_speed,drill_speed_norm: {drill_speed},{drill_speed_norm}")
        print(f"arm_tilt,arm_tilt_norm: {arm_tilt},{arm_tilt_norm}")
        print(f"bin_tilt,bin_tilt_norm: {bin_tilt},{bin_tilt_norm}")

        # Joystick Tolerances
        joy_tol_pos = .04
        joy_tol_neg = -1*joy_tol_pos

        # One Direction Channel Zero Tolerance
        one_way_min_tol = 0.08 + self.Pi_MIN
        one_way_max_tol = self.Pi_MAX - 0.08

        if DCs != [0] * 6:
            # Motor Control Logic (drive_speed, drive_direction)
            if drive_direction_norm < joy_tol_neg:
                if abs(drive_speed_norm) >= joy_tol_pos :
                    print("Swing Turning Left")
                    self.pi.set_PWM_dutycycle(self.LUNA_GPIO_OUT["L_M"][0], drive_speed)
                    self.pi.set_PWM_dutycycle(self.LUNA_GPIO_OUT["R_M"][0], 0)
                else:
                    print("Point Turning Left")
                    self.pi.set_PWM_dutycycle(self.LUNA_GPIO_OUT["L_M"][0], drive_direction)
                    self.pi.set_PWM_dutycycle(self.LUNA_GPIO_OUT["R_M"][0], 150-drive_direction)

            elif drive_direction_norm > joy_tol_pos:
                if abs(drive_speed_norm) >= joy_tol_pos:
                    print("Swing Turning Right")
                    self.pi.set_PWM_dutycycle(self.LUNA_GPIO_OUT["L_M"][0], 0)
                    self.pi.set_PWM_dutycycle(self.LUNA_GPIO_OUT["R_M"][0], drive_speed)
                else:
                    print("Point Turning Right")
                    self.pi.set_PWM_dutycycle(self.LUNA_GPIO_OUT["L_M"][0], 150-drive_direction)
                    self.pi.set_PWM_dutycycle(self.LUNA_GPIO_OUT["R_M"][0], drive_direction)
            else:
                print("Straight")
                self.pi.set_PWM_dutycycle(self.LUNA_GPIO_OUT["L_M"][0], drive_speed)
                self.pi.set_PWM_dutycycle(self.LUNA_GPIO_OUT["R_M"][0], drive_speed)
            # Drill Control Logic (drill_speed)
            if drill_speed > one_way_min_tol and drill_speed < one_way_max_tol:
                print("Drilling")
                self.pi.set_PWM_dutycycle(self.LUNA_GPIO_OUT["DRILL"][0], drill_speed)
            elif drill_speed <= one_way_min_tol:
                print("Drill speed is at Min Boundary")
                self.pi.set_PWM_dutycycle(self.LUNA_GPIO_OUT["DRILL"][0], one_way_min_tol)
            elif drill_speed >= one_way_max_tol:
                print("Drill speed is at Max Boundary")
                self.pi.set_PWM_dutycycle(self.LUNA_GPIO_OUT["DRILL"][0], one_way_max_tol)
            else:
                print("Unexpected Input for Arm Length")
                print("Returning to Home Position (currently min)")
                self.pi.set_PWM_dutycycle(self.LUNA_GPIO_OUT["DRILL"][0], one_way_min_tol)
        else:
            print("Turning all Motors Off")
            for joint in self.LUNA_GPIO_OUT:
                self.pi.set_PWM_dutycycle(self.LUNA_GPIO_OUT[joint][0], 0)



    def luna_output_timer_callback(self):
        print(f"Sending output from: {self.switch_status}")
        if self.switch_status == "RC":
            DCs = self.read_ibus()
        elif self.switch_status == "Pi":
            DCs = [self.Pi_STOP] * 6 # TODO: add in speed from cmd topic
        else:
            DCs = [self.Pi_STOP] * 6 # TODO: create a number of channels varaible?
        # Move Luna
        self.luna_move(DCs)
        # Update the output PWMs
        self.update_info(DCs)

    def luna_info_timer_callback(self):
        print("Luna Info:\n")
        info = "Luna Info [GPIO, DC]:\nL_M: {self.LUNA_GPIO_OUT['L_M']}\nR_M: {self.LUNA_GPIO_OUT['R_M']}\nDRILL: {self.LUNA_GPIO_OUT['DRILL']}"
        msg = String()
        msg.data = info
        self.luna_info_publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
        self.i += 1

    def switch_listener_callback(self, msg):
        self.get_logger().info('Switch Status: "%s"' % msg.data)
        if msg.data in ["RC","Pi"]:
            if msg.data == "RC":
                self.switch_status = "RC"
            elif msg.data == "Pi":
                self.switch_status = "Pi"
            else:
                print(f"Invalid Switch Cmd was received: {msg.data}")
        else:
            print(f"Invalid Switch Cmd was received: {msg.data}")

    def cmd_listener_callback(self, msg):
        self.get_logger().info('Base Cmds: "%s"' % msg.data)

    def read_ibus(self):
        cf = 20 # conversion factor (microsec to the appropriate duty cycle range (50 to 100))
        frame = bytearray()
        received_data = None
        received_data = ser.read()  # read serial port
        intReceived = int.from_bytes(received_data, byteorder='little')
        if intReceived == 32:
            frame.extend(received_data)  # add the header
            # read the next 31 bytes of the frame (to make a 32 byte frame size)
            nextBytes = ser.read(31)
            # add the readed 31 bytes to the frame bytearray
            frame.extend(nextBytes)

            ch1byte = bytearray()
            ch1byte.append(frame[2])
            ch1byte.append(frame[3])
            ch1 = int.from_bytes(ch1byte, byteorder='little')

            ch2byte = bytearray()
            ch2byte.append(frame[4])
            ch2byte.append(frame[5])
            ch2 = int.from_bytes(ch2byte, byteorder='little')

            ch3byte = bytearray()
            ch3byte.append(frame[6])
            ch3byte.append(frame[7])
            ch3 = int.from_bytes(ch3byte, byteorder='little')

            ch4byte = bytearray()
            ch4byte.append(frame[8])
            ch4byte.append(frame[9])
            ch4 = int.from_bytes(ch4byte, byteorder='little')

            ch5byte = bytearray()
            ch5byte.append(frame[10])
            ch5byte.append(frame[11])
            ch5 = int.from_bytes(ch5byte, byteorder='little')

            ch6byte = bytearray()
            ch6byte.append(frame[12])
            ch6byte.append(frame[13])
            ch6 = int.from_bytes(ch6byte, byteorder='little')

            # flysky I6-i6 has only 6 channels - ends here

            print("ch1=", ch1,  "ch2=", ch2, "ch3=", ch3, "ch4=", ch4, "ch5=", ch5, "ch6=", ch6)

            return [ch1 / cf, ch2 / cf, ch3 / cf, ch4 / cf, ch5 / cf, ch6 / cf]
        else:
            return [75] * 6


def main(args=None):
    rclpy.init(args=args)

    # Setting up the I-Bus Serial Connection
    global ser
    ser = serial.Serial("/dev/ttyS0", 115200)
    ser.parity = serial.PARITY_NONE
    ser.stopbits = serial.STOPBITS_ONE


    luna = Luna()

    try:

        rclpy.spin(luna)

    except KeyboardInterrupt:

        print("Ctrl+C was pressed - Exiting...")

    finally:

        DCs_STOP = [luna.Pi_STOP] * 6
        luna.luna_move(DCs_STOP)
        print(f"Stopping All Motors:\nDCs: {DCs_STOP}...")
        time.sleep(1)

        print("Tidying up...")
        DCs_OFF = [0] * 6
        luna.luna_move(DCs_OFF)
        print(f"Turn Off All Motors:\nDCs: {DCs_OFF}...")
        luna.pi.stop()
        print("\nStopping PWM")

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    luna.destroy_node()
    rclpy.shutdown()

    print("Closing")
    luna.pi.close()
    print("Closing Program")

if __name__ == '__main__':
    main()
