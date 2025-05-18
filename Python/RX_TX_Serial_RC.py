import serial
from serial.tools import list_ports
import time

#----------------------
# Defined System Values:
#----------------------
RC_FWD_MAX_DC = 10.0 # (%)
RC_BWD_MAX_DC = 5.0 # (%)
RC_STOP_DC = 7.5 # (%)
RC_FWD_MAX_PW_US = 2000.0 # (us)
RC_BWD_MAX_PW_US = 1000.0 # (us)
RC_STOP_PW_US =  1500.0 # (us)

#---------------------
# Default State Values
#---------------------
DEFAULT_SWITCH_CONTROL = 0 # 0: RC Control; 1: ROS Control; Else: RC Control
DEFAULT_ENCODER = -999999999999 # TODO: Find a better inital value
DEFAULT_MOTOR_VEL = 0.0

#----------------------
# Initial State Values
#----------------------
#Control Switch
switch_control = DEFAULT_SWITCH_CONTROL # 0: RC Control; 1: ROS Control; Else: RC Control
#Encoders
encoderLeftVal = DEFAULT_ENCODER
encoderRightVal = DEFAULT_ENCODER
#Left Motor
leftMotorVel = DEFAULT_MOTOR_VEL
leftMotorDC = RC_STOP_DC
leftMotorPW_US = RC_STOP_PW_US
#Right Motor
rightMotorVel = DEFAULT_MOTOR_VEL
rightMotorDC = RC_STOP_DC
rightMotorPW_US = RC_STOP_PW_US
#Arm Tilt
#armTiltDir = 0 # -1: BWD; 0: Stop; 1: FWD; Else: Stop
armTiltVel = DEFAULT_MOTOR_VEL # Speed for tilt (float); positive: FWD; negative: BWD
armTiltDC = RC_STOP_DC # % for DC for tilt
armTilt_PW_US = RC_STOP_PW_US
#Arm Extend
#armExtendDir = 0 # -1: BWD; 0: Stop; 1: FWD; Else: Stop
armExtendVel = DEFAULT_MOTOR_VEL # Speed for Extend; positive: FWD; negative: BWD
armExtendDC = RC_STOP_DC # % for DC for Extend
armExtend_PW_US = RC_STOP_PW_US
#Drill
drillMotorVel = DEFAULT_MOTOR_VEL
drillMotorDC = RC_STOP_DC
drillMotorPW_US = RC_STOP_PW_US
#Bin
binMotorVel = DEFAULT_MOTOR_VEL
binMotorDC = RC_STOP_DC
binMotorPW_US = RC_STOP_PW_US

#####
angular = RC_STOP_PW_US
linear = RC_STOP_PW_US
armExtend = RC_STOP_PW_US
armTilt = RC_STOP_PW_US
drill = RC_STOP_PW_US
bin = RC_STOP_PW_US



df_idx = 0
# dataFieldsSerRx = [switch_control,
#                 encoderLeftVal, encoderRightVal,
#                 leftMotorVel, rightMotorVel, armTiltVel, armExtendVel, drillMotorVel, binMotorVel, 
#                 leftMotorDC, rightMotorDC, armTiltDC, armExtendDC, drillMotorDC, binMotorDC,
#                 leftMotorPW_US, rightMotorPW_US, armTilt_PW_US, armExtend_PW_US, drillMotorPW_US, binMotorPW_US]

dataFieldsSerRx = [angular, linear, armExtend, armTilt, drill, bin]

dataFieldsSerTx = [switch_control,
                   leftMotorVel, rightMotorVel, armTiltVel, armExtendVel, drillMotorVel, binMotorVel]


def sendMsgToSerial(ser:serial.Serial, msg:list):
    '''
    The message (msg) must be a list, (i.e, Sending "hello", msg = ["hello"]).

    ***IMPORTANT NOTE: The Serial Rx is expecting the following format: "<data>",
        where the message is a string that starts with a "<" and ends with a ">"
        ***This format must match the expected format for the Serial Rx***
        formatted_msg = "<data1, data2, ..., dataN>"
    '''
    formatted_msg = "<" + ", ".join(str(i) for i in msg) + ">"
    data = ser.write(formatted_msg.encode())
    #ser.close()

def getHardwareMsgToSer()->list:

    '''IMPORTANT NOTE: ORDER of Data Being transmitted to Serial Receiver. This MUST match the ORDER of Data on the Serial Receiver.
        dataFieldsTx = [(int),
                      (float), (float), (float), (float), (float), (float)]
    '''
    switch_control = DEFAULT_SWITCH_CONTROL #TODO: Replace with real values from Base Station
    leftMotorVel = DEFAULT_MOTOR_VEL#TODO: Replace with real values from Base Station
    rightMotorVel = DEFAULT_MOTOR_VEL#TODO: Replace with real values from Base Station
    armTiltVel = DEFAULT_MOTOR_VEL#TODO: Replace with real values from Base Station
    armExtendVel = DEFAULT_MOTOR_VEL#TODO: Replace with real values from Base Station
    drillMotorVel = DEFAULT_MOTOR_VEL#TODO: Replace with real values from Base Station
    binMotorVel = DEFAULT_MOTOR_VEL#TODO: Replace with real values from Base Station

    dataFieldsSerTx = [switch_control,
                       leftMotorVel, rightMotorVel, armTiltVel, armExtendVel, drillMotorVel, binMotorVel]
    
    # NOTE: Input is a blocking function. Comment it out to receive data when they are sent from the arduino with no delay.
    #dataFieldsSerTx = [input("Enter the value for the hardware (switch (0|1), leftMotor(float), rightMotor(float), armTilt(float), armExtend(float), drill(float), bin(float)): ")]
    
    return dataFieldsSerTx

def recvMsgFromSerial(ser:serial.Serial):
    try:
        # Read the line
        s_bytes = ser.readline()
        decoded_bytes = s_bytes.decode("utf-8").strip('\r\n')
        # print(decoded_bytes) # Troubleshooting
        
        data = decoded_bytes.split() # split the received data string into an array using a " " as the split.
        # print(data) # Troubleshooting

        
        '''
        ***BIG NOTE: This section MUST match the dataFieldsSerTx order from the Serial Transmitter for proper messages to be read!!!***
        dataFieldsTx = [switch_control ,
                        encoderLeftVal ,
                        encoderRightVal ,
                        leftMotorVel ,
                        rightMotorVel ,
                        armTiltVel ,
                        armExtendVel ,
                        drillMotorVel ,
                        binMotorVel, 
                        leftMotorDC ,
                        rightMotorDC ,
                        armTiltDC ,
                        armExtendDC ,
                        drillMotorDC ,
                        binMotorDC ,
                        leftMotorPW_US ,
                        rightMotorPW_US ,
                        armTilt_PW_US ,
                        armExtend_PW_US ,
                        drillMotorPW_US,
                        binMotorPW_US,
                        ]
        '''
        if len(data) > 0:
            
            # #Control Switch
            # switch_control = int(data[0]) # 0: RC Control; 1: ROS Control; Else: RC Control
            # #Encoders
            # encoderLeftVal = int(data[1])
            # encoderRightVal = int(data[2])
            # #Left Motor
            # leftMotorVel = float(data[3])
            # leftMotorDC = float(data[4])
            # leftMotorPW_US = float(data[5])
            # #Right Motor
            # rightMotorVel = float(data[6])
            # rightMotorDC = float(data[7])
            # rightMotorPW_US = float(data[8])
            # #Arm Tilt
            # #armTiltDir = 0 # -1: BWD; 0: Stop; 1: FWD; Else: Stop
            # armTiltVel = float(data[9]) # Speed for tilt (float); positive: FWD; negative: BWD
            # armTiltDC = float(data[10]) # % for DC for tilt
            # armTilt_PW_US = float(data[11])
            # #Arm Extend
            # #armExtendDir = 0 # -1: BWD; 0: Stop; 1: FWD; Else: Stop
            # armExtendVel = float(data[12]) # Speed for Extend; positive: FWD; negative: BWD
            # armExtendDC = float(data[13]) # % for DC for Extend
            # armExtend_PW_US = float(data[14])
            # #Drill
            # drillMotorVel = float(data[15])
            # drillMotorDC = float(data[16])
            # drillMotorPW_US = float(data[17])
            # #Bin
            # binMotorVel = float(data[18])
            # binMotorDC = float(data[19])
            # binMotorPW_US = float(data[20])

            # dataFieldsSerRx = [switch_control,
            #                 encoderLeftVal, encoderRightVal,
            #                 leftMotorVel, rightMotorVel, armTiltVel, armExtendVel, drillMotorVel, binMotorVel, 
            #                 leftMotorDC, rightMotorDC, armTiltDC, armExtendDC, drillMotorDC, binMotorDC,
            #                 leftMotorPW_US, rightMotorPW_US, armTilt_PW_US, armExtend_PW_US, drillMotorPW_US, binMotorPW_US])
            angular = int(data[0])
            linear = int(data[1])
            armExtend = int(data[2])
            armTilt = int(data[3])
            drill = int(data[4])
            bin = int(data[5])
            
            dataFieldsSerRx = [angular, linear, armExtend, armTilt, drill, bin]

            showSerialMsgRx(dataFieldsSerRx)
    except Exception as e:
        print("Error encountered, line was not recorded.")
        print(f"{e}")
    except ValueError as a:
        print("Error encountered, line was not recorded.")
        print(f"{a}")

def showSerialMsgRx(SerMsgRx):
    print(SerMsgRx)

def setupSerialPort()->serial.Serial:
    while True:
        try:
            ports = list_ports.comports()
            for p in ports:
                print(p)
            COM_PORT = "COM" + input("Enter the COM PORT # (e.g. 14) for your Arduino: ")
            ser = serial.Serial(COM_PORT, 115200, timeout=1)  # Replace with your Arduino's port and baud rate
            break
        except serial.SerialException as s:
            print(f"\nError: {s}\nCOM_PORT({COM_PORT}) selection was not correct. Please select another one.")
        except KeyboardInterrupt:
            print("Closing - KeyboardInterrupt")
            break

    print(f"Successful Serial Connection at {COM_PORT}")
    #https://github.com/curiores/ArduinoTutorials/blob/main/PythonProcess/serialReadSave.py
    # Toggle DTR to reset the Arduino
    ser.setDTR(False)
    time.sleep(1)
    ser.flushInput()
    ser.setDTR(True)
    
    return ser


def main():
        
    ser = setupSerialPort()
    
    k = 0 # Loop counter for troubleshooting purposes
    while True:
        k = k + 1
        recvMsgFromSerial(ser)
        #hardwareMsgToSer = getHardwareMsgToSer()
        #sendMsgToSerial(ser, hardwareMsgToSer)

if __name__ == "__main__":
    main()