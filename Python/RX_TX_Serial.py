import serial
from serial.tools import list_ports
import time

while True:
    try:
        ports = list_ports.comports()
        for p in ports:
            print(p)
        COM_PORT = "COM" + input("Enter the COM PORT # (e.g. 14) for your Arduino: ")
        ser = serial.Serial(COM_PORT, 9600, timeout=1)  # Replace with your Arduino's port and baud rate
        break
    except serial.SerialException as s:
        print(f"\nError: {s}\nCOM_PORT({COM_PORT}) selection was not correct. Please select another one.")
    except KeyboardInterrupt:
        print("Closing - KeyboardInterrupt")
        break

#https://github.com/curiores/ArduinoTutorials/blob/main/PythonProcess/serialReadSave.py
# Toggle DTR to reset the Arduino
ser.setDTR(False)
time.sleep(1)
ser.flushInput()
ser.setDTR(True)

def sendmess(msg):
    # bin = str(text) + str(x) + str(y) #Pack float value into 4 bytes
    data = ser.write(msg.encode())
    # data = ser.write(bin.encode())
    # echo = rxmsg()
    # print("Echo: " + echo.decode())
    #ser.close()

def rxmsg():
    echo = ser.readline()
    return echo

msg = "none"
leftMotorDC = -1.0
leftMotorPW_US = -1
rightMotorDC = -1.0
rightMotorPW_US = -1
encoderLeft = -999
encoderRight = -999

dataFields = [msg,leftMotorDC,leftMotorPW_US,rightMotorDC,rightMotorPW_US,encoderLeft,encoderRight]

def main():
        
    k = 0
    while True:
        k = k + 1
        try:
            # Read the line
            s_bytes = ser.readline()
            decoded_bytes = s_bytes.decode("utf-8").strip('\r\n')
            print(decoded_bytes)
            # Parse the line:
            #Format: data[0]    data[1] data[2]     data[3] data[4]         data[5] data[6]         data[7] data[8]         data[9] data[10]    data[11]data[12]        data[13]  
            #Format: Message    str     leftMotorDC float   leftMotorPW_US  int     rightMotorDC    float   rightMotorPW_US int     encoderLeft int     encoderRight    int 
            #Ex: Message Hi leftMotorDC -1.00 leftMotorPW_US 1000 rightMotorDC -1.00 rightMotorPW_US 1000 encoderLeft 0 encoderRight 0
            data = decoded_bytes.split() # split the received data string into an array using a " " as the split.

            if len(data) > 0:
                msg = data[1]
                leftMotorDC = float(data[3])
                leftMotorPW_US = int(data[5])
                rightMotorDC = float(data[7])
                rightMotorPW_US = int(data[9])
                encoderLeft = int(data[11])
                encoderRight = int(data[13])

                dataFields = [msg,leftMotorDC,leftMotorPW_US,rightMotorDC,rightMotorPW_US,encoderLeft,encoderRight]

            print(dataFields)

            # # # Parse the line
            # # if k == 0:
            # #     values = decoded_bytes.split(",")
            # # else:
            #     # values = [float(x) for x in decoded_bytes.split()]
            # values = [float(x) for x in decoded_bytes.split()]
            # print(values)
        except Exception as e:
            print("Error encountered, line was not recorded.")
            print(f"{e}")
        cmd = input("Enter the DC for the Motors (format: <str, float, float>): ")
        sendmess(cmd)

if __name__ == "__main__":
    main()