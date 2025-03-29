import struct
import serial
import time
# x=0.8
# y=0.2

ser = serial.Serial('COM15', 9600, timeout=1)

#print(ser)
time.sleep(3)

def sendmess(msg):
    # bin = str(text) + str(x) + str(y) #Pack float value into 4 bytes
    data = ser.write(msg.encode())
    # data = ser.write(bin.encode())
    echo = rxmsg()
    print("Echo: " + echo.decode())
    #ser.close()

def rxmsg():
    echo = ser.readline()
    return echo

while True:
    cmd = input("Enter the DC for the Motors (format: <str, float, float>): ")
    sendmess(cmd)