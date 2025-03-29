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

k = 0
while True:
    k = k + 1
    try:
        # Read the line
        s_bytes = ser.readline()
        decoded_bytes = s_bytes.decode("utf-8").strip('\r\n')
        # print(decoded_bytes)

        # Parse the line
        if k == 0:
            values = decoded_bytes.split(",")
        else:
            values = [float(x) for x in decoded_bytes.split()]
        print(values)
    except Exception as e:
        print("Error encountered, line was not recorded.")
        print(f"{e}")
    # command = input("Enter command (LED_ON/LED_OFF): ")
    # ser.write(command.encode())  # Send the command