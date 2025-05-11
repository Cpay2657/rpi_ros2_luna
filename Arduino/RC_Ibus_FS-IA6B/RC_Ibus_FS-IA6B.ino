#include <IBusBM.h>
#include <Servo.h>
#include <string.h>


/*
  Translate iBUS signal to servo for MBED architecture (such as Arduino Nano 33 BLE)
  
  Supports any Arduino board where serial0 is available. 
 
  For boards where serial0 is connected to the onboard USB port (such as MEGA, UNO and micro) you need
  to disconnect the RX line from the receiver during programming. 

  Alternatively you can change the code below to use another serial port.

  Please use 5V boards only.

  Serial port RX/TX connected as follows:
  - RX connected to the iBUS servo pin (disconnect during programming on MEGA, UNO and micro boards!)
  - TX left open or for Arduino boards without an onboard USB controler - connect to an 
    USB/Serial converter to display debug information on your PC (set baud rate to 115200).  

*/

IBusBM IBus; // IBus object
Servo myservo;  // create servo object to control a servo

void setup() {
  Serial.begin(115200);

  while (!Serial)
    ;
    
  IBus.begin(Serial);    // iBUS connected to Serial1 - change to Serial0 or Serial2 port when required

  myservo.attach(9);  // attaches the servo on pin 9 to the servo object

  Serial.println("Start IBus2PWM");
}

int saveval[6] = {0};
char *channelLabels[] = {"Angular (CH1)", "Linear (CH2)", "ArmExtend (CH3)", "ArmTilt (CH4)", "Drill (CH5)", "Bin (CH6)"};

void loop() {
  int val[6];
  for(int i = 0; i < 6; i++){
    val[i] = IBus.readChannel(i); // get latest value for servo channel 1
  }
  

  if (saveval != val) {
    for(int i = 0; i < 6; i++){
      Serial.print(channelLabels[i]);Serial.print(": ");Serial.print(val[i]);Serial.print("\t"); // display new value in microseconds on PC
      
    }
    Serial.println();
    memcpy(val, saveval, sizeof(val));  //saveval = val;    
    myservo.writeMicroseconds(val[0]);   // sets the servo position 
  }
  
  delay(100);
}
