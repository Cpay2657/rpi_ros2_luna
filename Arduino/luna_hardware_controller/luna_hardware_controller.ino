/* 
 * This code was edited by Christopher Payne on 1 April 2025

 TODO: Solve why the switch_pin always starts HIGH. It should start LOW for RC Default. - C.P. 15 April 2025

 example serial input: <0,2,-2,0,0,0,0>
 switch pin: off, linearVel = +2 m/s (fwd), angularVel = -2 rev/s (left), armTiltVel = 0, armExtendVel = 0, drillVel = 0, binVel = 0
 */

#include <Encoder.h>
#include <Servo.h>

// Change these pin numbers to the pins connected to your encoder.
//   Interrupt Pins for Arduino Uno: Pins	2 & 3	
//   Do NOT use these Pins for an Arduino Uno: 13 (These pins have a LED attached, which is bad!!!)

// Hardware Pins
static int encoderLeftPinA = 2;
static int encoderLeftPinB = 5;
static int encoderRightinA = 3;
static int encoderRightPinB = 6;
static int linearPin = 9;
static int angularPin = 10;
static int switchPin = 13;
static int armTiltPin = 0;
static int armExtendPin = 1;
static int drillMotorPin = 12;
static int binMotorPin = 7;

// Setting up rcPWM
unsigned long previousMillis = 0;
float rcPWMPeriodMaxUS = 2000;
float rcPWMPeriodMidUS = 1500;
float rcPWMPeriodMinUS = 1000;

//Setting up Duty Cycle Range
float FwdMaxDC = 100;
float StopDC = 0;
float BwdMaxDC = -100;

//Setting up Velocity Ranges
float driveMotorMaxVel = 2; //m/s
float armTiltMaxVel = 2; //m/s
float armExtendMaxVel = 2; //m/s
float drillMotorMaxVel = 2; //m/s
float binMotorMaxVel = 2; //m/s
float StopVel = 0;

//Motors' Initial Vel:
float linearVelInit = 0.0; //m/s
float angularVelInit = 0.0; //m/s
float armTiltVelInit = 0.0; //m/s
float armExtendVelInit = 0.0; //m/s
float drillMotorVelInit = 0.0; //m/s
float binMotorVelInit = 0.0; //m/s

//Intialize all message values
int switch_control;
long encoderLeftVal;
long encoderRightVal;
float linearVel;
float angularVel;
float armTiltVel;
float armExtendVel;
float drillMotorVel;
float binMotorVel;
float linearDC;
float angularDC;
float armTiltDC;
float armExtendDC;
float drillMotorDC;
float binMotorDC;
float linearPW_US;
float angularPW_US;
float armTiltPW_US;
float armExtendPW_US;
float drillMotorPW_US;
float binMotorPW_US;

void initMsgValues(){
  //Intialize all message values
  int switch_control = 0; // Default to RC Control
  long encoderLeftVal  = -999;
  long encoderRightVal = -999;
  float linearVel = linearVelInit;
  float angularVel = angularVelInit;
  float armTiltVel = armTiltVelInit;
  float armExtendVel = armExtendVelInit;
  float drillMotorVel = drillMotorVelInit;
  float binMotorVel = binMotorVelInit;
  float linearDC = StopDC;
  float angularDC = StopDC;
  float armTiltDC = StopDC;
  float armExtendDC = StopDC;
  float drillMotorDC = StopDC;
  float binMotorDC = StopDC;
  float linearPW_US = rcPWMPeriodMidUS;
  float angularPW_US = rcPWMPeriodMidUS;
  float armTiltPW_US = rcPWMPeriodMidUS;
  float armExtendPW_US = rcPWMPeriodMidUS;
  float drillMotorPW_US = rcPWMPeriodMidUS;
  float binMotorPW_US = rcPWMPeriodMidUS;
}


// Encoder: used to get feedback on how far the wheel's travel
Encoder encoderLeft(encoderLeftPinA, encoderLeftPinB);
Encoder encoderRight(encoderRightinA, encoderRightPinB);

// Motors *Using Servo library to send pulse width (ms) rather than duty cycle
Servo linear;
Servo angular;
Servo armTilt;
Servo drillMotor;
Servo binMotor;

boolean newData = false;

const byte numChars = 32;
char receivedChars[numChars];
char tempChars[numChars];        // temporary array for use when parsing

      // variables to hold the parsed data
// char messageFromPC[numChars] = {'a'};
// int integerFromPC = 0;
// float linearDC = 0.0;
// float angularDC = 0.0;

void setup() {
  // Setup Serial: *Note this must match in the Python Script
  Serial.begin(9600);

  initMsgValues();
  sendDataToSerial();

  // Setup Hardware Pins:
  linear.attach(linearPin);
  angular.attach(angularPin);
  armTilt.attach(armTiltPin);
  drillMotor.attach(drillMotorPin);
  binMotor.attach(binMotorPin);
  pinMode(switchPin, OUTPUT);

  // Intialize Hardware Pins:
  linear.writeMicroseconds(rcPWMPeriodMidUS);  // set servo to mid-point
  angular.writeMicroseconds(rcPWMPeriodMidUS);  // set servo to mid-point
  armTilt.writeMicroseconds(rcPWMPeriodMidUS);  // set servo to mid-point
  drillMotor.writeMicroseconds(rcPWMPeriodMidUS);  // set servo to mid-point
  binMotor.writeMicroseconds(rcPWMPeriodMidUS);  // set servo to mid-point
  digitalWrite(switchPin, HIGH);

}

void loop() {
  long newLeft, newRight;
  newLeft = encoderLeft.read();
  newRight = encoderRight.read();
  // Check if the encoder position has changed, if so, update the encoder position
  if (newLeft != encoderLeftVal || newRight != encoderRightVal) {
    encoderLeftVal = newLeft;
    encoderRightVal = newRight;
    sendDataToSerial();
  }
  
  recvWithStartEndMarkers();
  if (newData == true) {
      strcpy(tempChars, receivedChars);
          // this temporary copy is necessary to protect the original data
          //   because strtok() used in parseReceivedData() replaces the commas with \0
      parseReceivedData();
      writeDataToHardware();
      sendDataToSerial();
      newData = false;
  }
  // if a character is sent from the serial monitor,
  // reset both back to zero.
//   if (Serial.available()) {
//     Serial.read();
//     Serial.println("Reset both motors to zero");
//     encoderLeft.write(0);
//     encoderRight.write(0);
//   }
}

void resetEncoders(){
  encoderLeft.write(0);
  encoderRight.write(0);
}

void recvWithStartEndMarkers() {
    static boolean recvInProgress = false;
    static byte ndx = 0;
    char startMarker = '<';
    char endMarker = '>';
    char rc;

    while (Serial.available() > 0 && newData == false) {
        rc = Serial.read();

        if (recvInProgress == true) {
            if (rc != endMarker) {
                receivedChars[ndx] = rc;
                ndx++;
                if (ndx >= numChars) {
                    ndx = numChars - 1;
                }
            }
            else {
                receivedChars[ndx] = '\0'; // terminate the string
                recvInProgress = false;
                ndx = 0;
                newData = true;
            }
        }

        else if (rc == startMarker) {
            recvInProgress = true;
        }
    }
}

void parseReceivedData() {      // split the data into its parts

    char * strtokIndx; // this is used by strtok() as an index

    /*IMPORTANT NOTE: ORDER of Data Being Recieved. This MUST match the ORDER of Data on the Transmitter 
      dataFieldsRx = [switch_control,
                      linearVel, angularVel, armTiltVel, armExtendVel, drillMotorVel, binMotorVel]
    */

    /*IMPORTANT NOTE: ORDER of Data Being Recieved. This MUST match the ORDER of Data on the transmitter
      dataFieldsRx = [(int),
                      (float), (float), (float), (float), (float), (float)]
    */

    /*
    //How to parse a string
    strtokIndx = strtok(tempChars,",");      // get the first part - the string
    strcpy(messageFromPC, strtokIndx); // copy it to messageFromPC
    
    //How to parse an integer
    strtokIndx = strtok(NULL, ","); // this continues where the previous call left off
    integerFromPC = atoi(strtokIndx);     // convert this part to an integer
    
    //How to parse a float
    strtokIndx = strtok(NULL, ",");
    floatFromPC = atof(strtokIndx);     // convert this part to a float
    */

    /*Switch Relay Control Data */
    strtokIndx = strtok(tempChars,",");      // get the first part - the switch control data
    switch_control = atoi(strtokIndx);     // convert this part to an integer
    
    /*Drive Motor Control Data*/
    strtokIndx = strtok(NULL, ",");
    linearVel = atof(strtokIndx);     // convert this part to a float

    strtokIndx = strtok(NULL, ",");
    angularVel = atof(strtokIndx);     // convert this part to a float

    /*Duty Cycle (DC) to Pulse Width in nanoseconds (PW_US)*/
    /*Example: ((2000us - 1000us) / (100% - (-100%))) * 100% + 1500us = 2000us*/
    /*Example: ((2000us - 1000us) / (100% - (-100%))) * 0% + 1500us = 1500us*/
    /*Example: ((2000us - 1000us) / (100% - (-100%))) * -100% + 1500us = 1000us*/
    /*Example: ((MaxOfNewRange - MinOfNewRange) / (MaxOfOldRange - (MinOfOldRange))) * (ValueInOldRange) + MidOfNewRange = ValueInNewRange*/
    //linearDC = ((FwdMaxDC - BwdMaxDC) / (driveMotorMaxVel - (-1*driveMotorMaxVel))) * (linearVel) + StopDC;
    linearDC = convertToNewRange(linearVel,(-1*driveMotorMaxVel),driveMotorMaxVel,BwdMaxDC,FwdMaxDC);
    //angularDC = ((FwdMaxDC-BwdMaxDC) / (driveMotorMaxVel-(-1*driveMotorMaxVel))) * angularVel + StopDC;
    angularDC = convertToNewRange(angularVel,(-1*driveMotorMaxVel),driveMotorMaxVel,BwdMaxDC,FwdMaxDC);

    /*Duty Cycle (DC) to Pulse Width in nanoseconds (PW_US)*/
    //linearPW_US = ((rcPWMPeriodMaxUS-rcPWMPeriodMinUS) / (FwdMaxDC-(BwdMaxDC))) * linearDC + rcPWMPeriodMidUS;
    //angularPW_US = ((rcPWMPeriodMaxUS-rcPWMPeriodMinUS) / (FwdMaxDC-(BwdMaxDC))) * angularDC + rcPWMPeriodMidUS;
    linearPW_US = convertToNewRange(linearDC,BwdMaxDC,FwdMaxDC,rcPWMPeriodMinUS,rcPWMPeriodMaxUS);
    angularPW_US = convertToNewRange(angularDC,BwdMaxDC,FwdMaxDC,rcPWMPeriodMinUS,rcPWMPeriodMaxUS);

    /*Arm Tilt Linear Actuator Control Data*/
    strtokIndx = strtok(NULL, ",");
    armTiltVel = atof(strtokIndx);     // convert this part to a float
    armTiltDC = convertToNewRange(armTiltVel,(-1*armTiltMaxVel),armTiltMaxVel,BwdMaxDC,FwdMaxDC);
    armTiltPW_US = convertToNewRange(armTiltDC,BwdMaxDC,FwdMaxDC,rcPWMPeriodMinUS,rcPWMPeriodMaxUS);

    /*Arm Extend Stepper Control Data*/
    strtokIndx = strtok(NULL, ",");
    armExtendVel = atof(strtokIndx);     // convert this part to a float
    armExtendDC = convertToNewRange(armExtendVel,(-1*armExtendMaxVel),armExtendMaxVel,BwdMaxDC,FwdMaxDC);
    armExtendPW_US = convertToNewRange(armExtendDC,BwdMaxDC,FwdMaxDC,rcPWMPeriodMinUS,rcPWMPeriodMaxUS);

    /*Drill Motor Control Data*/
    strtokIndx = strtok(NULL, ",");
    drillMotorVel = atof(strtokIndx);     // convert this part to a float
    drillMotorDC = convertToNewRange(drillMotorVel,(-1*binMotorMaxVel),binMotorMaxVel,BwdMaxDC,FwdMaxDC);
    drillMotorPW_US = convertToNewRange(drillMotorDC,BwdMaxDC,FwdMaxDC,rcPWMPeriodMinUS,rcPWMPeriodMaxUS);

    /*Bin Motor Control Data*/
    strtokIndx = strtok(NULL, ",");
    binMotorVel = atof(strtokIndx);     // convert this part to a float
    binMotorDC = convertToNewRange(binMotorVel,(-1*binMotorMaxVel),binMotorMaxVel,BwdMaxDC,FwdMaxDC);
    binMotorPW_US = convertToNewRange(binMotorDC,BwdMaxDC,FwdMaxDC,rcPWMPeriodMinUS,rcPWMPeriodMaxUS);

}

float convertToNewRange(float(ValueInOldRange),float(OldRangeMin),float(OldRangeMax),float(NewRangeMin),float(NewRangeMax)){
  float ValueInNewRange = -999999999;
  float MidNewRange = (NewRangeMax + NewRangeMin) / 2;
  ValueInNewRange = ((NewRangeMax - NewRangeMin) / (OldRangeMax - (OldRangeMin))) * (ValueInOldRange) + MidNewRange;
  return ValueInNewRange;
}

void writeDataToHardware(){
  
  /**/

  /*Switch Control*/
  if(switch_control == 0){
    digitalWrite(switchPin,LOW);
  }
  else if(switch_control == 1){
    digitalWrite(switchPin, HIGH);
  }
  else{
    digitalWrite(switchPin, LOW);
  }
  
  /*Drive Motor Control*/
  if ((linearPW_US == 0) && (angularPW_US == 0)){
    linear.writeMicroseconds(0);
    angular.writeMicroseconds(0);
  }
  else{
    linear.writeMicroseconds(linearPW_US);
    angular.writeMicroseconds(angularPW_US);
  }

  /*Arm Tilt Motor Control*/
  if (armTiltPW_US == 0){
    armTilt.writeMicroseconds(0);
  }
  else{
    armTilt.writeMicroseconds(armTiltPW_US);
  }
  /*Arm Extend Stepper Motor Control*/
  //TODO: Create stepper motor logic

  /*Drill Motor Control*/
  if (drillMotorPW_US == 0){
    drillMotor.writeMicroseconds(0);
  }
  else{
    drillMotor.writeMicroseconds(drillMotorPW_US);
  }
  /*Bin Motor Control*/
  if (binMotorPW_US == 0){
    binMotor.writeMicroseconds(0);
  }
  else{
    binMotor.writeMicroseconds(binMotorPW_US);
  }
}

void sendDataToSerial() {
    
    /*IMPORTANT NOTE: ORDER of Data Being Sent. This MUST match the python code on the receiver 
      dataFieldsTx = [switch_control ,
                    encoderLeftVal ,
                    encoderRightVal ,
                    linearVel ,
                    angularVel ,
                    armTiltVel ,
                    armExtendVel ,
                    drillMotorVel ,
                    binMotorVel, 
                    linearDC ,
                    angularDC ,
                    armTiltDC ,
                    armExtendDC ,
                    drillMotorDC ,
                    binMotorDC ,
                    linearPW_US ,
                    angularPW_US ,
                    armTiltPW_US ,
                    armExtendPW_US ,
                    drillMotorPW_US,
                    binMotorPW_US,
                     ]
    */
    /* 
      //Message sending template w/ labels:
      Serial.print(" "); Serial.print(); Serial.print(" ");
      Serial.print(" "); Serial.print(); Serial.print(" ");
      
      //Message sending template w/o labels:
      Serial.print(); Serial.print(" ");
      Serial.print(); Serial.print(" ");
    */
    
    /*Serial.print("switch_control ");*/ Serial.print(switch_control); Serial.print(" ");
    /*Serial.print("encoderLeftVal ");*/ Serial.print(encoderLeftVal); Serial.print(" ");
    /*Serial.print("encoderRightVal ");*/ Serial.print(encoderRightVal); Serial.print(" ");
    /*Serial.print("linearVel ");*/ Serial.print(linearVel); Serial.print(" ");
    /*Serial.print("angularVel ");*/ Serial.print(angularVel); Serial.print(" ");
    /*Serial.print("armTiltVel ");*/ Serial.print(armTiltVel); Serial.print(" ");
    /*Serial.print("armExtendVel ");*/ Serial.print(armExtendVel); Serial.print(" ");
    /*Serial.print("drillMotorVel ");*/ Serial.print(drillMotorVel); Serial.print(" ");
    /*Serial.print("binMotorVel ");*/ Serial.print(binMotorVel); Serial.print(" ");
    /*Serial.print("linearDC ");*/ Serial.print(linearDC); Serial.print(" ");
    /*Serial.print("angularDC ");*/ Serial.print(angularDC); Serial.print(" ");
    /*Serial.print("armTiltDC ");*/ Serial.print(armTiltDC); Serial.print(" ");
    /*Serial.print("armExtendDC ");*/ Serial.print(armExtendDC); Serial.print(" ");
    /*Serial.print("drillMotorDC ");*/ Serial.print(drillMotorDC); Serial.print(" ");
    /*Serial.print("binMotorDC ");*/ Serial.print(binMotorDC); Serial.print(" ");
    /*Serial.print("linearPW_US ");*/ Serial.print(linearPW_US); Serial.print(" ");
    /*Serial.print("angularPW_US ");*/ Serial.print(angularPW_US); Serial.print(" ");
    /*Serial.print("armTiltPW_US ");*/ Serial.print(armTiltPW_US); Serial.print(" ");
    /*Serial.print("armExtendPW_US ");*/ Serial.print(armExtendPW_US); Serial.print(" ");
    /*Serial.print("drillMotorPW_US ");*/ Serial.print(drillMotorPW_US); Serial.print(" ");
    /*Serial.print("binMotorPW_US ");*/ Serial.print(binMotorPW_US); //Lastline is a println() as the newline ('\n') is the endline symbol
    Serial.println();
}