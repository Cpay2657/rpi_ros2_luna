#include <IBusBM.h>
#include <Servo.h>
#include <string.h>
#include <AccelStepper.h>

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

// Hardware Pins:
static int iBusPin = 0; // Serial Rx **NOTE: Disconnect during upload**
static int switchPin = 13; // Not implemmented
static int linearPin = 11; // CH 2 on RC Tx
static int angularPin = 10; // CH 1 on RC Tx
static int armExtendPUL = 2; // CH 3
static int armExtendDIR = 3; // CH 3
static int armTiltPin = 9; // CH 4
static int drillMotorPin = 6; // CH 5
static int binMotorPin = 5; // CH 6

//Pre-defined Values:
//For Stepper Motor
int stepPerRev = 6400; // All dip switches LOW for Stepper Motor Driver
float revPerSec = 2.0; // Desired Max RPS (revolutions per second for the steppers)
float stepPerSec = stepPerRev*revPerSec; // Desired Max Steps per Second
int minAbsoluteSteps = 0; // Home or Docked Position
int maxAbsoluteSteps = 22000; // Fully Extended Position
int BUFFER = 10;

// Setting up rcPWM
unsigned long previousMillis = 0;
float rcPWMPeriodMaxUS = 2000;
float rcPWMPeriodMidUS = 1500;
float rcPWMPeriodMinUS = 1000;

IBusBM IBus; // IBus object
// Motors *Using Servo library to send pulse width (ms) rather than duty cycle
Servo linear;
Servo angular;
Servo armTilt;
Servo drillMotor;
Servo binMotor;
// Stepper Motor is controlled by a Driver (PUL/STEP = Pin 2; DIR = Pin 3 (High is FWD))
AccelStepper stepper(AccelStepper::DRIVER);

void setup() {
  Serial.begin(115200); //Required for IBus Reading from Arduino

  while (!Serial)
    ;
    
  IBus.begin(Serial);    // iBUS connected to Serial (Pin 0 - "RX")- change to Serial0 or Serial2 port when required

  //Setup for Stepper Motor:
  stepper.setMaxSpeed(stepPerSec); // Set max speed
  stepper.setAcceleration(.1*stepPerSec);
  stepper.setCurrentPosition(0); //Set the current position as 0

  // Setup Hardware Pins:
  linear.attach(linearPin);
  angular.attach(angularPin);
  armTilt.attach(armTiltPin);
  drillMotor.attach(drillMotorPin);
  binMotor.attach(binMotorPin);
  pinMode(switchPin, OUTPUT);

  Serial.println("Start IBus2PWM");
}

int saveval[6] = {0};
int val[6] = {0};
int oldval2 = 1000;
long newpos;
char *channelLabels[] = {"Angular (CH1)", "Linear (CH2)", "ArmExtend (CH3)", "ArmTilt (CH4)", "Drill (CH5)", "Bin (CH6)"};
//Servo joints[5] = {linear, angular, armTilt, drillMotor, binMotor};

float convertToNewRange(float(ValueInOldRange),float(OldRangeMin),float(OldRangeMax),float(NewRangeMin),float(NewRangeMax)){
  float MidNewRange = (NewRangeMax + NewRangeMin) / 2;
  float ValueInNewRange = ((NewRangeMax - NewRangeMin) / (OldRangeMax - (OldRangeMin))) * (ValueInOldRange) - NewRangeMax;
  return ValueInNewRange;
}

void loop() {
  //val[i] = IBus.readChannel(i); // get latest value for servo channel 1
  for(int i = 0; i < 6; i++){
    val[i] = IBus.readChannel(i);
  }
  angular.writeMicroseconds(val[0]);
  linear.writeMicroseconds(val[1]);
  if(abs(val[2]-oldval2) > BUFFER){
    if ((val[2] > rcPWMPeriodMinUS + BUFFER) && (val[2] < rcPWMPeriodMaxUS - BUFFER)){
      newpos = convertToNewRange(val[2], rcPWMPeriodMinUS, rcPWMPeriodMaxUS, minAbsoluteSteps, maxAbsoluteSteps);
      stepper.moveTo(newpos);
    }
    else if (val[2] < (rcPWMPeriodMinUS + BUFFER)){
      stepper.moveTo(minAbsoluteSteps);
      newpos = minAbsoluteSteps;
    }
    else if (val[2] > (rcPWMPeriodMinUS - BUFFER)){
      stepper.moveTo(maxAbsoluteSteps);
      newpos = maxAbsoluteSteps;
    }
    oldval2 = val[2];
  }
  /*else{
    stepper.moveTo(0);
  }*/
  stepper.run();
  armTilt.writeMicroseconds(val[3]);
  drillMotor.writeMicroseconds(val[4]);
  binMotor.writeMicroseconds(val[5]);

  /*Serial.print(channelLabels[0]);Serial.print(": ");Serial.print(IBus.readChannel(0));Serial.print("\t"); // display new value in microseconds on PC
  Serial.print(channelLabels[1]);Serial.print(": ");Serial.print(IBus.readChannel(1));Serial.print("\t"); // display new value in microseconds on PC
  Serial.print(channelLabels[2]);Serial.print(": ");Serial.print(IBus.readChannel(2));Serial.print("\t"); // display new value in microseconds on PC
  Serial.print(channelLabels[3]);Serial.print(": ");Serial.print(IBus.readChannel(3));Serial.print("\t"); // display new value in microseconds on PC
  Serial.print(channelLabels[4]);Serial.print(": ");Serial.print(IBus.readChannel(4));Serial.print("\t"); // display new value in microseconds on PC
  Serial.print(channelLabels[5]);Serial.print(": ");Serial.print(IBus.readChannel(5));Serial.print("\t"); // display new value in microseconds on PC
  Serial.println();
  */
  /*if (saveval != val) {
    for(int i = 0; i < 6; i++){
      Serial.print(channelLabels[i]);Serial.print(": ");Serial.print(val[i]);Serial.print("\t"); // display new value in microseconds on PC    
    }
    Serial.println();
    Serial.println(newpos);
    memcpy(val, saveval, sizeof(val));  //saveval = val;    
  }*/
  
  //delay(100);
}
