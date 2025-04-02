/* 
 * This code was edited by Christopher Payne on 1 April 2025
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
static int leftMotorPin = 10;
static int rightMotorPin = 9;
static int powerPin = 11;

// Setting up rcPWM
unsigned long previousMillis = 0;
float FwdMax = 1;
float Stop = 0;
float BwdMax = -1;
float rcPWMPeriodMaxUS = 2000;
float rcPWMPeriodMidUS = 1500;
float rcPWMPeriodMinUS = 1000;

int leftMotorPW_US = rcPWMPeriodMidUS;
int rightMotorPW_US = rcPWMPeriodMidUS;

// Encoder: used to get feedback on how far the wheel's travel
Encoder encoderLeft(encoderLeftPinA, encoderLeftPinB);
Encoder encoderRight(encoderRightinA, encoderRightPinB);
// Motors *Using Servo library to send pulse width (ms) rather than duty cycle
Servo leftMotor;
Servo rightMotor;

boolean newData = false;

const byte numChars = 32;
char receivedChars[numChars];
char tempChars[numChars];        // temporary array for use when parsing

      // variables to hold the parsed data
char messageFromPC[numChars] = {'a'};
int integerFromPC = 0;
float leftMotorDC = 0.0;
float rightMotorDC = 0.0;

void setup() {
  // Setup Serial: *Note this must match in the Python Script
  Serial.begin(9600);

  // Setup Hardware Pins:
  leftMotor.attach(leftMotorPin);
  rightMotor.attach(rightMotorPin);
  pinMode(powerPin, OUTPUT);
  leftMotor.writeMicroseconds(rcPWMPeriodMidUS);  // set servo to mid-point
  rightMotor.writeMicroseconds(rcPWMPeriodMidUS);  // set servo to mid-point
  digitalWrite(powerPin, HIGH);

}

long positionLeft  = -999;
long positionRight = -999;

void loop() {
  long newLeft, newRight;
  newLeft = encoderLeft.read();
  newRight = encoderRight.read();
  // Check if the encoder position has changed, if so, update the encoder position
  if (newLeft != positionLeft || newRight != positionRight) {
    // Serial.print("Left = ");
    // Serial.print(newLeft);
    // Serial.print(", Right = ");
    // Serial.print(newRight);
    // Serial.println();
    positionLeft = newLeft;
    positionRight = newRight;
    sendData();
  }
  
  recvWithStartEndMarkers();
  if (newData == true) {
      strcpy(tempChars, receivedChars);
          // this temporary copy is necessary to protect the original data
          //   because strtok() used in parseData() replaces the commas with \0
      parseData();
      writeData();
      sendData();
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

void parseData() {      // split the data into its parts

    char * strtokIndx; // this is used by strtok() as an index

    strtokIndx = strtok(tempChars,",");      // get the first part - the string
    strcpy(messageFromPC, strtokIndx); // copy it to messageFromPC
 
    // strtokIndx = strtok(NULL, ","); // this continues where the previous call left off
    // integerFromPC = atoi(strtokIndx);     // convert this part to an integer
    strtokIndx = strtok(NULL, ",");
    leftMotorDC = atof(strtokIndx);     // convert this part to a float

    strtokIndx = strtok(NULL, ",");
    rightMotorDC = atof(strtokIndx);     // convert this part to a float

    leftMotorPW_US = ((rcPWMPeriodMaxUS-rcPWMPeriodMinUS) / (FwdMax-(BwdMax))) * leftMotorDC + rcPWMPeriodMidUS;
    rightMotorPW_US = ((rcPWMPeriodMaxUS-rcPWMPeriodMinUS) / (FwdMax-(BwdMax))) * rightMotorDC + rcPWMPeriodMidUS;

}

void writeData(){
  if ((leftMotorPW_US == 0) && (rightMotorPW_US == 0)){
    digitalWrite(powerPin, LOW);
    leftMotor.writeMicroseconds(0);
    rightMotor.writeMicroseconds(0);
  }
  else{
    digitalWrite(powerPin, HIGH);
    leftMotor.writeMicroseconds(leftMotorPW_US);
    rightMotor.writeMicroseconds(rightMotorPW_US);
  }
}

void sendData() {
    Serial.print("Message ");
    Serial.print(messageFromPC);
    Serial.print(" ");
    Serial.print("leftMotorDC ");
    Serial.print(leftMotorDC);
    Serial.print(" ");
    Serial.print("leftMotorPW_US ");
    Serial.print(leftMotorPW_US);
    Serial.print(" ");
    Serial.print("rightMotorDC ");
    Serial.print(rightMotorDC);
    Serial.print(" ");
    Serial.print("rightMotorPW_US ");
    Serial.print(rightMotorPW_US);
    Serial.print(" ");
    Serial.print("encoderLeft ");
    Serial.print(positionLeft);
    Serial.print(" ");
    Serial.print("encoderRight ");
    Serial.print(positionRight);
    // Serial.print(" ");
    Serial.println();
}