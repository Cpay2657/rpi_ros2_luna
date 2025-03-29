//https://forum.arduino.cc/t/serial-input-basics-updated/382007

#include <Servo.h>

// Example 5 - Receive with start- and end-markers combined with parsing

const byte numChars = 32;
char receivedChars[numChars];
char tempChars[numChars];        // temporary array for use when parsing

      // variables to hold the parsed data
char messageFromPC[numChars] = {0};
int integerFromPC = 0;
float leftMotorDC = 0.0;
float rightMotorDC = 0.0;

// LED Pins
static int leftMotorPin = 10;
static int rightMotorPin = 9;
static int powerPin = 11;

// Motors *Using Servo library to send pulse width (ms) rather than duty cycle
Servo leftMotor;
Servo rightMotor;

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

boolean newData = false;

//============

void setup() {
    Serial.begin(9600);
    // Serial.println("This demo expects 3 pieces of data - text, an integer and a floating point value");
    // Serial.println("Enter data in this style <HelloWorld, 12, 24.7>  ");
    // Serial.println();

    leftMotor.attach(leftMotorPin);
    rightMotor.attach(rightMotorPin);
    pinMode(powerPin, OUTPUT);
    leftMotor.writeMicroseconds(rcPWMPeriodMidUS);  // set servo to mid-point
    rightMotor.writeMicroseconds(rcPWMPeriodMidUS);  // set servo to mid-point
    digitalWrite(powerPin, HIGH);
}

//============

void loop() {
    recvWithStartEndMarkers();
    if (newData == true) {
        strcpy(tempChars, receivedChars);
            // this temporary copy is necessary to protect the original data
            //   because strtok() used in parseData() replaces the commas with \0
        parseData();
        showParsedData();
        newData = false;
    }
    if (strcmp(messageFromPC, "off") != 0){
      digitalWrite(powerPin, HIGH);
      leftMotor.writeMicroseconds(leftMotorPW_US);
      rightMotor.writeMicroseconds(rightMotorPW_US);
    }
    else{
      digitalWrite(powerPin, LOW);
      leftMotor.writeMicroseconds(rcPWMPeriodMidUS);
      rightMotor.writeMicroseconds(rcPWMPeriodMidUS);
    }
}

//============

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

//============

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

//============

void showParsedData() {
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
    Serial.println();
}

