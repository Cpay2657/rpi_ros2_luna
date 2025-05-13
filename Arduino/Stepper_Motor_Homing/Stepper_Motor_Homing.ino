// ConstantSpeed.pde
// -*- mode: C++ -*-
//
// Shows how to run AccelStepper in the simplest,
// fixed speed mode with no accelerations
/// \author  Mike McCauley (mikem@airspayce.com)
// Copyright (C) 2009 Mike McCauley
// $Id: ConstantSpeed.pde,v 1.1 2011/01/05 01:51:01 mikem Exp mikem $

#include <AccelStepper.h>
int stepPerRev = 6400;
float revPerSec = 20;//200.0;
float MAXSPEED = stepPerRev*revPerSec;
// DRIVER mode (for A4988, DRV8825, etc.)
AccelStepper stepper(AccelStepper::DRIVER);
long MAXSTEPS = 22000;
long MINSTEPS = 0;

void setup() {
  stepper.setMaxSpeed(MAXSPEED);   // Increase max speed
  stepper.setAcceleration(.1*MAXSPEED);
  //stepper.setSpeed(speed);      // Run at faster constant speed
  Serial.begin(115200);
  Serial.println(stepper.currentPosition());
  //stepper.moveTo(6400);
}

void loop() {
  stepper.run();          // Keep running at set speed
  //Serial.println(stepper.currentPosition());
  // if a character is sent from the serial monitor,
  // reset both back to zero.
  if (Serial.available()>0) {
    String cmd = Serial.readStringUntil("\n");
    cmd.trim();
    if(cmd == "a"){
      Serial.print("Current Position: ");Serial.println(stepper.currentPosition()); 
    }
    else if (cmd == "d"){
      Serial.print("Stopping at: "); Serial.println(stepper.currentPosition());
      stepper.stop();      // Run at faster constant speed
    }
    else if (cmd == "b"){
      Serial.println("Current Position to 0");
      stepper.setCurrentPosition(0);
      Serial.print("Current Position: ");Serial.println(stepper.currentPosition()); 
      //stepper.setSpeed(speed);      // Run at faster constant speed
    }
    else if (cmd == "c"){
      int newpos = 64000;
      Serial.print("Moving to");Serial.println(newpos);
      stepper.moveTo(newpos);
      //stepper.setSpeed(speed);
    }
    else if (cmd == "w"){
      Serial.print("Moving (Speed):");Serial.println(MAXSPEED);
      //stepper.setSpeed(speed);
      stepper.moveTo(1000000000);
    }
    else if (cmd == "s"){
      Serial.print("Moving (Speed):");Serial.println(MAXSPEED);
      //stepper.setSpeed(-speed);
      stepper.moveTo(-1000000000);
    }
    else if (cmd == "f"){
      Serial.print("Moving to:");Serial.println(MAXSTEPS);
      stepper.moveTo(MAXSTEPS);
      //stepper.setSpeed(speed);
    }
    else if (cmd == "g"){
      Serial.print("Moving to:");Serial.println(MINSTEPS);
      stepper.moveTo(MINSTEPS);
      //stepper.setSpeed(-speed);
    }
  }
}
