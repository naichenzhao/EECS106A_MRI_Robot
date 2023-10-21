#include <Arduino.h>

#include "encoder.h"
#include "motors.h"


//  + -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- - +
//  | Run setup
//  + -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- - +

void setup() {
  // Setup stepper motors
  setupMotors();
  // homeMotors();

  // setup encoders
  setupEncoders();
  resetEncoders();

  // begin serail port
  Serial.begin(115200);
}

//  + -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- - +
//  | Run main loop
//  + -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- - +

void loop() {
  // // Set the target position:
  // x_stepper.moveTo(3000);
  // // Run to target position with set speed and acceleration/deceleration:
  // x_stepper.runToPosition();

  Serial.println(get_x());
}


//  + -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- - +
//  | Any functions are listed below
//  + -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- - +

