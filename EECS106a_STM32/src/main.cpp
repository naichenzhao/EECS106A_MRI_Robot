#include <Arduino.h>

#include "encoder.h"
#include "motors.h"


//  + -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- - +
//  | Global Variables
//  + -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- - +


int nums[6];


//  + -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- - +
//  | Run setup
//  + -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- - +

void setup() {
  // Setup stepper motors
  setupMotors();
  // homeMotors();

  // setup encoders
  setupEncoders();
  //resetEncoders();

  // Setup any other ports
  pinMode(13, OUTPUT);

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

  int angle1, angle2, angle3, angle4, angle5, angle6;
  if (Serial.available()) {
    // get string of values
    String ser_read = Serial.readStringUntil('\n');

    // Flush the serial port just incase
    Serial.flush();

    // Convert the input into a char array
    char *str = (char*)ser_read.c_str();
    char * token = strtok(str, ", ");
    
    // Split the string into ints
    int counter = 0;
    while( token != NULL ) {
        nums[counter] = ((String) token).toInt();
        token = strtok(NULL, ", ");
        counter ++;
    }

    // Confirm revieved values by printing them back
    Serial.println("recieved");
    Serial.println(nums[0]);
    Serial.println(nums[1]);
    Serial.println(nums[2]);
    Serial.println(nums[3]);
    Serial.println(nums[4]);
    Serial.println(nums[5]);


    digitalWrite(13, HIGH);
    delay(1000);
    digitalWrite(13, LOW);
  }
  

}


//  + -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- - +
//  | Any functions are listed below
//  + -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- - +

