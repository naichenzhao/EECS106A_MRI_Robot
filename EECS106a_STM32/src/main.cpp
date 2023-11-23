#include <Arduino.h>

#include "encoder.h"
#include "motors.h"


//  + -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- - +
//  | Global Variables
//  + -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- - +

// Position for each motor -> We will use a scale from 0 to 100000
int positions[6];
int counter = 0;


//  + -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- - +
//  | Global Functions
//  + -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- - +
void set_positions(String ser_read);



//  + -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- - +
//  | Run setup
//  + -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- - +
void setup() {
  Serial.begin(115200);
  Serial.println("---- Starting Setup ---- ");

  setupMotors(); // Setup stepper motors
  setupEncoders(); // setup encoders
  homeMotors(); // home the motors
  

  // Setup any other ports
  pinMode(13, OUTPUT);

  Serial.println("---- Finished Setup ---- ");
}

//  + -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- - +
//  | Run main loop
//  + -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- - +

void loop() {
  if (Serial.available()) {
    // get string of values
    String ser_read = Serial.readStringUntil('\n');

    // Flush the serial port just incase
    Serial.flush();

    char ind = ser_read.charAt(0);

    switch (ind) {
    case 'p':
      set_positions(ser_read.substring(1));
      break;
    }

    // Confirm revieved values
    Serial.print("recievd: ");
    Serial.println(ind);
  }

  if (counter >= 1000) {
    // Serial.println(get_x());
    // Serial.println(get_y());

    // Serial.println(" ");
    // // print_motor_x();
    // counter = 0;
  }
  counter ++;
 
  // Serial.println(digitalRead(PA0));
  // delay(100);

  run_motors();
}


//  + -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- - +
//  | Any functions are listed below
//  + -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- - +

void set_positions(String ser_read) {
  // Convert the input into a char array
  char *str = (char *)ser_read.c_str();
  char *token = strtok(str, ", ");

  // Split the string into ints
  int counter = 0;
  while (token != NULL)
  {
    int value = ((String)token).toInt();
    positions[counter] = (value == -1) ? positions[counter] : value;
    token = strtok(NULL, ", ");
    counter++;
  }

  set_motors(positions);
}