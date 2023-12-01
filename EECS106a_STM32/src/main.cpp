#include <Arduino.h>

#include "encoder.h"
#include "motors.h"


//  + -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- - +
//  | Global Variables
//  + -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- - +

// Position for each motor -> We will use a scale from 0 to 100000
long positions[6];
int counter = 0;


//  + -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- - +
//  | Global Functions
//  + -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- - +
void set_positions(String ser_read);
void print_encoders();



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

    // set_x_motor(10000);
  }


  // Everything should be printed in here as to not interfere with stepper motors
  if (counter >= 20000) {
    print_encoders();
    counter = 0;
  }
  counter ++;
 
  // Serial.println(digitalRead(PA0));
  // delay(100);

  run_motors();
  // Serial.println("   ");
}


//  + -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- - +
//  | Any functions are listed below
//  + -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- - +


void print_encoders() {
  Serial.print("d");
  for (int i = 0; i < 6; i++) {
    Serial.print(get_conv(i));
    Serial.print(" ");
  }
  Serial.println("");
}

void set_positions(String ser_read) {
  // Convert the input into a char array
  char *str = (char *)ser_read.c_str();
  char *token = strtok(str, ", ");

  // Split the string into ints
  int counter = 0;
  while (token != NULL)
  {
    int value = ((String)token).toInt();
    value = value > 10000? 10000: value;
    positions[counter] = (value == -1) ? positions[counter] : value;
    token = strtok(NULL, ", ");
    counter++;
  }

  Serial.print(positions[0]);
  Serial.print("   ");
  Serial.print(positions[1]);
  Serial.print("   ");
  Serial.print(positions[2]);
  Serial.print("   ");
  Serial.print(positions[3]);
  Serial.print("   ");
  Serial.print(positions[4]);
  Serial.print("   ");
  Serial.println(positions[5]);

  set_motors(positions);
}