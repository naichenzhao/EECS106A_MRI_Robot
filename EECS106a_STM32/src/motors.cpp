// motor.cpp
#include <Arduino.h>
#include <AccelStepper.h>

#include "motors.h"



//  + -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- - +
//  | Define ports for stepper motors and homing flags
//  + -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- - +
#define motorInterfaceType 1

#define X_PULSE PC8
#define X_DIR PC6
#define X_HOME PC5

#define Y_PULSE PA12
#define Y_DIR PA11
#define Y_HOME PB12

#define Z_PULSE PB2
#define Z_DIR PB1
#define Z_HOME PB15

#define R_PULSE PB14
#define R_DIR PB13
#define R_HOME PC4

#define A1_PULSE PC9
#define A1_DIR PB8
#define A1_HOME PB9

#define A2_PULSE PA6
#define A2_DIR PA7
#define A2_HOME PB6

const int MAX_SPEED = 10000;
const int MAX_ACCELERATION = 1000;

const int homing_step = 100;
const int homing_backoff_step = 1;

//  + -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- - +
//  | Setup Steppers
//  + -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- - +

// Steppers for the gantry
AccelStepper x_stepper = AccelStepper(motorInterfaceType, X_PULSE, X_DIR);
AccelStepper y_stepper = AccelStepper(motorInterfaceType, Y_PULSE, Y_DIR);
AccelStepper z_stepper = AccelStepper(motorInterfaceType, Z_PULSE, Z_DIR);

// Rotary stepper
AccelStepper r_stepper = AccelStepper(motorInterfaceType, R_PULSE, R_DIR);

// Steppers for the arm
AccelStepper a1_stepper = AccelStepper(motorInterfaceType, A1_PULSE, A1_DIR);
AccelStepper a2_stepper = AccelStepper(motorInterfaceType, A2_PULSE, A2_DIR);



//  + -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- - +
//  | Stepper Motor
//  + -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- - +

void setupMotors()
{
    // setup constant values for steppers
    x_stepper.setMaxSpeed(MAX_SPEED);
    x_stepper.setAcceleration(MAX_ACCELERATION);

    y_stepper.setMaxSpeed(MAX_SPEED);
    y_stepper.setAcceleration(MAX_ACCELERATION);

    z_stepper.setMaxSpeed(MAX_SPEED);
    z_stepper.setAcceleration(MAX_ACCELERATION);

    r_stepper.setMaxSpeed(MAX_SPEED);
    r_stepper.setAcceleration(MAX_ACCELERATION);

    a1_stepper.setMaxSpeed(MAX_SPEED);
    a1_stepper.setAcceleration(MAX_ACCELERATION);

    a2_stepper.setMaxSpeed(MAX_SPEED);
    a2_stepper.setAcceleration(MAX_ACCELERATION);

    // set input for all homing pins
    pinMode(X_HOME, INPUT);
    pinMode(Y_HOME, INPUT);
    pinMode(Z_HOME, INPUT);
    pinMode(R_HOME, INPUT);
    pinMode(A1_HOME, INPUT);
    pinMode(A2_HOME, INPUT);

}

void homeMotors() {
    homeSingleStepper(&x_stepper, X_HOME);
    homeSingleStepper(&y_stepper, Y_HOME);
    homeSingleStepper(&z_stepper, Z_HOME);
    homeSingleStepper(&r_stepper, R_HOME);
    homeSingleStepper(&a1_stepper, A1_HOME);
    homeSingleStepper(&a2_stepper, A2_HOME);
}

void set_motors(int positions[]) {
    // Takes in array of length 6, each it the position of a stepper motor
    moveSingleStepper(&x_stepper, positions[0]);
    moveSingleStepper(&y_stepper, positions[1]);
    moveSingleStepper(&z_stepper, positions[2]);
    moveSingleStepper(&r_stepper, positions[3]);
    moveSingleStepper(&a1_stepper, positions[4]);
    moveSingleStepper(&a2_stepper, positions[5]);
}

void set_x_motor(int pos) {
    moveSingleStepper(&x_stepper, pos);
}
void set_y_motor(int pos) {
    moveSingleStepper(&y_stepper, pos);
}
void set_z_motor(int pos) {
    moveSingleStepper(&z_stepper, pos);
}
void set_r_motor(int pos) {
    moveSingleStepper(&r_stepper, pos);
}
void set_a1_motor(int pos) {
    moveSingleStepper(&a1_stepper, pos);
}
void set_a2_motor(int pos) {
    moveSingleStepper(&a2_stepper, pos);
}




//  + -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- - +
//  | Helper Functions
//  + -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- - +

void moveSingleStepper(AccelStepper* stepper, int pos){
    stepper->moveTo(pos);

    while ((stepper->currentPosition() - pos) > 100 || (stepper->currentPosition() - pos) < -100)
    {
        stepper->run();
    }
}

void homeSingleStepper(AccelStepper* stepper, int switch_port){
    // home x_stepper
    while (!digitalRead(switch_port))
    {
        stepper->moveTo(stepper->currentPosition() - homing_step);
        stepper->run();
        delay(5);
    }
    stepper->stop();
    while (digitalRead(switch_port))
    {
        stepper->moveTo(stepper->currentPosition() + homing_backoff_step);
        stepper->run();
        delay(1);
    }
    stepper->stop();
    stepper->setCurrentPosition(0);
}