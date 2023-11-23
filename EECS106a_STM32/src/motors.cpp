// motor.cpp
#include <Arduino.h>
#include <AccelStepper.h>

#include "motors.h"
#include "encoder.h"

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
const int MAX_ACCELERATION = 5000;

const int homing_step = 1000;
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

AccelStepper *STEPPERS[6] = {
    &x_stepper,
    &y_stepper,
    &z_stepper,
    &r_stepper,
    &a1_stepper,
    &a2_stepper,
};

typedef long (*encoder_pointer)();
encoder_pointer ENCODERS[6] = {
    get_x,
    get_y,
    get_z,
    get_r,
    get_a1,
    get_a2,
};

//  + -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- - +
//  | Setup Variables
//  + -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- - +

int MAX_POS[6] = {26000, 14000, 80000, 10000, 10000, 10000}; // Max step position for each motor
float RATIO[6] = {1.5, 1.5, 1, 1, 1, 1}; // Conversion ratio -> step * ratio = encoder
int MOVEMENT_MODE[6] = {0, 0, 1, 0, 0, 0}; // Movement type -> [0] for open loop, [1] for encoder PID

// Target position for motor to move towards
int TARGET_POS[6] = {0, 0, 0, 0, 0, 0};

// Global variables for PID control
long last_times[6] = {0, 0, 0, 0, 0, 0};
long last_positions[6] = {0, 0, 0, 0, 0, 0};
float error_i[6] = {0, 0, 0, 0, 0, 0};

float Kp[6] = {2, 2, 2, 2, 2, 2};
float Ki[6] = {0.5, 0.5, 0.5, 0.5, 0.5, 0.5};
float Kd[6] = {0.1, 0.1, 0.1, 0.1, 0.1, 0.1};



//  + -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- - +
//  | Stepper Motor
//  + -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- - +

void setupMotors() {
    // set input for all homing pins
    pinMode(X_HOME, INPUT);
    pinMode(Y_HOME, INPUT);
    pinMode(Z_HOME, INPUT);
    pinMode(R_HOME, INPUT);
    pinMode(A1_HOME, INPUT);
    pinMode(A2_HOME, INPUT);
}

void homeMotors() {
    homeSingleStepperEncoder(0);
    reset_x();
    homeSingleStepperEncoder(1);
    reset_y();
    homeSingleStepperEncoder(2);
    reset_z();

    // setup constant values for steppers
    for (int i = 0; i < 6; i++) {
        STEPPERS[i]->setMaxSpeed(MAX_SPEED);
        STEPPERS[i]->setAcceleration(MAX_ACCELERATION);
    }
}

void set_motors(int positions[]) {
    // Takes in array of length 6, each it the position of a stepper motor
    for (int i = 0; i < 6; i++) {
        TARGET_POS[i] = (positions[i] * MAX_POS[i]) / 100000;
        error_i[i] = 0;
    }
}

void set_x_motor(int pos) {
    TARGET_POS[0] = (pos * MAX_POS[0]) / 100000;
    error_i[0] = 0;
}
void set_y_motor(int pos) {
    TARGET_POS[1] = (pos * MAX_POS[1]) / 100000;
    error_i[1] = 0;
}
void set_z_motor(int pos) {
    TARGET_POS[2] = (pos * MAX_POS[2]) / 100000;
    error_i[2] = 0;
}
void set_r_motor(int pos) {
    TARGET_POS[3] = (pos * MAX_POS[3]) / 100000;
    error_i[3] = 0;
}
void set_a1_motor(int pos) {
    TARGET_POS[4] = (pos * MAX_POS[4]) / 100000;
    error_i[4] = 0;
}
void set_a2_motor(int pos) {
    TARGET_POS[5] = (pos * MAX_POS[5]) / 100000;
    error_i[5] = 0;
}


void run_motors() {
    for (int i = 0; i < 6; i++) {
        if (MOVEMENT_MODE[i]) 
            runStepperCL(i);
        else
            runStepperOL(i);
    }
}



//  + -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- - +
//  | Motor Movement
//  + -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- - +

void runStepperOL(int num){
    AccelStepper *stepper = STEPPERS[num];
    stepper->moveTo(TARGET_POS[num]);

    if (stepper->currentPosition() != stepper->targetPosition())
        stepper->run();
    else
        stepper->stop();
}

void runStepperCL(int num) {
    AccelStepper *stepper = STEPPERS[num];
    long current_pos = ENCODERS[num]();
    long current_time = millis();

    // Calculate PID errors
    float error_p = TARGET_POS[num] * RATIO[num] - current_pos;
    float error_d = (current_pos - last_positions[num]) / (current_time - last_times[num]);
    error_i[num] = error_p + 0.9 * error_i[num];

    // Update previosu timestep's values
    last_positions[num] = current_pos;
    last_times[num] = current_time;

    // get PIC control value
    float PID_val = Kp[num] * error_p + Ki[num] * error_i[num] + Kd[num] * error_d;

    // Set values
    if (quick_abs( (long) PID_val) > MAX_SPEED) { // Make sure we are within max speed bounds
        stepper->setSpeed(PID_val > 0 ? MAX_SPEED : -MAX_SPEED);
    } else if (quick_abs( (long) PID_val) < 50) { // Mpve towards the target
        stepper->setSpeed((int)PID_val);
        stepper->run();
    } else { // This is for if we have reached it
        stepper->stop();
    }
        
}



//  + -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- - +
//  | Motor Homing
//  + -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- - +

void homeSingleStepperSwitch(int num, int switch_port){
    AccelStepper *stepper = STEPPERS[num];
    while (!digitalRead(switch_port)) {
        stepper->moveTo(stepper->currentPosition() - homing_step);
        stepper->run();
        delay(5);
    }
    stepper->stop();

    while (digitalRead(switch_port)) {
        stepper->moveTo(stepper->currentPosition() + homing_backoff_step);
        stepper->run();
        delay(1);
    }
    stepper->stop();
    stepper->setCurrentPosition(0);
}

void homeSingleStepperEncoder(int num) {
    AccelStepper *stepper = STEPPERS[num];
    long last_pos = 0;
    long encoder_diff = 0;

    do {
        for(int i = 0; i < 8; i++) {
            stepper->setSpeed(-2000);
            stepper->run();
            delay(1);
        }
        encoder_diff = ENCODERS[num]() - last_pos;
        last_pos = ENCODERS[num]();
        Serial.println(ENCODERS[num]());

    } while (quick_abs(encoder_diff) >= 2);

    stepper->stop();
    stepper->setCurrentPosition(0);
}

long quick_abs(long value) {
    return (value > 0)? value:-value;
}
