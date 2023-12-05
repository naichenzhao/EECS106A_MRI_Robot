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

#define A2_PULSE PA6
#define A2_DIR PA7

#define LIN_HOME PB9
#define ROT_HOME PB6




// Max position to end at. This is set to stepper value of OL and encoder value for CL
// long MAX_POS[6] = {8000, 1000, 44000, 12568, 50000, 50000}; // Max step position for each motor
long MAX_POS[6] = {8000, 1150, 44000, 160000, 52000, 52000}; // Max step position for each motor
const long MAX_SPEED[6] = {700, 600, 3000, 5000, 1500, 1500};
const long MAX_ACCELERATION[6] = {40000, 40000, 40000, 40000, 40000, 40000};

const int homing_step = 10000;
const int homing_backoff_step = 1;
const long HOME_SPEED[6] = {-700, -700, -3500, 4000, 4000, 4000};
const long HOME_STEP[6] = {180, 300, 100, 0, 0, 0};

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
int MOVEMENT_MODE[6] = {1, 1, 1, 0, 0, 0}; // Movement type -> [0] for open loop, [1] for encoder PID

// Target position for motor to move towards
int TARGET_POS[6] = {0, 0, 0, 0, 0, 0};

// Global variables for PID control
long last_times[6] = {0, 0, 0, 0, 0, 0};
long last_positions[6] = {0, 0, 0, 0, 0, 0};
float error_i[6] = {0, 0, 0, 0, 0, 0};

float Kp[6] = {2, 2, 2, 0, 0, 0};
float Ki[6] = {0.5, 0.5, 0.5, 0, 0, 0};
float Kd[6] = {0.1, 0.1, 0.1, 0, 0, 0};

float Kw = 0.9;



//  + -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- - +
//  | Stepper Motor
//  + -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- - +

void setupMotors() {
    // set input for all homing pins
    pinMode(X_HOME, INPUT_PULLDOWN);
    pinMode(Y_HOME, INPUT_PULLDOWN);
    pinMode(Z_HOME, INPUT_PULLDOWN);
    pinMode(R_HOME, INPUT_PULLDOWN);
    pinMode(LIN_HOME, INPUT_PULLDOWN);
    pinMode(ROT_HOME, INPUT_PULLDOWN);
}

void homeMotors() {
    for (int i = 0; i < 6; i++) {
        STEPPERS[i]->setMaxSpeed(HOME_SPEED[i] + 1000);
        STEPPERS[i]->setAcceleration(MAX_ACCELERATION[i]);
    }

    // Home XYZ Gantry
    homeSingleStepperEncoder(0);
    homeSingleStepperEncoder(1);
    homeSingleStepperEncoder(2);

    // Home Rotationary Arm
    motor_goto(3, -30000);
    homeSingleStepperSwitch(3, R_HOME);
    delay(150);
    motor_goto(3, 4000);
    r_stepper.setCurrentPosition(0);
    reset_r();

    //Home Differential Drive
    homeDifferentialEncoder();
    delay(150);
    motor_goto(4, 10);
    motor_goto(5, 10);
    dgear_goto(26000, -26000);
    a1_stepper.setCurrentPosition(0);
    a2_stepper.setCurrentPosition(0);

    reset_x();
    reset_y();
    reset_z();

    // setup constant values for steppers
    for (int i = 0; i < 6; i++)
    {
        STEPPERS[i]->setMaxSpeed(MAX_SPEED[i] + 1000);
        STEPPERS[i]->setAcceleration(MAX_ACCELERATION[i]);
    }
}

void set_motors(long positions[]) {
    // Takes in array of length 6, each it the position of a stepper motor
    for (int i = 0; i < 6; i++) {
        TARGET_POS[i] = ((positions[i]) * MAX_POS[i]) / 10000;
        error_i[i] = 0;
        last_positions[i] = 0;
        Serial.print(TARGET_POS[i]);
        Serial.print("   ");
    }
    Serial.println("   ");
}

void set_x_motor(int pos) {
    TARGET_POS[0] = ((long)pos * MAX_POS[0]) / 10000;
    error_i[0] = 0;
}
void set_y_motor(int pos) {
    TARGET_POS[1] = ((long)pos * MAX_POS[1]) / 10000;
    error_i[1] = 0;
}
void set_z_motor(int pos) {
    TARGET_POS[2] = ((long)pos * MAX_POS[2]) / 10000;
    error_i[2] = 0;
}
void set_r_motor(int pos) {
    TARGET_POS[3] = ((long)pos * MAX_POS[3]) / 10000;
    error_i[3] = 0;
}
void set_a1_motor(int pos) {
    TARGET_POS[4] = ((long)pos * MAX_POS[4]) / 10000;
    error_i[4] = 0;
}
void set_a2_motor(int pos) {
    TARGET_POS[5] = ((long)pos * MAX_POS[5]) / 10000;
    error_i[5] = 0;
}

void motor_goto(int num, int target) {
    AccelStepper *stepper = STEPPERS[num];
    stepper->moveTo(target);

    while (stepper->currentPosition()  != target) {
        stepper->run();
    }
    stepper->stop();
}

void dgear_goto(int target1, int target2) {
    AccelStepper *stepper1 = STEPPERS[4];
    AccelStepper *stepper2 = STEPPERS[5];
    stepper1->moveTo(target1);
    stepper2->moveTo(target2);

    while (stepper1->currentPosition()  != target1 || stepper2->currentPosition()  != target2) {
        if (stepper1->currentPosition() != target1) {
            stepper1->run();
        } else {
            stepper1->stop();
        }
        if (stepper2->currentPosition() != target2) {
            stepper2->run();
        } else {
            stepper2->stop();
        }
            
    }
    stepper1->stop();
    stepper2->stop();
}


void run_motors() {
    for (int i = 0; i < 6; i++) {
        if (MOVEMENT_MODE[i]) 
            runStepperCL(i);
        else
            runStepperOL(i);
    }
}



long get_conv(int num) {
    long raw_val = 0;
    if (MOVEMENT_MODE[num]) {
        raw_val = ENCODERS[num]();
    } else {
        raw_val = get_val(num);
    }
    return (10000 * raw_val) / MAX_POS[num];
}

long get_val(int num) {
    AccelStepper *stepper = STEPPERS[num];
    return (long) stepper->currentPosition();
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
    float error_p = TARGET_POS[num] - current_pos;
    float error_d = (current_pos - last_positions[num]) / (current_time - last_times[num]);
    error_i[num] = error_p + (int)(Kw * error_i[num]);

    // Update previosu timestep's values
    last_positions[num] = current_pos;
    last_times[num] = current_time;

    // get PID control value
    int PID_val = (int) (Kp[num] * error_p + Ki[num] * error_i[num] + Kd[num] * error_d);

    // Set values
    if (PID_val > (MAX_SPEED[num]-100) || PID_val < -(MAX_SPEED[num]-100)) { // Make sure we are within max speed bounds
        stepper->setSpeed(PID_val > 0 ? MAX_SPEED[num]-100 : -(MAX_SPEED[num]-100));
        // stepper->setSpeed(PID_val);
        stepper->run();
    } else 
    
    if ((PID_val > 10 || PID_val < -10) && (error_p > 0 || error_p < -0)) { // Mpve towards the target
        stepper->setSpeed(PID_val);
        stepper->run();
    } else { // This is for if we have reached it
        stepper->stop();
    }

    // Serial.println("   ");
}



//  + -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- - +
//  | Motor Homing
//  + -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- - +

void homeSingleStepperSwitch(int num, int switch_port){
    AccelStepper *stepper = STEPPERS[num];
    // AccelStepper *stepper = &z_stepper;
    int counter = 0;
    while (counter < 10) {
        stepper->setSpeed(HOME_SPEED[num]);
        stepper->run();
        if (digitalRead(switch_port)) {
            counter ++;
        } else {
            counter = 0;
        }
    }
    stepper->stop();

    while (digitalRead(switch_port)) {
        stepper->setSpeed(-HOME_SPEED[num]/10);
        stepper->run();
    }
    stepper->stop();
    stepper->setCurrentPosition(0);
}

void homeSingleStepperEncoder(int num) {
    Serial.print("Homing stepper: ");
    Serial.println(num);

    AccelStepper *stepper = STEPPERS[num];
    long last_pos = 0;
    long curr_pos = 0;
    long encoder_diff = 0;

    do {
        for(int i = 0; i < HOME_STEP[num]; i++) {
            stepper->setSpeed(HOME_SPEED[num]);
            stepper->run();
            delayMicroseconds(300);
        }
        curr_pos = ENCODERS[num]();
        encoder_diff = curr_pos - last_pos;
        last_pos = curr_pos;

        // Serial.println(encoder_diff);
    } while (encoder_diff < 0);

    stepper->stop();
    stepper->setCurrentPosition(0);
}

void homeDifferentialEncoder() {
    Serial.println("Homing differential gear");

    AccelStepper *stepper1 = STEPPERS[4];
    AccelStepper *stepper2 = STEPPERS[5];

    Serial.println("Home Linear");
    int counter = 0;
    while (counter < 10) {
        stepper1->setSpeed(-HOME_SPEED[4]);
        stepper2->setSpeed(HOME_SPEED[5]);
        stepper1->run();
        stepper2->run();
        counter = digitalRead(LIN_HOME) ? counter+1:0;
    }
    while (digitalRead(LIN_HOME)) {
        stepper1->setSpeed(HOME_SPEED[4]/10);
        stepper2->setSpeed(-HOME_SPEED[5]/10);
        stepper1->run();
        stepper2->run();
    }
    stepper1->stop();
    stepper2->stop();

    Serial.println("Home Rotational");
    counter = 0;
    while (counter < 10) {
        stepper1->setSpeed(-HOME_SPEED[4]);
        stepper2->setSpeed(-HOME_SPEED[5]);
        stepper1->run();
        stepper2->run();
        counter = digitalRead(ROT_HOME) ? counter+1:0;
    }
    while (digitalRead(ROT_HOME)) {
        stepper1->setSpeed(HOME_SPEED[4]/10);
        stepper2->setSpeed(HOME_SPEED[5]/10);
        stepper1->run();
        stepper2->run();
    }
    stepper1->stop();
    stepper2->stop();

    stepper1->setCurrentPosition(0);
    stepper2->setCurrentPosition(0);
}

long quick_abs(long value) {
    return (value > 0)? value:-value;
}

//  + -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- - +
//  | Motor Movement
//  + -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- - +