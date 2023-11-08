// encoder.h
#ifndef MOTORS_h
#define MOTORS_h

#include <AccelStepper.h>

void setupMotors();
void homeMotors();
void set_motors(int positions[]);
void run_motors();

void set_x_motor(int pos);
void set_y_motor(int pos);
void set_z_motor(int pos);
void set_r_motor(int pos);
void set_a1_motor(int pos);
void set_a2_motor(int pos);

void homeSingleStepper(AccelStepper *stepper, int switch_port);
void moveSingleStepper(AccelStepper *stepper, int pos);
void runSingleStepper(AccelStepper* stepper);


#endif