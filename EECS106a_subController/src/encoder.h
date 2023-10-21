// encoder.h
#ifndef ENCODER_h
#define ENCODER_h

#include <Arduino.h>

void setupEncoders();
void resetEncoders();

void reset_x();
void reset_y();
void reset_z();
void reset_r();
void reset_a1();
void reset_a2();

long get_x();
long get_y();
long get_z();
long get_r();
long get_a1();
long get_a2();

void pulseX();
void pulseY();
void pulseZ();
void pulseR();
void pulseA1();
void pulseA2();

#endif