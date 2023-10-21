// encoder.cpp
#include "encoder.h"

//  + -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- - +
//  | Define ports for encoders
//  + -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- - +

#define X_A PA0
#define X_B PA1

#define Y_A PA4
#define Y_B PB0

#define Z_A PC1
#define Z_B PC0

#define R_A PC3
#define R_B PC2

#define A1_A PC15
#define A1_B PC14

#define A2_A PC13
#define A2_B PC7

volatile long X_COUNTER = 0;
volatile long Y_COUNTER = 0;
volatile long Z_COUNTER = 0;
volatile long R_COUNTER = 0;
volatile long A1_COUNTER = 0;
volatile long A2_COUNTER = 0;



//  + -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- - +
//  | Encoder Functions
//  + -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- - +

void setupEncoders() {
    // Set all A ports to internal pullup
    pinMode(X_A, INPUT_PULLUP);
    pinMode(Y_A, INPUT_PULLUP);
    pinMode(Z_A, INPUT_PULLUP);
    pinMode(R_A, INPUT_PULLUP);
    pinMode(A1_A, INPUT_PULLUP);
    pinMode(A2_A, INPUT_PULLUP);

    // Attach respective interrupts to each encoder
    attachInterrupt(digitalPinToInterrupt(X_A), pulseX, RISING);
    attachInterrupt(digitalPinToInterrupt(Y_A), pulseY, RISING);
    attachInterrupt(digitalPinToInterrupt(Z_A), pulseZ, RISING);
    attachInterrupt(digitalPinToInterrupt(R_A), pulseR, RISING);
    attachInterrupt(digitalPinToInterrupt(A1_A), pulseA1, RISING);
    attachInterrupt(digitalPinToInterrupt(A2_A), pulseA2, RISING);
}

// Overall reset function
void resetEncoders() {
    X_COUNTER = 0;
    Y_COUNTER = 0;
    Z_COUNTER = 0;
    R_COUNTER = 0;
    A1_COUNTER = 0;
    A2_COUNTER = 0;
}

// Reset function for each encoder
void reset_x() {
    X_COUNTER = 0;
}
void reset_y() {
    Y_COUNTER = 0;
}
void reset_z() {
    Z_COUNTER = 0;
}
void reset_r() {
    R_COUNTER = 0;
}
void reset_a1() {
    A1_COUNTER = 0;
}
void reset_a2() {
    A2_COUNTER = 0;
}

// getters for each encoder
long get_x() {
    return X_COUNTER;
}
long get_y() {
    return Y_COUNTER;
}
long get_z() {
    return Z_COUNTER;
}
long get_r() {
    return R_COUNTER;
}
long get_a1() {
    return A1_COUNTER;
}
long get_a2() {
    return A2_COUNTER;
}



//  + -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- - +
//  | Encoder Interrupt Functions
//  + -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- - +

void pulseX() {
    X_COUNTER += digitalRead(X_B) ? 1 : -1;
}
void pulseY() {
    Y_COUNTER += digitalRead(Y_B) ? 1 : -1;
}
void pulseZ() {
    Z_COUNTER += digitalRead(Z_B) ? 1 : -1;
}
void pulseR() {
    R_COUNTER += digitalRead(R_B) ? 1 : -1;
}
void pulseA1() {
    A1_COUNTER += digitalRead(A1_B) ? 1 : -1;
}
void pulseA2() {
    A2_COUNTER += digitalRead(A2_B) ? 1 : -1;
}
