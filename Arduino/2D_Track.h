#ifndef TWO_D_TRACK_H
#define TWO_D_TRACK_H

/* C++ STD Headers */
#include <array>
/* Arudunio Headers */
#include "esp32-hal-timer.h"
/* Application headers */


/******************************************/
/*           MACRO DEFINITIONS            */
/******************************************/

/* Pin Definitions: Steppers  */
#define STEP_PIN_AZ             15
#define DIR_PIN_AZ              3
#define STEP_PIN_ALT            36
#define DIR_PIN_ALT             37
/* Step Size Definitions */
#define FULL_STEP               {LOW, LOW, LOW}     /* MS1, MS2, MS3: drive all low  */
#define HALF_STEP               {HIGH, LOW, LOW}    /* MS1: drive high; MS2, MS3: drive low */
#define QUARTER_STEP            {LOW, HIGH, LOW}    /* MS1: drive low; MS2: drive high; MS3: drive low */
#define EIGHTH_STEP             {HIGH, HIGH, LOW}   /* MS1, MS2: drive high; MS3: drive low */
#define SIXTEENTH_STEP          {HIGH, HIGH, HIGH}  /* MS1, MS2, MS3: drive all high */
/* Pin Definitions: Driver */
#define MS1_PIN                 7
#define MS2_PIN                 5
#define MS3_PIN                 6
/* Serial Definitions */
#define SERIAL_BAUDE            115200
#define SERIAL_UART_RX          18
#define SERIAL_UART_TX          17
/* Clock Definitions */
#define TIMER_CLK_DIV           40
#define ATTACH_TIMER_ALARM      1000000
#define ATTACH_INT_ON_EDGE      true
#define TIMER_COUNT_UP          true
#define TIMER_INT_AUTO_RELOAD   true
#define MAX_TIMER_INTERVAL      250000


/******************************************/
/*           FUNCTION PROTOTYPES          */
/******************************************/

int calculatePID(double kp, double ki, double kd, double* integralXY, int const currentError, double* previousError);
void driveStepper(std::array<int, 3> const &stepSize);
void controlStepper(hw_timer_t* stepperTimer, int directionPin, int pidOutput);
void findMarkerX(void);


/******************************************/
/*             MACRO FUNCTIONS            */
/******************************************/

#define CALCULATE_PID_X(DIST_X)           calculatePID(Kp_az, Ki_az, Kd_az, &integralX, DIST_X, &previousErrorX)
#define CALCULATE_PID_Y(DIST_Y)           calculatePID(Kp_alt, Ki_alt, Kd_alt, &integralX, DIST_Y, &previousErrorY)
#define CONTROL_AZ_STEPPER(PID_OUTPUT)    controlStepper(azTimer, DIR_PIN_AZ, PID_OUTPUT)
#define CONTROL_ALT_STEPPER(PID_OUTPUT)   controlStepper(altTimer, DIR_PIN_ALT, PID_OUTPUT)


#endif