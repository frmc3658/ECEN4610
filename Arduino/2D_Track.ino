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
/*           FORWARD DECLARATIONS         */
/******************************************/

int calculatePID(double kp, double ki, double kd, double* integralXY, int const currentError, double* previousError);
void controlStepper(hw_timer_t* stepperTimer, int directionPin, int pidOutput);
/* MACRO functions for easier use */
#define CALCULATE_PID_X(DIST_X)           calculatePID(Kp_az, Ki_az, Kd_az, &integralX, DIST_X, &previousErrorX)
#define CALCULATE_PID_Y(DIST_Y)           calculatePID(Kp_alt, Ki_alt, Kd_alt, &integralX, DIST_Y, &previousErrorY)
#define CONTROL_AZ_STEPPER(PID_OUTPUT)    controlStepper(azTimer, DIR_PIN_AZ, PID_OUTPUT)
#define CONTROL_ALT_STEPPER(PID_OUTPUT)   controlStepper(altTimer, DIR_PIN_ALT, PID_OUTPUT)


/******************************************/
/*           GLOBAL VARIABLES             */
/******************************************/

// Stepper tick flags
volatile bool azTick = false;
volatile bool altTick = false;

// Timers for the alt/az stepper motors
hw_timer_t* azTimer = NULL;
hw_timer_t* altTimer = NULL;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;

// PID variables: ALT (adjust these to fine-tune)
double Kp_alt = 0.5;    // Proportional gain
double Ki_alt = 0.0005; // Integral gain
double Kd_alt = 0.005;  // Derivative gain

// PID variables: AZ (adjust these to fine-tune)
double Kp_az = 0.55;  // Proportional gain
double Ki_az = 0.005; // Integral gain
double Kd_az = 0.05;  // Derivative gain

// Error variables
double previousErrorX   = 0;
double integralX        = 0;
double previousErrorY   = 0;
double integralY        = 0;

int setPoint        = 0;
bool lastDirection  = 0;
int scanCounter     = 0;


/******************************************/
/*        INTERRUPT SERVICE ROUTINES      */
/******************************************/


/*
 * @brief Azimuth stepper timer ISR
 */
void IRAM_ATTR AzTimer()
{
  // Enter critical section
  portENTER_CRITICAL_ISR(&timerMux);

  // Drive signal to the azimuth stepper
  azTick = !azTick;
  digitalWrite(STEP_PIN_AZ, azTick ? HIGH : LOW);

  // Exit critical section
  portEXIT_CRITICAL_ISR(&timerMux);
}


/*
 * @brief Altitude stepper timer ISR
 */
void IRAM_ATTR AltTimer()
{
  // Enter critical section
  portENTER_CRITICAL_ISR(&timerMux);

  // Drive signal to the altitude stepper
  altTick = !altTick;
  digitalWrite(STEP_PIN_ALT, (altTick ? HIGH : LOW));

  // Exit critical section
  portEXIT_CRITICAL_ISR(&timerMux);
}


/******************************************/
/*        ARDUINO SETUP/MAIN LOOP         */
/******************************************/

/*
 * @brief Arduino Setup
 */
void setup()
{
  // Set serial port baud rates
  // NOTE: Serial port is the standard serial monitor for debugging.
  //       Serial1 port is the UART pin connected to the raspberry pi,
  //       which is used as input to drive the stepper motors in order
  //       to track the marker.
  Serial.begin(SERIAL_BAUDE);
  Serial1.begin(SERIAL_BAUDE, SERIAL_8N1, SERIAL_UART_RX, SERIAL_UART_TX);

  // Set alt/az stepper pins as Output
  pinMode(STEP_PIN_AZ, OUTPUT);
  pinMode(DIR_PIN_AZ, OUTPUT);
  pinMode(STEP_PIN_ALT, OUTPUT);
  pinMode(DIR_PIN_ALT, OUTPUT);

  // Set stepper driver pins as output
  pinMode(MS1_PIN, OUTPUT);
  pinMode(MS2_PIN, OUTPUT);
  pinMode(MS3_PIN, OUTPUT);

  // Setup the azimuth timer
  azTimer = timerBegin(0, TIMER_CLK_DIV, TIMER_COUNT_UP);
  timerAttachInterrupt(azTimer, &AzTimer, ATTACH_INT_ON_EDGE);
  timerAlarmWrite(azTimer, ATTACH_TIMER_ALARM, TIMER_INT_AUTO_RELOAD);
  timerAlarmEnable(azTimer);
  
  // Setup the altitude timer
  altTimer = timerBegin(1, TIMER_CLK_DIV, TIMER_COUNT_UP);
  timerAttachInterrupt(altTimer, &AltTimer, ATTACH_INT_ON_EDGE);
  timerAlarmWrite(altTimer, ATTACH_TIMER_ALARM, TIMER_INT_AUTO_RELOAD);
  timerAlarmEnable(altTimer);
}


/*
 * @brief Arduino main loop
 */
void loop()
{
  // Process serial UART input data from raspberry pi
  // NOTE: If there is data available, then that means OpenCV can
  //       find the marker
  if(Serial1.available() > 0)
  {
    // Pasrse distance coordinate data received from the raspberry pi
    // over the serial UART port
    String data = Serial1.readStringUntil('\n');
    int dist_x = data.substring(0, data.indexOf(',')).toInt();
    int dist_y = data.substring(data.indexOf(',') + 1).toInt();

    // Print data to the serial monitor
    Serial.print("Received: X: ");
    Serial.print(dist_x);
    Serial.print(", Y: ");
    Serial.println(dist_y);

    // Calculate PID output
    int pidOutputAZ = CALCULATE_PID_X(dist_x);
    int pidOutputALT = CALCULATE_PID_Y(dist_y);

    // Control stepper motor based on PID output
    CONTROL_AZ_STEPPER(pidOutputAZ);
    CONTROL_ALT_STEPPER(pidOutputALT);
  }
  // else /* Marker has been lost */
  // {
  //   findMarkerX();
  // }
}

/******************************************/
/*          FUNCTION DEFINITIONS          */
/******************************************/

/*
 * @brief Calculates PID error
 *
 * @param[in] kp  Proportional gain
 * @param[in] ki  Integral gain
 * @param[in] kd  Derivative gain
 * @param[in] integralXY  Previous x or y integral term
 * @param[in] currentError  The current error based on feedback loop
 * @param[in] previousError Previously calculated PID error
 * 
 * @return Step size based on output of PID loop
 * 
 * @note MACRO functions provided for ease of use
 */
int calculatePID(double kp, double ki, double kd, double* integralXY, 
                 int const currentError, double* previousError)
{
  // Calculate error and adjust integral term
  double error = setPoint - currentError;
  *integralXY += error;

  // Anti-windup: Limit the integral term
  if(*integralXY > 100){ *integralXY = 100; }
  else if(*integralXY < -100){ *integralXY = -100; }

  // Calculate derivative
  double derivative = error - *previousError;

  // PID formula
  double output = kp * error + ki * (*integralXY) + kd * derivative;

  // Save current error for the next iteration
  *previousError = error;

  // Convert the output to a step size
  return static_cast<int>(output);
}


/*
 * @brief Signals the
 *
 * @param[in] stepSize  Array of signals to output to the driver
 * 
 * @note  Macros are provided for stepSize input param
 *        for ease of use
 */
void driveStepper(std::array<int, 3> const &stepSize)
{
  digitalWrite(MS1_PIN, stepSize[0]);
  digitalWrite(MS2_PIN, stepSize[1]);
  digitalWrite(MS3_PIN, stepSize[2]);
}


/*
 * @brief Controls one of the stepper motors based on frequency
 *
 * @param[in] stepperTimer  The timer associated the stepped to control
 * @param[in] directionPin  Direction control pin of the stepper
 * @param[in] pidOutput     Output of the PID loop
 * 
 * @note  Maco functions are provided for use with the alt/az steppers
 *        for ease of use
 */
void controlStepper(hw_timer_t* stepperTimer, int directionPin, int pidOutput)
{
  driveStepper(SIXTEENTH_STEP);
  
  digitalWrite(directionPin, (pidOutput >= 0) ? HIGH : LOW);
  
  if(pidOutput < 4)
  {
    digitalWrite(directionPin, LOW);
  }

  // Calculate stepper frequency and interval
  float stepperFreq = (abs(pidOutput) / 100.0) * 500 * 4;  
  float stepperInterval = (stepperFreq > 0) ? 1000000 / stepperFreq : MAX_TIMER_INTERVAL;
  
  // Print stepper frequency to serial monitor
  Serial.print("Stepper Frequency: ");
  Serial.println(stepperFreq);

  // Change the stepper interrupt alarm
  timerAlarmWrite(stepperTimer, stepperInterval, TIMER_INT_AUTO_RELOAD);
}

/*
 * @brief Drives steppers to scan for the marker when not detected
 */
void findMarkerX()
{
  // Adjust scanning parameters based on scan counter
  int tick = 16;  // Default step size
  int len = 1200; // Default step duration

  if(scanCounter > 50 && scanCounter < 100)
  {
    // Reverse direction for smoother scanning
    digitalWrite(DIR_PIN_AZ, !lastDirection);
  }
  else if(scanCounter >= 100)
  {
    // Marker not found within reasonable scans, handle error or fallback
    // stop scanning and return to main loop
    scanCounter = 0; // Reset scan counter
    return;
  }

  // Pulse StepPin
  for(int i = 0; i < abs(tick); ++i)
  {
    digitalWrite(STEP_PIN_AZ, HIGH);
    delayMicroseconds(len);
    digitalWrite(STEP_PIN_AZ, LOW);
    delayMicroseconds(len);
  }

  // Update scanning parameters and counters
  lastDirection = !lastDirection; // Toggle direction for next scan
  scanCounter++;
}
