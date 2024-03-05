#include "esp32-hal-timer.h"

volatile bool azTick = false;
volatile bool altTick = false;

hw_timer_t* azTimer = NULL;
hw_timer_t* altTimer = NULL;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;


// Pin Definitions
#define stepPinAZ 15
#define dirPinAZ 3
#define stepPinALT 36
#define dirPinALT 37
//#define potPin A5
#define MS1pin 7
#define MS2pin 5
#define MS3pin 6

static int MAX_INTERVAL = 250000;

// PID variables (adjust these to fine-tune)
double Kp = 1.0;  // Proportional gain
double Ki = 0.01;  // Integral gain
double Kd = 0.1;  // Derivative gain

int k = 15;

double previousErrorX = 0;
double integralX = 0;
double previousErrorY = 0;
double integralY = 0;

int setPoint = 0;

bool lastDirection = 0;
int scanCounter = 0;

void IRAM_ATTR AzTimer() {
  portENTER_CRITICAL_ISR(&timerMux);
  azTick = !azTick;
  digitalWrite(stepPinAZ, azTick ? HIGH : LOW);
  portEXIT_CRITICAL_ISR(&timerMux);
}

void IRAM_ATTR AltTimer() {
  portENTER_CRITICAL_ISR(&timerMux);
  altTick = !altTick;
  digitalWrite(stepPinALT, altTick ? HIGH : LOW);
  portEXIT_CRITICAL_ISR(&timerMux);
}

void setupAzTimer(float intervalAz){
    // Setup AZ timer
  azTimer = timerBegin(0, 80, true); 
  timerAttachInterrupt(azTimer, &AzTimer, true);
  timerAlarmWrite(azTimer, intervalAz, true);
  timerAlarmEnable(azTimer);
}

void setupAltTimer(float intervalAlt){
    // Setup ALT timer
  altTimer = timerBegin(1, 80, true);
  timerAttachInterrupt(altTimer, &AltTimer, true);
  timerAlarmWrite(altTimer, intervalAlt, true);
  timerAlarmEnable(altTimer);
}


void setup() {
  Serial.begin(115200);
  Serial1.begin(115200, SERIAL_8N1, 18, 17);
  // Set pins as Outputs
  pinMode(stepPinAZ, OUTPUT);
  pinMode(dirPinAZ, OUTPUT);
  pinMode(stepPinALT, OUTPUT);
  pinMode(dirPinALT, OUTPUT);

  pinMode(MS1pin, OUTPUT);
  pinMode(MS2pin, OUTPUT);
  pinMode(MS3pin, OUTPUT);

  setupAzTimer(10000000);
  setupAltTimer(10000000);

}

void loop() {
  if (Serial1.available() > 0) {
    String data = Serial1.readStringUntil('\n');
    int dist_x = data.substring(0, data.indexOf(',')).toInt();
    int dist_y = data.substring(data.indexOf(',') + 1).toInt();

    Serial.print("Received: X: ");
    Serial.print(dist_x);
    Serial.print(", Y: ");
    Serial.println(dist_y);

    // Calculate PID output
    int pidOutputAZ = calculatePIDX(dist_x);
    int pidOutputALT = calculatePIDY(dist_y);

    // Control stepper motor based on PID output
    controlStepperAZ(pidOutputAZ);
    controlStepperALT(pidOutputALT);
  }
  //  else{
  //    findMarkerX();
  //  }
}

// Function to calculate PID output
int calculatePIDX(int currentError) {
  double errorX = setPoint - currentError;
  integralX += errorX;

  // Anti-windup (optional): Limit the integral term
  if (integralX > 100) {
    integralX = 100;
  } else if (integralX < -100) {
    integralX = -100;
  }

  double derivativeX = errorX - previousErrorX;

  // PID formula
  double outputX = Kp * errorX + Ki * integralX + Kd * derivativeX;

  // Save current error for the next iteration
  previousErrorX = errorX;

  // Convert the output to a step size
  return static_cast<int>(outputX);
}

// Function to calculate PID output
int calculatePIDY(int currentError) {
  double errorY = setPoint - currentError;
  integralY += errorY;

  // Anti-windup (optional): Limit the integral term
  if (integralY > 100) {
    integralY = 100;
  } else if (integralY < -100) {
    integralY = -100;
  }

  double derivativeY = errorY - previousErrorY;

  // PID formula
  double outputY = Kp * errorY + Ki * integralY + Kd * derivativeY;

  // Save current error for the next iteration
  previousErrorY = errorY;

  // Convert the output to a step size
  return static_cast<int>(outputY);
}

float normalizePIDOutput(int pidOutput) {
    float normalizedOutput = pidOutput / 300.0f;
    return normalizedOutput;
}

// Function to control the stepper motor based on the frequency
void controlStepperAZ(int pidOutputAZ) {
    float outputAZ = normalizePIDOutput(pidOutputAZ);
    if(outputAZ == 0){
      setupAltTimer(5000000);
    }
    float k = 3.0;
    float azFreq = 600 * (1 - exp(-k * outputAZ)); // Calculate azimuth frequency
    
    float intervalAZ = (azFreq > 0) ? 1000000 / azFreq : MAX_INTERVAL;
    setupAzTimer(intervalAZ);
}

void controlStepperALT(int pidOutputALT) {
    float outputALT = normalizePIDOutput(pidOutputALT);

    if(outputALT == 0){
      setupAltTimer(5000000);
    }
    float k = 3.0;    
    float altFreq = 600 * (1 - exp(-k * outputALT));     
    
    float intervalALT = (altFreq > 0) ? 1000000 / altFreq : MAX_INTERVAL;
    setupAltTimer(intervalALT);
}


void findMarkerX() {
  // Adjust scanning parameters based on scan counter
  int tick = 16; // Default step size
  int len = 1200; // Default step duration

  if (scanCounter > 50 && scanCounter < 100) {
    // Reverse direction for smoother scanning
    digitalWrite(dirPinAZ, !lastDirection);
  } else if (scanCounter >= 100) {
    // Marker not found within reasonable scans, handle error or fallback
    // stop scanning and return to main loop
    scanCounter = 0; // Reset scan counter
    return;
  }

  // Pulse StepPin
  for (int i = 0; i < abs(tick); ++i) {
    digitalWrite(stepPinAZ, HIGH);
    delayMicroseconds(len);
    digitalWrite(stepPinAZ, LOW);
    delayMicroseconds(len);
  }

  // Update scanning parameters and counters
  lastDirection = !lastDirection; // Toggle direction for next scan
  scanCounter++;
}


// Custom Functions for Step Sizes
void fullStep() {
  digitalWrite(MS1pin, LOW);
  digitalWrite(MS2pin, LOW);
  digitalWrite(MS3pin, LOW);
}

void halfStep() {
  digitalWrite(MS1pin, HIGH);
  digitalWrite(MS2pin, LOW);
  digitalWrite(MS3pin, LOW);
}

void quarterStep() {
  digitalWrite(MS1pin, LOW);
  digitalWrite(MS2pin, HIGH);
  digitalWrite(MS3pin, LOW);
}

void eightStep() {
  digitalWrite(MS1pin, HIGH);
  digitalWrite(MS2pin, HIGH);
  digitalWrite(MS3pin, LOW);
}

void sixteenStep() {
  digitalWrite(MS1pin, HIGH);
  digitalWrite(MS2pin, HIGH);
  digitalWrite(MS3pin, HIGH);
}