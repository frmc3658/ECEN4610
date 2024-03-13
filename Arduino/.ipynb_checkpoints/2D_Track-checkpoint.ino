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

// PID variables: ALT (adjust these to fine-tune)
double Kp_alt = 0.5;  // Proportional gain
double Ki_alt = 0.0005;  // Integral gain
double Kd_alt = 0.005;  // Derivative gain

// PID variables: AZ (adjust these to fine-tune)
double Kp_az = 0.55;  // Proportional gain
double Ki_az = 0.005;  // Integral gain
double Kd_az = 0.05;  // Derivative gain

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
  //Serial.println("Az Tick!");
  azTick = !azTick;
  digitalWrite(stepPinAZ, azTick ? HIGH : LOW);
  portEXIT_CRITICAL_ISR(&timerMux);
}

void IRAM_ATTR AltTimer() {
  portENTER_CRITICAL_ISR(&timerMux);
  //Serial.println("Alt Tick!");
  altTick = !altTick;
  digitalWrite(stepPinALT, altTick ? HIGH : LOW);
  portEXIT_CRITICAL_ISR(&timerMux);
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

  
  // setupAzTimer(1000000);
  azTimer = timerBegin(0, 40, true); 
  //Serial.println("HereAZ1");
  timerAttachInterrupt(azTimer, &AzTimer, true);
  //Serial.println("HereAZ"); 
  timerAlarmWrite(azTimer, 1000000, true);
  timerAlarmEnable(azTimer);
  
  // setupAltTimer(1000000);
  altTimer = timerBegin(1, 40, true);
  timerAttachInterrupt(altTimer, &AltTimer, true);
  timerAlarmWrite(altTimer, 1000000/*intervalAlt*/, true);
  //Serial.println("HereALT"); 
  timerAlarmEnable(altTimer);
  
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
  double outputX = Kp_az * errorX + Ki_az * integralX + Kd_az * derivativeX;

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
  double outputY = Kp_alt * errorY + Ki_alt * integralY + Kd_alt * derivativeY;

  // Save current error for the next iteration
  previousErrorY = errorY;

  // Convert the output to a step size
  return static_cast<int>(outputY);
}

// Function to control the stepper motor based on the frequency
void controlStepperAZ(int pidOutputAZ) {
  sixteenStep();
  
  digitalWrite(dirPinAZ, (pidOutputAZ >= 0) ? HIGH : LOW);
  
  if(pidOutputAZ < 4){
    digitalWrite(stepPinAZ, LOW);
  }
  float k = 0.5;
  //float azFreq = 600 * (1 - exp(-k * abs(pidOutputAZ))); // Calculate azimuth frequency
  float azFreq = ((abs(pidOutputAZ) / 100.0) * 500)*4;  
  float intervalAZ = (azFreq > 0) ? 1000000 / azFreq : MAX_INTERVAL;
  Serial.print("Az Frequency: ");
  Serial.println(azFreq);

  timerAlarmWrite(azTimer, intervalAZ, true);
}

void controlStepperALT(int pidOutputALT) {
  
  digitalWrite(dirPinALT, (pidOutputALT >= 0) ? HIGH : LOW);

  if(pidOutputALT < 4){
    // setupAltTimer(5000000);
    digitalWrite(stepPinALT, LOW);
   }
  float k = 0.5;    
  //float altFreq = 600 * (1 - exp(-k * abs(pidOutputALT)));     
  float altFreq = (abs(pidOutputALT) / 100.0) * 500 * 4; // Linearly maps 0-100 to 0-400Hz

  float intervalALT = (altFreq > 0) ? 1000000 / altFreq : MAX_INTERVAL;
  Serial.print("Alt Freq: ");
  Serial.println(altFreq);
  timerAlarmWrite(altTimer, intervalALT, true);
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