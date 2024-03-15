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
double Kd_alt = 0.1;  // Derivative gain

// PID variables: AZ (adjust these to fine-tune)
double Kp_az = 0.55;  // Proportional gain
double Ki_az = 0.005;  // Integral gain
double Kd_az = 0.25;  // Derivative gain

int k = 15;

double previousErrorX = 0;
double integralX = 0;
double previousErrorY = 0;
double integralY = 0;

double squaredIntegralX = 0; 
double squaredIntegralY = 0; 

int setPoint = 0;

bool lastDirectionX = 0;
bool lastDirectionY = 0;

int pidOutputAZ = 0;
int pidOutputALT = 0;

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

    if(data == "SCAN"){
      //Serial.println("Scanning");
      findMarkerX();
      if(abs(pidOutputALT) > 80){
        findMarkerY();
      }
      else{
        timerAlarmWrite(altTimer, MAX_INTERVAL, true); // talk with Aidan on this
      }
      
    }
    else{
      int dist_x = data.substring(0, data.indexOf(',')).toInt();
      int dist_y = data.substring(data.indexOf(',') + 1).toInt();
  
      //Serial.print("Received: X: ");
      Serial.println(dist_x);
      //Serial.print(", Y: ");
      //Serial.println(dist_y);
  
      // Calculate PID output
      pidOutputAZ = calculatePIDX(dist_x);
      pidOutputALT = calculatePIDY(dist_y);
  
      // Control stepper motor based on PID output
      controlStepperAZ(pidOutputAZ);
      controlStepperALT(pidOutputALT);
      
    } 
  }
}

// Function to calculate PID output
int calculatePIDX(int currentError) {
  double errorX = setPoint - currentError;
  integralX += errorX;
  squaredIntegralX += errorX * errorX + 100;

  // Anti-windup (optional): Limit the integral term
  if (integralX > 100) {
    integralX = 100;
  } else if (integralX < -100) {
    integralX = -100;
  }

  if (squaredIntegralX > 1000) { 
    squaredIntegralX = 0;
  } else if (squaredIntegralX < -1000) {
    squaredIntegralX = 0;
  }

  double derivativeX = errorX - previousErrorX;

  // PID formula
  //double outputX = Kp_az * errorX + Ki_az * integralX + Kd_az * derivativeX;
  double outputX = Kp_az * errorX + Ki_az * integralX + Kd_az * derivativeX + Ki_az * squaredIntegralX;

  // Save current error for the next iteration
  previousErrorX = errorX;

  // Convert the output to a step size
  return static_cast<int>(outputX);
}

// Function to calculate PID output
int calculatePIDY(int currentError) {
  double errorY = setPoint - currentError;
  integralY += errorY;
  squaredIntegralX += errorY * errorY + 80;

  // Anti-windup (optional): Limit the integral term
  if (integralY > 100) {
    integralY = 100;
  } else if (integralY < -100) {
    integralY = -100;
  }
  if (squaredIntegralY > 1000) { 
    squaredIntegralY = 0;
  } else if (squaredIntegralY < -1000) {
    squaredIntegralY = 0;
  }

  double derivativeY = errorY - previousErrorY;

  // PID formula
  double outputY = Kp_alt * errorY + Ki_alt * integralY + Kd_alt * derivativeY + Ki_alt * squaredIntegralY;

  // Save current error for the next iteration
  previousErrorY = errorY;

  // Convert the output to a step size
  return static_cast<int>(outputY);
}

// Function to control the stepper motor based on the frequency
void controlStepperAZ(int pidOutputAZ) {
  sixteenStep();
  
  digitalWrite(dirPinAZ, (pidOutputAZ >= 0) ? HIGH : LOW);
  lastDirectionX = (pidOutputAZ >= 0) ? HIGH : LOW; 
  
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
  lastDirectionY = (pidOutputALT >= 0) ? HIGH : LOW;

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
    digitalWrite(dirPinAZ, lastDirectionX);
    int scanFreq = 800;
    float intervalAZ = (scanFreq > 0) ? 1000000 / scanFreq : MAX_INTERVAL;
    timerAlarmWrite(azTimer, intervalAZ, true);
    lastDirectionX = !lastDirectionX;
}

void findMarkerY() {
    digitalWrite(dirPinALT, lastDirectionY);
    int scanFreq = 600; 
    float intervalALT = (scanFreq > 0) ? 1000000 / scanFreq : MAX_INTERVAL;
    timerAlarmWrite(altTimer, intervalALT, true);
    lastDirectionY = !lastDirectionY;
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