#include "esp32-hal-timer.h"

#define PRINTING 0

volatile bool azTick = false;
volatile bool altTick = false;

hw_timer_t* azTimer = NULL;
hw_timer_t* altTimer = NULL;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;


// Pin Definitions
#define lasPin 1

#define stepPinAZ 5
#define dirPinAZ 4

#define stepPinALT 39
#define dirPinALT 38

#define MS1pinAZ 15
#define MS2pinAZ 7
#define MS3pinAZ 6

#define MS1pinALT 42
#define MS2pinALT 41
#define MS3pinALT 40

static int MAX_INTERVAL = 250000;

// PID variables: ALT (adjust these to fine-tune)
double Kp_alt = 1.54;  // Proportional gain 0.69 1.45
double Ki_alt = 0.095;  // Integral gain 0.065 0.095
double Kd_alt = 0.195;  // Derivative gain 0.07 0.195 //.25

// PID variables: AZ (adjust these to fine-tune)

double Kp_az = 1.65;/*0.52;  // Proportional gain  1.6 */
double Ki_az = 0.068;/*0.0475;  // Integral gain 0.087*/
double Kd_az = 0.156; //0.055;/*0.052;  // Derivative gain 0.1 */ 

double a1 = 25.58; /*derivative calc constant 25.58*/
double a0 = -25.58; /*derivative calc constant -25.58*/
double b1 = -0.162; /*derivative calc constant -0.162*/
double b0 = 0.006561; /*derivative calc constant 0.006561*/


int k = 15;

double errorX = 0;
double derivativeX = 0;
double previousErrorX = 0;
double previousDerivativeX = 0;
double integralX = 0;
double derivativeY = 0;
double previousErrorY = 0;
double previousDerivativeY = 0;
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
  pinMode(lasPin, OUTPUT);
  pinMode(stepPinAZ, OUTPUT);
  pinMode(dirPinAZ, OUTPUT);
  pinMode(stepPinALT, OUTPUT);
  pinMode(dirPinALT, OUTPUT);

  pinMode(MS1pinAZ, OUTPUT);
  pinMode(MS2pinAZ, OUTPUT);
  pinMode(MS3pinAZ, OUTPUT);

  pinMode(MS1pinALT, OUTPUT);
  pinMode(MS2pinALT, OUTPUT);
  pinMode(MS3pinALT, OUTPUT);


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

    if (data == "SCAN") {
      //Serial.println("Scanning");
      digitalWrite(lasPin, LOW);
      findMarkerX();
      if (abs(pidOutputALT) > 80) {
        findMarkerY();
      }
      else {
        timerAlarmWrite(altTimer, MAX_INTERVAL, true); // talk with Aidan on this
      }

    }
    else {
      int dist_x = data.substring(0, data.indexOf(',')).toInt();
      int dist_y = data.substring(data.indexOf(',') + 1).toInt();

      //      Serial.print("Received: X: ");
      //      Serial.println(dist_x);
      //      Serial.print(", Y: ");
      //      Serial.println(dist_y);

      // Calculate PID output
      if(abs(dist_y) < 3){dist_y = 0;}
      pidOutputAZ = calculatePIDX(dist_x);
      pidOutputALT = calculatePIDY(dist_y);
#if PRINTING
      Serial.print(pidOutputAZ);
      Serial.print("\t");
      Serial.print(pidOutputALT);
      Serial.print("\t");
#endif
      

      if (abs(pidOutputAZ) > 2000 || abs(pidOutputALT) > 1500) {
        digitalWrite(lasPin, LOW);
      }
      else {
        digitalWrite(lasPin, HIGH);
      }

      // Control stepper motor based on PID output
      controlStepperAZ(pidOutputAZ);
      controlStepperALT(pidOutputALT);

    }

#if !PRINTING
    if (Serial.available() > 0) {
      String command = Serial.readStringUntil(' ');
      double value = Serial.parseFloat();
  
      if (command == "Kp_az") {
        Kp_az = value;
      } else if (command == "Ki_az") {
        Ki_az = value;
      } else if (command == "Kd_az") {
        Kd_az = value;
      } else if (command == "Kp_alt") {
        Kp_alt = value;
      } else if (command == "Ki_alt") {
        Ki_alt = value;
      } else if (command == "Kd_alt") {
        Kd_alt = value;
      }
    }
#endif
  }
}

// Function to calculate PID output
int calculatePIDX(int currentError) {
  errorX = setPoint - currentError;
  integralX += errorX;
  squaredIntegralX += errorX * errorX + 100;

  // Anti-windup (optional): Limit the integral term
  if (integralX > 500) {
    integralX = 500;
  } else if (integralX < -500) {
    integralX = -500;
  }

  if (squaredIntegralX > 500) {
    squaredIntegralX = 500;
  } else if (squaredIntegralX < -500) {
    squaredIntegralX = -500;
  }

  derivativeX = (a1 * errorX) + (a0 * previousErrorX) - (b1 * derivativeX) - (b0 * previousDerivativeX); /*y(k) = a1*u1+a0*u2-b1*y1-b0*y2;*/
  // PID formula
  //double outputX = Kp_az * errorX + Ki_az * integralX + Kd_az * derivativeX;
  double outputX = Kp_az * errorX + Ki_az * integralX + Kd_az * derivativeX /*+ Ki_az * squaredIntegralX*/;

  // Save current error and derivative for the next iteration
  previousErrorX = errorX;
  previousDerivativeX = derivativeX;

  // Convert the output to a step size
  return static_cast<int>(outputX);
}

// Function to calculate PID output
int calculatePIDY(int currentError) {
  double errorY = setPoint - currentError;
  integralY += errorY;
  squaredIntegralX += errorY * errorY + 80;

  // Anti-windup (optional): Limit the integral term
  if (integralY > 600) {
    integralY = 500;
  } else if (integralY < -600) {
    integralY = -500;
  }
  if (squaredIntegralY > 1000) {
    squaredIntegralY = 0;
  } else if (squaredIntegralY < -1000) {
    squaredIntegralY = 0;
  }

  derivativeY = (a1 * errorY) + (a0 * previousErrorY) - (b1 * derivativeY) - (b0 * previousDerivativeY);

  // PID formula
  double outputY = Kp_alt * errorY + Ki_alt * integralY + Kd_alt * derivativeY /*+ Ki_alt * squaredIntegralY*/;

  // Save current error for the next iteration
  previousErrorY = errorY;
  previousDerivativeY = derivativeY;

  // Convert the output to a step size
  return static_cast<int>(outputY);
}

// Function to control the stepper motor based on the frequency
void controlStepperAZ(int pidOutputAZ) {
  sixteenStepAZ();

  digitalWrite(dirPinAZ, (pidOutputAZ >= 0) ? HIGH : LOW);
  lastDirectionX = (pidOutputAZ >= 0) ? HIGH : LOW;

  //  if(abs(pidOutputAZ) > 400){
  //    digitalWrite(stepPinAZ, LOW);
  //  }
  float azFreq = ((abs(pidOutputAZ) / 100.0) * 3350);
  float intervalAZ = (azFreq > 0) ? 1000000 / azFreq : MAX_INTERVAL;
  //Serial.print("Az Frequency: ");
#if PRINTING
  Serial.print(azFreq);
  Serial.print("\t");
#endif
  timerAlarmWrite(azTimer, intervalAZ, true);
}

void controlStepperALT(int pidOutputALT) {
  sixteenStepALT();

  digitalWrite(dirPinALT, (pidOutputALT >= 0) ? HIGH : LOW);
  lastDirectionY = (pidOutputALT >= 0) ? HIGH : LOW;

  //  if(abs(pidOutputALT) > 400){
  //    digitalWrite(stepPinALT, LOW);
  //   }
  float altFreq = (abs(pidOutputALT) / 100.0) * 3350; // Linearly maps 0-100 to 0-400Hz
  // if(abs(pidOutputALT) < 7){altFreq = 0;}
  float intervalALT = (altFreq > 0) ? 1000000 / altFreq : MAX_INTERVAL;
  //Serial.print("Alt Freq: ");
#if PRINTING
  Serial.println(altFreq);
#endif
  timerAlarmWrite(altTimer, intervalALT, true);
}


void findMarkerX() {
  digitalWrite(dirPinAZ, lastDirectionX);
  int scanFreq = 1;
  float intervalAZ = (scanFreq > 0) ? 1000000 / scanFreq : 100;
  timerAlarmWrite(azTimer, intervalAZ, true);
  lastDirectionX = !lastDirectionX;
}

void findMarkerY() {
  digitalWrite(dirPinALT, lastDirectionY);
  int scanFreq = 1;
  float intervalALT = (scanFreq > 0) ? 1000000 / scanFreq : 100;
  timerAlarmWrite(altTimer, intervalALT, true);
  lastDirectionY = !lastDirectionY;
}


// Custom Functions for Step Sizes
void fullStepAZ() {
  digitalWrite(MS1pinAZ, LOW);
  digitalWrite(MS2pinAZ, LOW);
  digitalWrite(MS3pinAZ, LOW);
}

void halfStepAZ() {
  digitalWrite(MS1pinAZ, HIGH);
  digitalWrite(MS2pinAZ, LOW);
  digitalWrite(MS3pinAZ, LOW);
}

void quarterStepAZ() {
  digitalWrite(MS1pinAZ, LOW);
  digitalWrite(MS2pinAZ, HIGH);
  digitalWrite(MS3pinAZ, LOW);
}

void eightStepAZ() {
  digitalWrite(MS1pinAZ, HIGH);
  digitalWrite(MS2pinAZ, HIGH);
  digitalWrite(MS3pinAZ, LOW);
}

void sixteenStepAZ() {
  digitalWrite(MS1pinAZ, HIGH);
  digitalWrite(MS2pinAZ, HIGH);
  digitalWrite(MS3pinAZ, HIGH);
}

void fullStepALT() {
  digitalWrite(MS1pinALT, LOW);
  digitalWrite(MS2pinALT, LOW);
  digitalWrite(MS3pinALT, LOW);
}

void halfStepALT() {
  digitalWrite(MS1pinALT, HIGH);
  digitalWrite(MS2pinALT, LOW);
  digitalWrite(MS3pinALT, LOW);
}

void quarterStepALT() {
  digitalWrite(MS1pinALT, LOW);
  digitalWrite(MS2pinALT, HIGH);
  digitalWrite(MS3pinALT, LOW);
}

void eightStepALT() {
  digitalWrite(MS1pinALT, HIGH);
  digitalWrite(MS2pinALT, HIGH);
  digitalWrite(MS3pinALT, LOW);
}

void sixteenStepALT() {
  digitalWrite(MS1pinALT, HIGH);
  digitalWrite(MS2pinALT, HIGH);
  digitalWrite(MS3pinALT, HIGH);
}
