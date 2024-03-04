// Pin Definitions
#define stepPinAZ 15
#define dirPinAZ 3
#define stepPinALT 36
#define dirPinALT 37
//#define potPin A5
#define MS1pin 7
#define MS2pin 5
#define MS3pin 6


// PID variables (adjust these to fine-tune)
double Kp = 1.0;  // Proportional gain
double Ki = 0.01;  // Integral gain
double Kd = 0.1;  // Derivative gain
int len = 1670; //ms
int tick = 0;

int len1 = 2000; //ms
int tick1 = 0;

double previousErrorX = 0;
double integralX = 0;
double previousErrorY = 0;
double integralY = 0;

int setPoint = 0;

bool lastDirection = 0;
int scanCounter = 0;

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

  // Initialize as 16th step
  digitalWrite(MS1pin, HIGH);
  digitalWrite(MS2pin, HIGH);
  digitalWrite(MS3pin, HIGH);
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

// Function to control the stepper motor based on the step size
void controlStepperAZ(int stepSize) {
  // Determine the appropriate step mode based on step size
  fullStep();
  if (abs(stepSize) < 8) {
    tick = 0;
    len = 0;
  } else if (abs(stepSize) < 32) {
    tick = 1;
    len = 327;
  } else if (abs(stepSize) < 64) {
    tick = 2;
    len = 410;
  } else if (abs(stepSize) < 128) {
    tick = 4;
    len = 633;
  }
  // else if (abs(stepSize) < 1024) {
  //   tick = 16;
  //   len = 2000;
  // }
  else {
    tick = 8;
    len = 1200;
  }

  // Set the direction based on the sign of stepSize
  digitalWrite(dirPinAZ, (stepSize >= 0) ? HIGH : LOW);

  // store the last used direction
  lastDirection = stepSize >= 0 ? HIGH : LOW;

  // Perform the steps
  for (int i = 0; i < abs(tick); ++i) {
    digitalWrite(stepPinAZ, HIGH);
    delayMicroseconds(len);
    digitalWrite(stepPinAZ, LOW);
    delayMicroseconds(len);
  }


}

// Function to control the stepper motor based on the step size
void controlStepperALT(int stepSize) {
  // Determine the appropriate step mode based on step size
  fullStep();
  if (abs(stepSize) < 8) {
    tick1 = 0;
    len1 = 0;
  } else if (abs(stepSize) < 32) {
    tick1 = 1;
    len1 = 1350;
  } else if (abs(stepSize) < 64) {
    tick1 = 2;
    len1 = 1400;
  } else if (abs(stepSize) < 128) {
    tick1 = 4;
    len1 = 1600;
  }
  // else if (abs(stepSize) < 1024) {
  //   tick = 16;
  //   len = 2200;
  // }
  else {
    tick1 = 8;
    len1 = 1400;
  }

  // Set the direction based on the sign of stepSize
  digitalWrite(dirPinALT, (stepSize >= 0) ? HIGH : LOW);

  // store the last used direction
  lastDirection = stepSize >= 0 ? HIGH : LOW;

  // Perform the steps
  for (int i = 0; i < abs(tick1); ++i) {
    digitalWrite(stepPinALT, HIGH);
    delayMicroseconds(len1);
    digitalWrite(stepPinALT, LOW);
    delayMicroseconds(len1);
  }


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