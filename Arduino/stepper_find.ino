// Pin Definitions
#define stepPin 15
#define dirPin 3
//#define potPin A5
#define MS1pin 7
#define MS2pin 6
#define MS3pin 5


// PID variables (adjust these to fine-tune)
double Kp = 1.0;  // Proportional gain
double Ki = 0.01;  // Integral gain
double Kd = 0.1;  // Derivative gain
int len = 1670; //ms
int tick = 0;

double previousError = 0;
double integral = 0;

int setPoint = 0;

bool lastDirection = 0;
int scanCounter = 0;

void setup() {
  Serial.begin(115200);
  Serial1.begin(115200, SERIAL_8N1, 18, 17); //set RX and TX for Serial1
  // Set pins as Outputs
  pinMode(stepPin, OUTPUT);
  pinMode(dirPin, OUTPUT);
  pinMode(MS1pin, OUTPUT);
  pinMode(MS2pin, OUTPUT);
  pinMode(MS3pin, OUTPUT);

  // Initialize as 16th step
  digitalWrite(MS1pin, HIGH);
  digitalWrite(MS2pin, HIGH);
  digitalWrite(MS3pin, HIGH);
}

void loop() {
//  Serial.println("Butthole");
//  delay(100);
  if (Serial1.available() > 0) {
    String data = Serial1.readStringUntil('\n');
    int dist_x = data.substring(0, data.indexOf(',')).toInt();
    int dist_y = data.substring(data.indexOf(',') + 1).toInt();

    Serial.print("Received: X: ");
    Serial.print(dist_x);
    Serial.print(", Y: ");
    Serial.println(dist_y);

    // Calculate PID output
    int pidOutput = calculatePID(dist_x);

    // Control stepper motor based on PID output
    controlStepper(pidOutput);
  }
  //  else{
  //    findMarkerX();
  //  }
}

// Function to calculate PID output
int calculatePID(int currentError) {
  double error = setPoint - currentError;
  integral += error;

  // Anti-windup (optional): Limit the integral term
  if (integral > 100) {
    integral = 100;
  } else if (integral < -100) {
    integral = -100;
  }

  double derivative = error - previousError;

  // PID formula
  double output = Kp * error + Ki * integral + Kd * derivative;

  // Save current error for the next iteration
  previousError = error;

  // Convert the output to a step size
  return static_cast<int>(output);
}

// Function to control the stepper motor based on the step size
void controlStepper(int stepSize) {
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
  digitalWrite(dirPin, (stepSize >= 0) ? HIGH : LOW);

  // store the last used direction
  lastDirection = stepSize >= 0 ? HIGH : LOW;

  // Perform the steps
  for (int i = 0; i < abs(tick); ++i) {
    digitalWrite(stepPin, HIGH);
    delayMicroseconds(len);
    digitalWrite(stepPin, LOW);
    delayMicroseconds(len);
  }


}

void findMarkerX() {
  // Adjust scanning parameters based on scan counter
  int tick = 16; // Default step size
  int len = 1200; // Default step duration

  if (scanCounter > 50 && scanCounter < 100) {
    // Reverse direction for smoother scanning
    digitalWrite(dirPin, !lastDirection);
  } else if (scanCounter >= 100) {
    // Marker not found within reasonable scans, handle error or fallback
    // stop scanning and return to main loop
    scanCounter = 0; // Reset scan counter
    return;
  }

  // Pulse StepPin
  for (int i = 0; i < abs(tick); ++i) {
    digitalWrite(stepPin, HIGH);
    delayMicroseconds(len);
    digitalWrite(stepPin, LOW);
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