// Pin Definitions
#define stepPin 2
#define dirPin 5
#define potPin A5
#define MS1pin 13
#define MS2pin 12
#define MS3pin 11


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
int cycles = 0;

void setup() {
  Serial.begin(9600);

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
  if (Serial.available() > 0) {
    String data = Serial.readStringUntil('\n');
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
  else{
    findMarkerX();
  }
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
    tick = 2;
    len = 327;
  } else if (abs(stepSize) < 64) {
    tick = 4;
    len = 410;
  } else if (abs(stepSize) < 128) {
    tick = 8;
    len = 633;
  }
  // else if (abs(stepSize) < 1024) {
  //   tick = 16;
  //   len = 2000;
  // } 
  else {
    tick = 16;
    len = 1200;
  }

  // Set the direction based on the sign of stepSize
  digitalWrite(dirPin, (stepSize >= 0) ? LOW : HIGH);

  // store the last used direction
  lastDirection = stepSize >= 0 ? LOW : HIGH;

  // Perform the steps
  for (int i = 0; i < abs(tick); ++i) {
    digitalWrite(stepPin, HIGH);
    delayMicroseconds(len);
    digitalWrite(stepPin, LOW);
    delayMicroseconds(len);
  }
}

void findMarkerX(){

  if(cycles > 20){
    cycles = 0;
    digitalWrite(dirPin, lastDirection);
  }
  else if(cycles > 10 && cycles < 20){ // adjust this after testing   
    // Use opposite direction from last seen frame
    digitalWrite(dirPin, !lastDirection);
  }
  else{
    // Use last direction from last seen frame
    digitalWrite(dirPin, lastDirection);
  }
  
  // Set ticks and length
  tick = 4;
  len = 410;

  // Pulse StepPin 
  for (int i = 0; i < abs(tick); ++i) {
    digitalWrite(stepPin, HIGH);
    delayMicroseconds(len);
    digitalWrite(stepPin, LOW);
    delayMicroseconds(len);
  }

  cycles++;
  
}

// Custom Functions for Step Sizes
void fullStep(){
  digitalWrite(MS1pin, LOW);
  digitalWrite(MS2pin, LOW);
  digitalWrite(MS3pin, LOW);
}

void halfStep(){
  digitalWrite(MS1pin, HIGH);
  digitalWrite(MS2pin, LOW);
  digitalWrite(MS3pin, LOW);
}

void quarterStep(){
  digitalWrite(MS1pin, LOW);
  digitalWrite(MS2pin, HIGH);
  digitalWrite(MS3pin, LOW);
}

void eightStep(){
  digitalWrite(MS1pin, HIGH);
  digitalWrite(MS2pin, HIGH);
  digitalWrite(MS3pin, LOW);
}

void sixteenStep(){
  digitalWrite(MS1pin, HIGH);
  digitalWrite(MS2pin, HIGH);
  digitalWrite(MS3pin, HIGH);
}