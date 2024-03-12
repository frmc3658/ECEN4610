// Arduino code to control a stepper motor based on input from Raspberry Pi

// Pin Definitions
#define stepPin 2
#define dirPin 5
#define potPin A5
#define MS1pin 13
#define MS2pin 12
#define MS3pin 11

void setup() {
  Serial.begin(9600);

  // Set pins as Outputs
  pinMode(stepPin, OUTPUT);
  pinMode(dirPin, OUTPUT);
  pinMode(MS1pin, OUTPUT);
  pinMode(MS2pin, OUTPUT);
  pinMode(MS3pin, OUTPUT);

  // Set pins based on step size desired
  digitalWrite(MS1pin, LOW);
  digitalWrite(MS2pin, HIGH);
  digitalWrite(MS3pin, LOW);
}

void loop() {
  // Read input from Raspberry Pi
  if (Serial.available() > 0) {
    String data = Serial.readStringUntil('\n');
    int dist_x = data.substring(0, data.indexOf(',')).toInt();
    int dist_y = data.substring(data.indexOf(',') + 1).toInt();

    // Print received data
    Serial.print("Received: X: ");
    Serial.print(dist_x);
    Serial.print(", Y: ");
    Serial.println(dist_y);

    // Control stepper motor based on dist_x value
    controlStepper(dist_x);
  }
}

// Function to control the stepper motor based on the distance value
void controlStepper(int distance_x) {
  digitalWrite(dirPin, (distance_x > 0) ? LOW : HIGH);

  while (abs(distance_x) > 10) {
    digitalWrite(stepPin, HIGH);
    delayMicroseconds(1500);
    digitalWrite(stepPin, LOW);
    delayMicroseconds(3000);

    String data = Serial.readStringUntil('\n');
    distance_x = data.substring(0, data.indexOf(',')).toInt();
    distance_x *= -1;
  }

  digitalWrite(stepPin, LOW);
}
