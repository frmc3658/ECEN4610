// Testing Measurements at 10m distance x moved at each step
// 1/16: 8, 7, 1, 3, 3, 4 ==> 4.3 cm
// 1/8:  3, 6, 8, 11, 2, 5 ==> 5.83 cm
// 1/4:  16, 13, 9, 25, 17 ==> 16 cm
// 1/2:  35, 25, 40, 21    ==> 30.25 cm
// 1:    65, 56, 65, 61, 65.5 ==> 62.5 cm

// defines pins
#define stepPin 2
#define dirPin 5
#define potPin 6

#define MS1pin 13
#define MS2pin 12
#define MS3pin 11

int customDelay, customDelayMapped;
int loopy = 0;
int dist_x = 0;
int dist_y = 0;

// Black Green Orange Red

void setup() {
  Serial.begin(9600);
  
  // Sets the pins as Outputs
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
// delay(1000);
// Serial.print("3...\n");
// delay(1000);
// Serial.print("2...\n");
// delay(1000);
// Serial.print("1...\n");
// delay(1000);
// Serial.print("============================\n");

// Serial Input from Pi
  if (Serial.available() > 0) {
    String data = Serial.readStringUntil('\n');
    // Parse the received data
    dist_x = data.substring(0, data.indexOf(',')).toInt();
    dist_y = data.substring(data.indexOf(',') + 1).toInt();

    // Use dist_x and dist_y as needed
    Serial.print("Received: ");
    Serial.print("X: ");
    Serial.print(dist_x);
    Serial.print(", Y: ");
    Serial.println(dist_y); 
    
    int distance_x = dist_x * -1;
    if(distance_x > 0){
 
      digitalWrite(dirPin, LOW);
//    
      while(distance_x > 10){
        digitalWrite(stepPin, HIGH);
        delayMicroseconds(1500);
        digitalWrite(stepPin, LOW);
        delayMicroseconds(3000);

        String data = Serial.readStringUntil('\n');
        distance_x = data.substring(0, data.indexOf(',')).toInt() * -1;
      }
    }
    else{
    
      digitalWrite(dirPin, HIGH);
    
      while(distance_x < -10){
        digitalWrite(stepPin, HIGH);
        delayMicroseconds(1500);
        digitalWrite(stepPin, LOW);
        delayMicroseconds(3000);

        String data = Serial.readStringUntil('\n');
        distance_x = data.substring(0, data.indexOf(',')).toInt() * -1;
        
      }
    }

    digitalWrite(stepPin, LOW);

   }

}









// Custom function for reading the potentiometer and mapping its value from 300 to 3000, suitable for the custom delay value in microseconds
void speedControl() {
  customDelay = analogRead(A5); // Read the potentiometer value
  //Serial.print("Potentiometer: ");
  //Serial.print(customDelay);
  customDelayMapped = map(customDelay, 0, 1023, 300, 3000); // Convert the analog input from 0 to 1024, to 300 to 3000
  //Serial.print("; Mapped Values: ");
  //Serial.println(customDelayMapped);
}
