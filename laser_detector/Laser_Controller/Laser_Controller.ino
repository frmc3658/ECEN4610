#include <vector>

#define laser_control_pin 18

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  pinMode(laser_control_pin, OUTPUT);
}

int state = 1;
std::vector<int> arr = {1,0,1,0,1,0,1,0};
std::vector<int> detector_arr = {0,0,0,0,0,0,0,0};

void loop() {
  //int sensorValue = analogRead(A14);
  //Serial.println(sensorValue);
  // put your main code here, to run repeatedly:

//  digitalWrite(laser_control_pin, HIGH);
//  delayMicroseconds(2); // lowest time possible is 10uS
//  int sensorValue = analogRead(A14);
//  Serial.println(sensorValue);
//  digitalWrite(laser_control_pin, LOW);
//  delayMicroseconds(2);
//  sensorValue = analogRead(A14);
//  Serial.println(sensorValue);
//  digitalWrite(laser_control_pin, HIGH);
//  delay(1);
//  digitalWrite(laser_control_pin, LOW);
//  delay(1);


//////////////////////////////////////////////
if(state == 1)
{
for (int j = 0; j< 2; j++)
{
for (int i = 0; i < 8; i++)
  {
    if(arr[i] == 1)
      {
        digitalWrite(laser_control_pin, HIGH);
        delayMicroseconds(10);
      }
    else
      {
        digitalWrite(laser_control_pin, LOW);
        delayMicroseconds(10);
      }
    int sensorValue = analogRead(A14);
    if(sensorValue > 2000)
        {
          detector_arr[i] = 1;
        }
    else
        {
          detector_arr[i] = 0;
        }
  }
  for(int i = 0; i < 8; i++)
  {
    Serial.print(detector_arr[i]);
  }
  state = 2;
  //Serial.println("done");
  arr = {0,1,0,1,0,1,0,1};
}
}
  if(state == 2)
  {
    delay(3000);
    detector_arr = {0,0,0,0,0,0,0,0};
    arr = {1,0,1,0,1,0,1,0};
    state = 1;
  }
}
 /////////////////////////////////////////////
