#include <iostream>
using namespace std;

#define NOTE_CS7 2093
#define laser_control_pin 18
#define photodetector_pin A14
#define buzzer_pin 12
#include <vector>

void setup() {
  Serial.begin(9600);
  pinMode(laser_control_pin, OUTPUT);
}

std::vector<int> detector_binary_array = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};

void loop() {
  int note = NOTE_CS7;
  char binary_note_array[16];


  //Convert note integer to a 16bit binary string
  String note_binary_string = decToBinary(note);
  //Convert string to character array
  note_binary_string.toCharArray(binary_note_array, 16);

  for(int k = 0; k <= 15; k++){
    Serial.print(binary_note_array[k]);
  }
  Serial.println("shit1");
  
  for(int i = 0; i <= 15; i++)
  {
    if(binary_note_array[i] == '1')
    {
      digitalWrite(laser_control_pin, HIGH);
      delayMicroseconds(100);
    }
    else
    {
      digitalWrite(laser_control_pin, LOW);
      delayMicroseconds(100);
    }
    int sensorValue = analogRead(A14);
    if(sensorValue > 2000)
        {
          detector_binary_array[i] = 1;
        }
    else
        {
          detector_binary_array[i] = 0;
        }
  }
  for(int j = 0; j <= 15; j++){
    Serial.print(detector_binary_array[j]);
  }
  Serial.println("shit");

  
}

String decToBinary(int n) 
{ 
    String binaryString = "";

  for (int i = 0; i <= 15; i++) {
    // Use bitwise AND to check the value of the i-th bit
    if (n & (1 << i)) {
      binaryString += '1';
    } else {
      binaryString += '0';
    }

//    // Add space for better readability (optional)
//    if (i % 4 == 0) {
//      binaryString += ' ';
//    }
  }

  return binaryString;
}
