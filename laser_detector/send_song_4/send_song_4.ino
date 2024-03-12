#include <iostream>


#define NOTE_B0 31
#define NOTE_C1 33
#define NOTE_CS1 35
#define NOTE_D1 37
#define NOTE_DS1 39
#define NOTE_E1 41
#define NOTE_F1 44
#define NOTE_FS1 46
#define NOTE_G1 49
#define NOTE_GS1 52
#define NOTE_A1 55
#define NOTE_AS1 58
#define NOTE_B1 62
#define NOTE_C2 65
#define NOTE_CS2 69
#define NOTE_D2 73
#define NOTE_DS2 78
#define NOTE_E2 82
#define NOTE_F2 87
#define NOTE_FS2 93
#define NOTE_G2 98
#define NOTE_GS2 104
#define NOTE_A2 110
#define NOTE_AS2 117
#define NOTE_B2 123
#define NOTE_C3 131
#define NOTE_CS3 139
#define NOTE_D3 147
#define NOTE_DS3 156
#define NOTE_E3 165
#define NOTE_F3 175
#define NOTE_FS3 185
#define NOTE_G3 196
#define NOTE_GS3 208
#define NOTE_A3 220
#define NOTE_AS3 233
#define NOTE_B3 247
#define NOTE_C4 262
#define NOTE_CS4 277
#define NOTE_D4 294
#define NOTE_DS4 311
#define NOTE_E4 330
#define NOTE_F4 349
#define NOTE_FS4 370
#define NOTE_G4 392
#define NOTE_GS4 415
#define NOTE_A4 440
#define NOTE_AS4 466
#define NOTE_B4 494
#define NOTE_C5 523
#define NOTE_CS5 554
#define NOTE_D5 587
#define NOTE_DS5 622
#define NOTE_E5 659
#define NOTE_F5 698
#define NOTE_FS5 740
#define NOTE_G5 784
#define NOTE_GS5 831
#define NOTE_A5 880
#define NOTE_AS5 932
#define NOTE_B5 988
#define NOTE_C6 1047
#define NOTE_CS6 1109
#define NOTE_D6 1175
#define NOTE_DS6 1245
#define NOTE_E6 1319
#define NOTE_F6 1397
#define NOTE_FS6 1480
#define NOTE_G6 1568
#define NOTE_GS6 1661
#define NOTE_A6 1760
#define NOTE_AS6 1865
#define NOTE_B6 1976
#define NOTE_C7 2093
#define NOTE_CS7 2217
#define NOTE_D7 2349
#define NOTE_DS7 2489
#define NOTE_E7 2637
#define NOTE_F7 2794
#define NOTE_FS7 2960
#define NOTE_G7 3136
#define NOTE_GS7 3322
#define NOTE_A7 3520
#define NOTE_AS7 3729
#define NOTE_B7 3951
#define NOTE_C8 4186
#define NOTE_CS8 4435
#define NOTE_D8 4699
#define NOTE_DS8 4978
#define REST 0

//int melody[] = {
//  NOTE_E5, NOTE_E5, REST, NOTE_E5, REST, NOTE_C5, NOTE_E5,
//  NOTE_G5, REST, NOTE_G4, REST, 
//  NOTE_C5, NOTE_G4, REST, NOTE_E4,
//  NOTE_A4, NOTE_B4, NOTE_AS4, NOTE_A4,
//  NOTE_G4, NOTE_E5, NOTE_G5, NOTE_A5, NOTE_F5, NOTE_G5,
//  REST, NOTE_E5,NOTE_C5, NOTE_D5, NOTE_B4,
//  NOTE_C5, NOTE_G4, REST, NOTE_E4,
//  NOTE_A4, NOTE_B4, NOTE_AS4, NOTE_A4,
//  NOTE_G4, NOTE_E5, NOTE_G5, NOTE_A5, NOTE_F5, NOTE_G5,
//  REST, NOTE_E5,NOTE_C5, NOTE_D5, NOTE_B4,
//  
//  REST, NOTE_G5, NOTE_FS5, NOTE_F5, NOTE_DS5, NOTE_E5,
//  REST, NOTE_GS4, NOTE_A4, NOTE_C4, REST, NOTE_A4, NOTE_C5, NOTE_D5,
//  REST, NOTE_DS5, REST, NOTE_D5,
//  NOTE_C5, REST,
//  
//  REST, NOTE_G5, NOTE_FS5, NOTE_F5, NOTE_DS5, NOTE_E5,
//  REST, NOTE_GS4, NOTE_A4, NOTE_C4, REST, NOTE_A4, NOTE_C5, NOTE_D5,
//  REST, NOTE_DS5, REST, NOTE_D5,
//  NOTE_C5, REST,
//  
//  NOTE_C5, NOTE_C5, NOTE_C5, REST, NOTE_C5, NOTE_D5,
//  NOTE_E5, NOTE_C5, NOTE_A4, NOTE_G4,
//  
//  NOTE_C5, NOTE_C5, NOTE_C5, REST, NOTE_C5, NOTE_D5, NOTE_E5,
//  REST, 
//  NOTE_C5, NOTE_C5, NOTE_C5, REST, NOTE_C5, NOTE_D5,
//  NOTE_E5, NOTE_C5, NOTE_A4, NOTE_G4,
//  NOTE_E5, NOTE_E5, REST, NOTE_E5, REST, NOTE_C5, NOTE_E5,
//  NOTE_G5, REST, NOTE_G4, REST, 
//  NOTE_C5, NOTE_G4, REST, NOTE_E4,
//  
//  NOTE_A4, NOTE_B4, NOTE_AS4, NOTE_A4,
//  NOTE_G4, NOTE_E5, NOTE_G5, NOTE_A5, NOTE_F5, NOTE_G5,
//  REST, NOTE_E5, NOTE_C5, NOTE_D5, NOTE_B4,
//  
//  NOTE_C5, NOTE_G4, REST, NOTE_E4,
//  NOTE_A4, NOTE_B4, NOTE_AS4, NOTE_A4,
//  NOTE_G4, NOTE_E5, NOTE_G5, NOTE_A5, NOTE_F5, NOTE_G5,
//  REST, NOTE_E5, NOTE_C5, NOTE_D5, NOTE_B4,
//  
//  NOTE_E5, NOTE_C5, NOTE_G4, REST, NOTE_GS4,
//  NOTE_A4, NOTE_F5, NOTE_F5, NOTE_A4,
//  NOTE_D5, NOTE_A5, NOTE_A5, NOTE_A5, NOTE_G5, NOTE_F5,
//  
//  NOTE_E5, NOTE_C5, NOTE_A4, NOTE_G4,
//  NOTE_E5, NOTE_C5, NOTE_G4, REST, NOTE_GS4,
//  NOTE_A4, NOTE_F5, NOTE_F5, NOTE_A4,
//  NOTE_B4, NOTE_F5, NOTE_F5, NOTE_F5, NOTE_E5, NOTE_D5,
//  NOTE_C5, NOTE_E4, NOTE_E4, NOTE_C4,
//  
//  NOTE_E5, NOTE_C5, NOTE_G4, REST, NOTE_GS4,
//  NOTE_A4, NOTE_F5, NOTE_F5, NOTE_A4,
//  NOTE_D5, NOTE_A5, NOTE_A5, NOTE_A5, NOTE_G5, NOTE_F5,
//  
//  NOTE_E5, NOTE_C5, NOTE_A4, NOTE_G4,
//  NOTE_E5, NOTE_C5, NOTE_G4, REST, NOTE_GS4,
//  NOTE_A4, NOTE_F5, NOTE_F5, NOTE_A4,
//  NOTE_B4, NOTE_F5, NOTE_F5, NOTE_F5, NOTE_E5, NOTE_D5,
//  NOTE_C5, NOTE_E4, NOTE_E4, NOTE_C4,
//  NOTE_C5, NOTE_C5, NOTE_C5, REST, NOTE_C5, NOTE_D5, NOTE_E5,
//  REST,
//  
//  NOTE_C5, NOTE_C5, NOTE_C5, REST, NOTE_C5, NOTE_D5,
//  NOTE_E5, NOTE_C5, NOTE_A4, NOTE_G4,
//  NOTE_E5, NOTE_E5, REST, NOTE_E5, REST, NOTE_C5, NOTE_E5,
//  NOTE_G5, REST, NOTE_G4, REST, 
//  NOTE_E5, NOTE_C5, NOTE_G4, REST, NOTE_GS4,
//  NOTE_A4, NOTE_F5, NOTE_F5, NOTE_A4,
//  NOTE_D5, NOTE_A5, NOTE_A5, NOTE_A5, NOTE_G5, NOTE_F5,
//  
//  NOTE_E5, NOTE_C5, NOTE_A4, NOTE_G4,
//  NOTE_E5, NOTE_C5, NOTE_G4, REST, NOTE_GS4,
//  NOTE_A4, NOTE_F5, NOTE_F5, NOTE_A4,
//  NOTE_B4, NOTE_F5, NOTE_F5, NOTE_F5, NOTE_E5, NOTE_D5,
//  NOTE_C5, NOTE_E4, NOTE_E4, NOTE_C4,
//  
//  // Game over sound
//  NOTE_C5, NOTE_G4, NOTE_E4,
//  NOTE_A4, NOTE_B4, NOTE_A4, NOTE_GS4, NOTE_AS4, NOTE_GS4,
//  NOTE_G4, NOTE_D4, NOTE_E4
//};

//int durations[] = {
//  8, 8, 8, 8, 8, 8, 8,
//  4, 4, 8, 4, 
//  4, 8, 4, 4,
//  4, 4, 8, 4,
//  8, 8, 8, 4, 8, 8,
//  8, 4,8, 8, 4,
//  4, 8, 4, 4,
//  4, 4, 8, 4,
//  8, 8, 8, 4, 8, 8,
//  8, 4,8, 8, 4,
//  
//  
//  4, 8, 8, 8, 4, 8,
//  8, 8, 8, 8, 8, 8, 8, 8,
//  4, 4, 8, 4,
//  2, 2,
//  
//  4, 8, 8, 8, 4, 8,
//  8, 8, 8, 8, 8, 8, 8, 8,
//  4, 4, 8, 4,
//  2, 2,
//  
//  8, 4, 8, 8, 8, 4,
//  8, 4, 8, 2,
//  
//  8, 4, 8, 8, 8, 8, 8,
//  1, 
//  8, 4, 8, 8, 8, 4,
//  8, 4, 8, 2,
//  8, 8, 8, 8, 8, 8, 4,
//  4, 4, 4, 4, 
//  4, 8, 4, 4,
//  
//  4, 4, 8, 4,
//  8, 8, 8, 4, 8, 8,
//  8, 4, 8, 8, 4,
//  
//  4, 8, 4, 4,
//  4, 4, 8, 4,
//  8, 8, 8, 4, 8, 8,
//  8, 4, 8, 8, 4,
//  
//  8, 4, 8, 4, 4,
//  8, 4, 8, 2,
//  8, 8, 8, 8, 8, 8,
//  
//  8, 4, 8, 2,
//  8, 4, 8, 4, 4,
//  8, 4, 8, 2,
//  8, 4, 8, 8, 8, 8,
//  8, 4, 8, 2,
//  
//  8, 4, 8, 4, 4,
//  8, 4, 8, 2,
//  8, 8, 8, 8, 8, 8,
//  
//  8, 4, 8, 2,
//  8, 4, 8, 4, 4,
//  8, 4, 8, 2,
//  8, 4, 8, 8, 8, 8,
//  8, 4, 8, 2,
//  8, 4, 8, 8, 8, 8, 8,
//  1,
//  
//  8, 4, 8, 8, 8, 4,
//  8, 4, 8, 2,
//  8, 8, 8, 8, 8, 8, 4,
//  4, 4, 4, 4, 
//  8, 4, 8, 4, 4,
//  8, 4, 8, 2,
//  8, 8, 8, 8, 8, 8,
//  
//  8, 4, 8, 2,
//  8, 4, 8, 4, 4,
//  8, 4, 8, 2,
//  8, 4, 8, 8, 8, 8,
//  8, 4, 8, 2,
//  
//  //game over sound
//  4, 4, 4,
//  8, 8, 8, 8, 8, 8,
//  8, 8, 2
//};

//int melody[] = {
//  
//  // Cantina BAnd - Star wars 
//  // Score available at https://musescore.com/user/6795541/scores/1606876
//  NOTE_B4, NOTE_E5, NOTE_B4, NOTE_E5, 
//  NOTE_B4,  NOTE_E5, NOTE_B4, REST, NOTE_AS4, NOTE_B4, 
//  NOTE_B4,  NOTE_AS4, NOTE_B4, NOTE_A4, REST, NOTE_GS4, NOTE_A4, NOTE_G4,
//  NOTE_G4,  NOTE_E4, 
//  NOTE_B4, NOTE_E5, NOTE_B4, NOTE_E5, 
//  NOTE_B4,  NOTE_E5, NOTE_B4, REST,  NOTE_AS4, NOTE_B4,
//
//  NOTE_A4, NOTE_A4, NOTE_GS4, NOTE_A4,
//  NOTE_D5,  NOTE_C5, NOTE_B4, NOTE_A4,
//  NOTE_B4, NOTE_E5, NOTE_B4, NOTE_E5, 
//  NOTE_B4,  NOTE_E5, NOTE_B4, REST,  NOTE_AS4, NOTE_B4,
//  NOTE_D5, NOTE_D5, NOTE_B4, NOTE_A4,
//  NOTE_G4, NOTE_E4,
//  NOTE_E4, NOTE_G4,
//  NOTE_B4, NOTE_D5,
//
//  NOTE_F5, NOTE_E5, NOTE_AS4, NOTE_AS4, NOTE_B4, NOTE_G4, 
//};
//
//int durations[] = {
//  4, 4, 4, 4,
//  8, 4, 8, 8, 8, 8,
//  8, 8, 8, 8, 8, 8, 8, 8,
//  4, 2,
//  4, 4, 4, 4,
//  8, 4, 8, 8, 8, 8,
//
//  4, 4, 8, 4,
//  8, 4, 4, 4,
//  4, 4, 4, 4,
//  8, 4, 8, 8, 8, 8, 
//  4, 4, 8, 4,
//  4, 2,
//  2, 2, 
//  2, 2,
//
//  4, 4, 8, 8, 4, 4,
//};

int melody[] = {
  NOTE_E4, NOTE_G4, NOTE_A4, NOTE_A4, REST,
  NOTE_A4, NOTE_B4, NOTE_C5, NOTE_C5, REST,
  NOTE_C5, NOTE_D5, NOTE_B4, NOTE_B4, REST,
  NOTE_A4, NOTE_G4, NOTE_A4, REST,
  
  NOTE_E4, NOTE_G4, NOTE_A4, NOTE_A4, REST,
  NOTE_A4, NOTE_B4, NOTE_C5, NOTE_C5, REST,
  NOTE_C5, NOTE_D5, NOTE_B4, NOTE_B4, REST,
  NOTE_A4, NOTE_G4, NOTE_A4, REST,
  
  NOTE_E4, NOTE_G4, NOTE_A4, NOTE_A4, REST,
  NOTE_A4, NOTE_C5, NOTE_D5, NOTE_D5, REST,
  NOTE_D5, NOTE_E5, NOTE_F5, NOTE_F5, REST,
  NOTE_E5, NOTE_D5, NOTE_E5, NOTE_A4, REST,
  
  NOTE_A4, NOTE_B4, NOTE_C5, NOTE_C5, REST,
  NOTE_D5, NOTE_E5, NOTE_A4, REST,
  NOTE_A4, NOTE_C5, NOTE_B4, NOTE_B4, REST,
  NOTE_C5, NOTE_A4, NOTE_B4, REST,
  
  NOTE_A4, NOTE_A4,
  //Repeat of first part
  NOTE_A4, NOTE_B4, NOTE_C5, NOTE_C5, REST,
  NOTE_C5, NOTE_D5, NOTE_B4, NOTE_B4, REST,
  NOTE_A4, NOTE_G4, NOTE_A4, REST,
  
  NOTE_E4, NOTE_G4, NOTE_A4, NOTE_A4, REST,
  NOTE_A4, NOTE_B4, NOTE_C5, NOTE_C5, REST,
  NOTE_C5, NOTE_D5, NOTE_B4, NOTE_B4, REST,
  NOTE_A4, NOTE_G4, NOTE_A4, REST,
  
  NOTE_E4, NOTE_G4, NOTE_A4, NOTE_A4, REST,
  NOTE_A4, NOTE_C5, NOTE_D5, NOTE_D5, REST,
  NOTE_D5, NOTE_E5, NOTE_F5, NOTE_F5, REST,
  NOTE_E5, NOTE_D5, NOTE_E5, NOTE_A4, REST,
  
  NOTE_A4, NOTE_B4, NOTE_C5, NOTE_C5, REST,
  NOTE_D5, NOTE_E5, NOTE_A4, REST,
  NOTE_A4, NOTE_C5, NOTE_B4, NOTE_B4, REST,
  NOTE_C5, NOTE_A4, NOTE_B4, REST,
  //End of Repeat
  
  NOTE_E5, REST, REST, NOTE_F5, REST, REST,
  NOTE_E5, NOTE_E5, REST, NOTE_G5, REST, NOTE_E5, NOTE_D5, REST, REST,
  NOTE_D5, REST, REST, NOTE_C5, REST, REST,
  NOTE_B4, NOTE_C5, REST, NOTE_B4, REST, NOTE_A4,
  
  NOTE_E5, REST, REST, NOTE_F5, REST, REST,
  NOTE_E5, NOTE_E5, REST, NOTE_G5, REST, NOTE_E5, NOTE_D5, REST, REST,
  NOTE_D5, REST, REST, NOTE_C5, REST, REST,
  NOTE_B4, NOTE_C5, REST, NOTE_B4, REST, NOTE_A4
};

int durations[] = {
  8, 8, 4, 8, 8,
  8, 8, 4, 8, 8,
  8, 8, 4, 8, 8,
  8, 8, 4, 8,
  
  8, 8, 4, 8, 8,
  8, 8, 4, 8, 8,
  8, 8, 4, 8, 8,
  8, 8, 4, 8,
  
  8, 8, 4, 8, 8,
  8, 8, 4, 8, 8,
  8, 8, 4, 8, 8,
  8, 8, 8, 4, 8,
  
  8, 8, 4, 8, 8,
  4, 8, 4, 8,
  8, 8, 4, 8, 8,
  8, 8, 4, 4,
  
  4, 8,
  //Repeat of First Part
  8, 8, 4, 8, 8,
  8, 8, 4, 8, 8,
  8, 8, 4, 8,
  
  8, 8, 4, 8, 8,
  8, 8, 4, 8, 8,
  8, 8, 4, 8, 8,
  8, 8, 4, 8,
  
  8, 8, 4, 8, 8,
  8, 8, 4, 8, 8,
  8, 8, 4, 8, 8,
  8, 8, 8, 4, 8,
  
  8, 8, 4, 8, 8,
  4, 8, 4, 8,
  8, 8, 4, 8, 8,
  8, 8, 4, 4,
  //End of Repeat
  
  4, 8, 4, 4, 8, 4,
  8, 8, 8, 8, 8, 8, 8, 8, 4,
  4, 8, 4, 4, 8, 4,
  8, 8, 8, 8, 8, 2,
  
  4, 8, 4, 4, 8, 4,
  8, 8, 8, 8, 8, 8, 8, 8, 4,
  4, 8, 4, 4, 8, 4,
  8, 8, 8, 8, 8, 2
};

#define laser_control_pin 18
#define photodetector_pin A14
#define buzzer_pin 12

void setup()
{
  Serial.begin(115200);
  pinMode(laser_control_pin, OUTPUT);
}

int detector_binary_array[20] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
int* binary_note_array[sizeof(melody)];
int* binary_duration_array[sizeof(durations)];
int bits_per_packet = 20;
int note_bits = 16; //bits 1-16 of the packet
int duration_bits = 4; //bits 17-20 of the packet
int pauseBetweenNotes;
int duration;

int notes_received[sizeof(melody)];
int durations_received[sizeof(durations)];

void loop()
{
    
//    for(int i=0; i < sizeof(melody)/sizeof(melody[0]); i++){
//      binary_note_array[i] = intToBinary(melody[i]);
//
//      for(int i=0; i < sizeof(durations)/sizeof(durations[0]); i++){
//      binary_note_array[i] |= binary_note_array;
//      }
//
//       for(int i = 0; i < 20; i++)
//    {
//        Serial.print(binary_note_array[i]);
//    };
    delay(1000);

    for(int i = 0; i < sizeof(melody); i++)
    {
      binary_note_array[i] = intToBinary(melody[i], durations[i]);

//      for(int j = 0; j < 20; j++)
//      {
//        Serial.print((binary_note_array[i][j]));
//      }
//      Serial.println("");
    }
  delay(1000);
  uint32_t packet = 0;    

    for(int i = 0; i < sizeof(melody); i++)
    {
        packet = 0;
        for(int j = 0; j < bits_per_packet; j++)
    {
        if(binary_note_array[i][j] == 1)
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

        packet = packet << 1;
        
        if(sensorValue > 2000)
        {
            packet = packet | 1;
        }
        else
        {
            packet = packet | 0;
        }
    }
      int note = packet >> 4;
      int duration = packet & 15; //bitwise and to save only duration bits
      notes_received[i] = note;
      durations_received[i] = duration;
  
    //to calculate the note duration, take one second divided by the note type.
    //e.g. quarter note = 1000 / 4, eighth note = 1000/8, etc
    duration = 1000 / durations_received[i];
    tone(buzzer_pin, notes_received[i], duration);

    //to distinguish the notes, set a minimum time between them.
    //the note's duration + 30% seems to work well:
    pauseBetweenNotes = duration * 1.30;
    delay(pauseBetweenNotes);
    
    //stop the tone playing:
    noTone(buzzer_pin);

    }
//    for (int note_num = 0; note_num < sizeof(notes_received); note_num++) {
//    //to calculate the note duration, take one second divided by the note type.
//    //e.g. quarter note = 1000 / 4, eighth note = 1000/8, etc
//    int duration = 1000 / durations_received[note_num];
//    tone(buzzer_pin, notes_received[note_num], duration);
//
//    //to distinguish the notes, set a minimum time between them.
//    //the note's duration + 30% seems to work well:
//    int pauseBetweenNotes = duration * 1.30;
//    delay(pauseBetweenNotes);
//    
//    //stop the tone playing:
//    noTone(buzzer_pin);
//    }

    memset(notes_received, 0, sizeof(melody));
    memset(durations_received, 0, sizeof(melody));
    
    stop_program();
    //digitalWrite(laser_control_pin, LOW); // turn off laser after sending a packet

//    int DecValue = binaryToInt(detector_binary_array,16);
//    Serial.println(DecValue);
    
}

void stop_program()
{
  Serial.print("done");
  while(1);
}


//creates a 20 bit number
//16 bits are the note
//4 bits are the duration of said note
int* intToBinary(int note, int duration)
{
    int* bin = new int[20];
    int note_size = 16;
    int duration_size = 4;
    int total_size = 20;

    //converting note to binary and storing it in first 16 bits
    for(int i = note_size - 1; i >= 0; i--)
    {
        int bit = (note >> i) & 1;
        bin[note_size - 1 - i] = bit;
    }

    //converting duration to binary and storing it in the last 4 bits
    for(int i = duration_size - 1; i >= 0; i--)
    {
        int bit = (duration >> i) & 1;
        bin[total_size - 1 - i] = bit;
    }

//    for(int j=0; j < size; j++)
//    {
//      Serial.print((bin[j]));
//    }
//    Serial.println("");
    return bin;
}

int binaryToInt(int* bin, int size)
{
    int result = 0;
    for(int i = 0; i < size; i++)
    {
        result = (result << 1) | bin[i];
    }

    return result;
}
