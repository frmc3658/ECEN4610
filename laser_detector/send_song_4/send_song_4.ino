#include <Arduino.h>
#include <iostream>

#include "pirates.h"
#include "songpacket.h"

#define laser_control_pin       18
#define A14                     (uint8_t)13u
#define photodetector_pin       A14
#define buzzer_pin              12

void setup()
{
  Serial.begin(115200);
  pinMode(laser_control_pin, OUTPUT);
}

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
    delay(1000);

    for(int i = 0; i < sizeof(melody); i++)
    {
      binary_note_array[i] = intToBinary(melody[i], durations[i]);
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
            
            // Sensor threshold
            if(sensorValue > 2000)
            {
                packet |= 1;
            }
            else
            {
                packet |= 0;
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

    // clear the notes and duration buffers
    memset(notes_received, 0, sizeof(melody));
    memset(durations_received, 0, sizeof(melody));
    
    stop_program();
}

void stop_program()
{
  Serial.print("done");
  while(1){}
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
