#define photodetectorPin A0

int ambientReading = 0;
bool readFinished;
int dataArray[16] = {0};


void setup()
{
    pinMode(photodetectorPin, INPUT);
    Serial.begin(9600);

    readFinished = false;

    ambientReading = analogRead(photodetectorPin);
    Serial.print("Ambient Reading: ");
    Serial.println(ambientReading);

    attachInterrupt(digitalPinToInterrupt(photodetectorPin), handleInterrupt, RISING);
}


void loop()
{
    if(readFinished)
    {
        for(int i = 0; i < 16; i++)
        {
            Serial.print(dataArray[i]);
        }

        Serial.println("");
        readFinished = false;
    }
}


void handleInterrupt()
{
    noInterrupts(); // disable interrupts

    int dataReceived = analogRead(photodetectorPin);

    if(dataReceived > ambientReading + 100)
    {
        for(int i = 15; i >= 0; i--)
        {
            if(analogRead(photodetectorPin) > ambientReading + 100)
            {
                dataArray[i] = 1;
            }
            else
            {
                dataArray[i] = 0;
            }
        
            delayMicroseconds(100);
        }
    }

    readFinished = true;
    interrupts(); // enable interrupts
}