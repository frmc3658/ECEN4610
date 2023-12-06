#define laser_control_pin 18

void setup()
{
    pinMode(laser_control_pin, OUTPUT);
}

void loop()
{
    uint16_t data = 0b1101010101100111;

    // initialize clock by sending high pulse
    digitalWrite(laser_control_pin, HIGH);
    delayMicroseconds(100); //fastest analog read speed
    digitalWrite(laser_control_pin, LOW);
    delayMicroseconds(100);

    for(int i = 15; i >= 0; i--)
    {
        digitalWrite(laser_control_pin, ((data >> i) & 1));
        delayMicroseconds(100);
    }

    digitalWrite(laser_control_pin, LOW);

    delayMicroseconds(1000); // delay between messages
}