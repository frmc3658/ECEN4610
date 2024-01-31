uint16_t data = 0b1101001101011101;

void setup()
{
  Serial.begin(115200);
}

void loop()
{
  Serial.write(data);
}