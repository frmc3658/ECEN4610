using namespace std;

#include <vector>

#define diode_read_pin 33
hw_timer_t *My_timer = NULL;
volatile bool flag = false;
volatile bool edge = true;
volatile int i = 15;
volatile int curr = 0;
volatile int timeSince = micros();
int period = 100000;
vector<bool> data; //0b1101010101100111; //0xd567 - 1101 0101 0110 0111 //0b1010101010101010;//

void IRAM_ATTR onTimer()
{
  timeSince = micros() - curr;
  if(timeSince > period)
  {
    data.push_back((bool)digitalRead(diode_read_pin));
    curr = micros();
    //delayMicroseconds(10000);
    i--;
    if(i == 0) flag = true;
  }
}

void edgeCase()
{
  if(edge) timerAlarmEnable(My_timer);
}

void setup() 
{
  Serial.begin(9600);
  pinMode(diode_read_pin, INPUT);
  attachInterrupt(digitalPinToInterrupt(diode_read_pin), edgeCase, RISING);
  My_timer = timerBegin(0, 8000, true);
  timerAttachInterrupt(My_timer, &onTimer, true);
  timerAlarmWrite(My_timer, 100, true);
}

void loop() 
{
  Serial.println("Begin!");
  // initialize clock by sending high pulse
  //Serial.println("Start!");
  while(true){
    if(flag){
    timerAlarmDisable(My_timer);
    //Serial.println("Done!");
    flag = false;
    edge = true;
    for(int k = 0; k < data.size(); k++){
      Serial.print(data[k]);
    }
    Serial.println("");
    data.clear();
    break;
    }
  }
  //delayMicroseconds(5000000); // delay between messages *10000
}

