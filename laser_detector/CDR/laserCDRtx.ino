#define laser_control_pin 18
hw_timer_t *My_timer = NULL;
volatile bool flag = false;
volatile int i = 15;
uint16_t data = 0b1101010101100111; //0xd567 - 1101 0101 0110 0111 //0b1010101010101010;//

void IRAM_ATTR onTimer()
{
  digitalWrite(laser_control_pin, ((data >> i) & 1));
  //delayMicroseconds(10000);
  i--;
  if(i == 0) flag = true;
}

void setup() 
{
  Serial.begin(9600);
  pinMode(laser_control_pin, OUTPUT);
  My_timer = timerBegin(0, 8000, true);
  timerAttachInterrupt(My_timer, &onTimer, true);
  timerAlarmWrite(My_timer, 1000, true);
}

void loop() 
{
  Serial.println("Begin.");
  // initialize clock by sending high pulse
  digitalWrite(laser_control_pin, HIGH);
  delayMicroseconds(1000); //fastest analog read speed 
  digitalWrite(laser_control_pin, LOW);
  delayMicroseconds(1000);
  Serial.println("Start!");
  timerAlarmEnable(My_timer);
  while(true){
    if(flag){
    timerAlarmDisable(My_timer);
    Serial.println("Done!");
    flag = false;
    break;
    }
  }
  digitalWrite(laser_control_pin, LOW);

  delayMicroseconds(5000000); // delay between messages *10000
}

