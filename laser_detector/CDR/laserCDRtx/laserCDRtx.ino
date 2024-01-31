/* GPIO Pin Definitions */
#define LASER_CTRL_PIN      18
/* Serial Definitions */
#define SERIAL_BAUD         9600
/* Clock Definitions */
#define CLK0                0
#define CLK_DIV             80
#define CLK_COUNTUP         true
/* Interrupt Definitions */
#define INT_ON_EDGE         true
#define CLK_CNT_THRESHOLD   10000
#define CLK_ALARM_RELOAD    true
/* Packet Definitions */
#define NUM_PACKET_BITS     32
#define PACKET_PREAMBLE     (uint32_t)0xBD  //  0b1011 1101 = 189
#define PACKET_EMPTY_DATA   (uint32_t)0x00  //  0b0000 0000 = 0
#define PACKET_EPILOGUE     (uint32_t)0xDB  //  0b1101 1011 = 219
#define PACKET_FRAME        ((PACKET_PREAMBLE << 24) | (PACKET_EMPTY_DATA << 8) | PACKET_EPILOGUE)

// Initialize clock timer
hw_timer_t* myTimer = NULL;

// Initialize packet
volatile bool packetSent    = false;
volatile uint32_t data      = 0b1101010101100111; // 0b1101 0101 0110 0111 = 0xD567 = 54631
volatile int numBits        = NUM_PACKET_BITS;
volatile uint32_t packet    = (data << 8) | PACKET_FRAME;
// ISR function
void IRAM_ATTR onTimer()
{
  // Decrement the number of bits
  numBits -= 1;

  // Send bit
  digitalWrite(LASER_CTRL_PIN, ((packet >> numBits) & 1));

  // If all bits have been sent, raise flag
  if(numBits == 0)
  {
    packetSent = true;
  }
}


void setup() 
{
  // Set serial baud rate
  Serial.begin(SERIAL_BAUD);

  // Initialize Laser Control pin as output
  pinMode(LASER_CTRL_PIN, OUTPUT);

  // Start clock
  // Freq = CLK / DIV = 80MHz / 8 = 10MHz
  // Period =  DIV / Freq = 8 / 80MHz = 10us
  myTimer = timerBegin(CLK0, CLK_DIV, CLK_COUNTUP);

  // Initialize clock interrupts
  timerAttachInterrupt(myTimer, &onTimer, INT_ON_EDGE);
  timerAlarmWrite(myTimer, CLK_CNT_THRESHOLD, CLK_ALARM_RELOAD);
  timerAlarmEnable(myTimer);

  // Reset signal to low between packets
  digitalWrite(LASER_CTRL_PIN, LOW);
}

void loop() 
{
  while(1)
  {
    // If packet has sent...
    if(packetSent == true)
    {
      // ... Lower packetSent flag
      packetSent = false;

      // Prepare for next packet
      numBits = NUM_PACKET_BITS;
      break;
    }
  }

  // Reset signal to low between packets
  digitalWrite(LASER_CTRL_PIN, LOW);
}

