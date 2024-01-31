#define SERIAL_BAUD         9600
#define DIODE_PIN           18
#define CLK0                0
#define CLK_DIV             80
#define CLK_COUNTUP         true
#define INT_ON_LEVEL        false
#define INT_ON_EDGE         true
#define CLK_CNT_THRESHOLD   100000
#define CLK_ALARM_RELOAD    true
#define NUM_PACKET_BITS     32
#define PACKET_PREAMBLE     (uint32_t)0xBD  //  0b1011 1101 = 189
#define PACKET_EPILOGUE     (uint32_t)0xDB  //  0b1101 1011 = 219


// // Initialize clock timer
hw_timer_t* myTimer = NULL;

// Initialize 
volatile int numBits            = 32;
volatile bool previousBit       = 0;
volatile uint32_t packet        = 0;
volatile uint32_t printPacket   = 0;
volatile bool packetReceived    = false;

// ISR function
void IRAM_ATTR onTimer()
{
  // Decrement the number of bits
  numBits -= 1;

  // 
  packet |= ((digitalRead(DIODE_PIN)) << numBits);

  if(numBits == 0)
  {
    packetReceived = true;
    printPacket = packet;
    packet = 0;
    numBits = NUM_PACKET_BITS;
  }
}


void setup()
{
  // Set serial baud rate
  Serial.begin(SERIAL_BAUD);

  // Initialize Diode pin as input
  pinMode(DIODE_PIN, INPUT);

  // Start clock
  myTimer = timerBegin(CLK0, CLK_DIV, CLK_COUNTUP);

  // Initialize clock interrupts
  timerAttachInterrupt(myTimer, &onTimer, INT_ON_EDGE);
  timerAlarmWrite(myTimer, CLK_CNT_THRESHOLD, CLK_ALARM_RELOAD);
  timerAlarmEnable(myTimer);
}


void loop()
{
  // If packet Received...
  if(packetReceived == true)
  {
    // ... Lower flag
    packetReceived = false;

    // Print received packet
    Serial.println("Packet Received");
    Serial.println(printPacket);

    // Reset print packet
    printPacket = 0;
  }
}