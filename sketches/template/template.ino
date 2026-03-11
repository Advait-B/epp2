// ============================================================
// Studio 12: Remote Control
// Arduino Template - template.ino
//
// This sketch is used for Activity 3 (Simple Command Protocol
// and E-Stop).  Complete the TODO sections marked below.
//
// ============================================================

#include <stdint.h>
#include <string.h>
#include <avr/io.h>
#include <avr/interrupt.h>

// ============================================================
// PACKET DEFINITIONS  (must match pi_template.py)
// ============================================================

#define MAX_STR_LEN   32
#define PARAMS_COUNT  16
#define LED_DELAY 100

// The turn variable decides whose turn it is to go now.
static volatile int turn=0;

// The _timerTicks variable maintains a count of how many 100us ticks have passed by.
volatile static unsigned long _timerTicks = 0;

// Our last time and current time variables
static unsigned long _lastTime = 0;
static unsigned long _currentTime = 0;


// The THRESHOLD value for debouncing
#define THRESHOLD   50  // We are using 100us ticks so this is equivalent to 5 ms.


typedef enum {
  PACKET_TYPE_COMMAND  = 0,
  PACKET_TYPE_RESPONSE = 1,
  PACKET_TYPE_MESSAGE  = 2,
} TPacketType;

// Commands sent from Pi to Arduino
typedef enum {
  COMMAND_ESTOP = 0,
} TCommandType;

// Responses sent from Arduino to Pi
typedef enum {
  RESP_OK     = 0,
  RESP_STATUS = 1,
} TResponseType;

// TPacket: 1 + 1 + 2 + 32 + (16 * 4) = 100 bytes.
// The dummy[2] field ensures params (uint32_t, 4-byte aligned) starts
// at offset 4, so the layout is identical on every platform.
typedef struct {
  uint8_t  packetType;               // TPacketType
  uint8_t  command;                  // TCommandType / TResponseType
  uint8_t  dummy[2];                 // alignment padding
  char     data[MAX_STR_LEN];        // optional null-terminated string
  uint32_t params[PARAMS_COUNT];     // numeric parameters
} TPacket;

// ============================================================
// BUTTON / E-STOP STATE
// ============================================================

typedef enum {
  STATE_RUNNING = 0,
  STATE_STOPPED = 1,
} TButtonState;

volatile TButtonState buttonState  = STATE_RUNNING;
volatile bool         stateChanged = false;

// ============================================================
// PACKET UTILITIES
// ============================================================

// Transmit a TPacket as raw bytes.
void sendPacket(TPacket *pkt) {
  Serial.write((uint8_t *)pkt, sizeof(TPacket));
}

// Accumulate incoming bytes until a complete TPacket has
// arrived.  Returns true (and fills *pkt) when done.
bool receivePacket(TPacket *pkt) {
  static uint8_t buf[sizeof(TPacket)];
  static uint8_t count = 0;

  while (Serial.available() > 0) {
    buf[count++] = (uint8_t)Serial.read();
    if (count == sizeof(TPacket)) {
      memcpy(pkt, buf, sizeof(TPacket));
      count = 0;
      return true;
    }
  }
  return false;
}

// TODO (Activity 3): Implement sendResponse().
//
// Create a TPacket with:
// then call sendPacket() to transmit it.
void sendResponse(TResponseType resp, uint32_t param) {
   TPacket test = {0};
  test.packetType = PACKET_TYPE_RESPONSE;
    test.command    = resp;
    test.params[0]  = param;
    sendPacket(&test);
}


// TODO (Activity 3): Implement sendStatus().
//
// Call sendResponse correctly
void sendStatus() {
  sendResponse(RESP_STATUS, buttonState);
}

// Process an incoming COMMAND packet.
void handleCommand(TPacket *pkt) {
  if (pkt->command == COMMAND_ESTOP) {
    buttonState  = STATE_STOPPED;
    stateChanged = true;
    sendResponse(RESP_OK, 0);
  }
}

// ============================================================
// BUTTON: INTERRUPT SERVICE ROUTINE
// ============================================================
//
// TODO (Activity 3): Complete ISR to implement

// Required behaviour:
//   STATE_RUNNING + button pressed  -> STATE_STOPPED; stateChanged = true
//   STATE_STOPPED + button released -> STATE_RUNNING; stateChanged = true
//
// ISR(...) {
// YOUR CODE HERE
// }
ISR(INT0_vect){
  _currentTime = _timerTicks; // UPDATE the current time!

  if ((_currentTime - _lastTime) > THRESHOLD) {
    // If running and button is PRESSED (Pin 2 is HIGH)
    if (buttonState == STATE_RUNNING && (PIND & (1 << 2))){
      buttonState = STATE_STOPPED;
      stateChanged = true;
    }
    // If stopped and button is RELEASED (Pin 2 is LOW)
    else if (buttonState == STATE_STOPPED && !(PIND & (1 << 2))){
      buttonState = STATE_RUNNING;
      stateChanged = true;
    }
    _lastTime = _currentTime;
  }
}

ISR(TIMER2_COMPA_vect)
{
  _timerTicks++;
}

// ============================================================
// SETUP
// ============================================================
void setup() {
  Serial.begin(9600);
  // TODO (Activity 3a): Enable the button to fire an interrupt on any
  // logical change (both rising and falling edges).
  cli();
  DDRD &= ~(1 << 2); // set PD2 to input
  //PORTD |= (1 << 2);
  EIMSK |= (1 << INT0);
  EICRA = 0b00000001;
  setupTimer();
  startTimer();
  sei();
}
void setupTimer()
{
  cli();
  TCCR2A = 0b10;
  TIMSK2 = 0b10;
  TCNT2 = 0;
  OCR2A = 199;
  sei();
}

// Timer start function
void startTimer()
{
  TCCR2B = 0b10;
  // Start timer 2 here.
}

// ============================================================
// MAIN LOOP
// ============================================================

void loop() {
  // Check for incoming command packets from the Pi.
  TPacket incoming;
  if (receivePacket(&incoming)) {
    if (incoming.packetType == PACKET_TYPE_COMMAND) {
      handleCommand(&incoming);
    }
  }

  // If the button state changed (hardware ISR), send a status
  // update to the Pi.
  if (stateChanged) {
    stateChanged = false;
    sendStatus();
  }
}
