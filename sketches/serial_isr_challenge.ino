// ============================================================
// Studio 12: Remote Control - Challenge Activity
// serial_isr_challenge.ino
// ============================================================

#include <avr/io.h>
#include <avr/interrupt.h>
#include <string.h>
#include <stdint.h>
#include <util/delay.h>

// ============================================================
// PACKET DEFINITIONS
// ============================================================

#define MAX_STR_LEN   32
#define PARAMS_COUNT  16

typedef enum {
  PACKET_TYPE_COMMAND  = 0,
  PACKET_TYPE_RESPONSE = 1,
  PACKET_TYPE_MESSAGE  = 2,
} TPacketType;

typedef enum {
  COMMAND_ESTOP = 0,
} TCommandType;

typedef enum {
  RESP_OK     = 0,
  RESP_STATUS = 1,
} TResponseType;

typedef struct {
  uint8_t  packetType;
  uint8_t  command;
  uint8_t  dummy[2];
  char     data[MAX_STR_LEN];
  uint32_t params[PARAMS_COUNT];
} TPacket;

typedef enum {
  STATE_RUNNING = 0,
  STATE_STOPPED = 1,
} TButtonState;

// ============================================================
// CIRCULAR TX BUFFER
// ============================================================

#define TX_BUFFER_SIZE  128
#define TX_BUFFER_MASK  (TX_BUFFER_SIZE - 1)

static volatile uint8_t tx_buf[TX_BUFFER_SIZE];
static volatile uint8_t tx_head = 0;
static volatile uint8_t tx_tail = 0;

// ============================================================
// CIRCULAR RX BUFFER
// ============================================================

#define RX_BUFFER_SIZE  128
#define RX_BUFFER_MASK  (RX_BUFFER_SIZE - 1)

static volatile uint8_t rx_buf[RX_BUFFER_SIZE];
static volatile uint8_t rx_head = 0;
static volatile uint8_t rx_tail = 0;

// ============================================================
// Part A: txEnqueue
// ============================================================

bool txEnqueue(const uint8_t *data, uint8_t len) {
  uint8_t tail = tx_tail;  // atomic uint8_t read on AVR
  uint8_t head = tx_head;

  // -1 ensures head never catches up to tail (head==tail means empty)
  uint8_t free_slots = (tail - head - 1) & TX_BUFFER_MASK;
  if (free_slots < len) {
    return false;
  }

  for (uint8_t i = 0; i < len; i++) {
    tx_buf[head] = data[i];
    head = (head + 1) & TX_BUFFER_MASK;
  }

  // Commit new head only after all bytes are written
  tx_head = head;

  // Enable UDRE interrupt atomically (UCSR0B is above 0x1F,
  // so |= is a non-atomic read-modify-write on AVR)
  uint8_t sreg = SREG;
  cli();
  UCSR0B |= (1 << UDRIE0);
  SREG = sreg;

  return true;
}

// ============================================================
// Part B: USART Data Register Empty ISR
// ============================================================

ISR(USART_UDRE_vect) {
  // Send first, then check — this ISR only fires when UDRIE0 is
  // set, and UDRIE0 is only set when there are bytes in the buffer
  UDR0 = tx_buf[tx_tail];
  tx_tail = (tx_tail + 1) & TX_BUFFER_MASK;

  if (tx_tail == tx_head) {
    // Buffer now empty — disable UDRE interrupt to stop
    // the ISR from firing continuously on an idle transmitter
    UCSR0B &= ~(1 << UDRIE0);
  }
}

// ============================================================
// BARE-METAL USART INITIALISATION (provided, do not modify)
// ============================================================

void usartInit(uint16_t ubrr) {
  UBRR0H = (uint8_t)(ubrr >> 8);
  UBRR0L = (uint8_t)(ubrr);
  UCSR0B = (1 << TXEN0) | (1 << RXEN0) | (1 << RXCIE0);
  UCSR0C = (1 << UCSZ01) | (1 << UCSZ00);
}

// ============================================================
// Part C: rxDequeue
// ============================================================

bool rxDequeue(uint8_t *data, uint8_t len) {
  uint8_t head = rx_head;  // atomic uint8_t read on AVR
  uint8_t tail = rx_tail;

  uint8_t available = (head - tail) & RX_BUFFER_MASK;
  if (available < len) {
    return false;
  }

  for (uint8_t i = 0; i < len; i++) {
    data[i] = rx_buf[tail];
    tail = (tail + 1) & RX_BUFFER_MASK;
  }

  // Commit new tail only after all bytes are copied out
  rx_tail = tail;

  return true;
}

// ============================================================
// Part D: RX Complete ISR
// ============================================================

ISR(USART_RX_vect) {
  // Must read UDR0 first — this clears the interrupt flag.
  // Leaving UDR0 unread keeps the interrupt pending forever
  // and locks up the USART.
  uint8_t byte = UDR0;

  uint8_t next = (rx_head + 1) & RX_BUFFER_MASK;

  if (next == rx_tail) {
    // Buffer full — discard byte rather than overwriting unread data.
    // UDR0 is already read above so no lockup occurs.
    return;
  }

  rx_buf[rx_head] = byte;
  rx_head = next;
}

// ============================================================
// Part E: setup() and loop()
// ============================================================

void setup() {
  // 9600 baud at 16 MHz: ubrr = (16000000 / (16 * 9600)) - 1 = 103
  usartInit(103);
  sei();  // enable global interrupts so both ISRs can fire
}

void loop() {
  // --- TX: send a STATUS packet once per loop ---
  TPacket statusPkt;
  memset(&statusPkt, 0, sizeof(statusPkt));
  statusPkt.packetType = PACKET_TYPE_RESPONSE;
  statusPkt.command    = RESP_STATUS;
  statusPkt.params[0]  = STATE_RUNNING;

  // Retry until buffer accepts the full 100-byte packet
  while (!txEnqueue((const uint8_t *)&statusPkt, sizeof(statusPkt))) {
    _delay_ms(1);
  }

  // --- RX: check for a complete incoming packet ---
  TPacket rxPkt;
  if (rxDequeue((uint8_t *)&rxPkt, sizeof(rxPkt))) {
    if (rxPkt.packetType == PACKET_TYPE_COMMAND &&
        rxPkt.command    == COMMAND_ESTOP) {

      TPacket okPkt;
      memset(&okPkt, 0, sizeof(okPkt));
      okPkt.packetType = PACKET_TYPE_RESPONSE;
      okPkt.command    = RESP_OK;
      okPkt.params[0]  = 0;

      while (!txEnqueue((const uint8_t *)&okPkt, sizeof(okPkt))) {
        _delay_ms(1);
      }
    }
  }

  // Rate-limit to ~1 status packet per second
  _delay_ms(1000);
}