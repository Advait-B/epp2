/*
 * sensor_miniproject_template.ino
 * Studio 13: Sensor Mini-Project
 *
 * This sketch is split across three files in this folder:
 *
 *   packets.h        - TPacket protocol: enums, struct, framing constants.
 *                      Must stay in sync with pi_sensor.py.
 *
 *   serial_driver.h  - Transport layer.  Set USE_BAREMETAL_SERIAL to 0
 *                      (default) for the Arduino Serial path that works
 *                      immediately, or to 1 to use the bare-metal USART
 *                      driver (Activity 1).  Also contains the
 *                      sendFrame / receiveFrame framing code.
 *
 *   sensor_miniproject_template.ino  (this file)
 *                    - Application logic: packet helpers, E-Stop state
 *                      machine, color sensor, setup(), and loop().
 */

#include "packets.h"
#include "serial_driver.h"
#include <avr/interrupt.h>
#include <avr/io.h>

// =============================================================
// Packet helpers (pre-implemented for you)
// =============================================================

/*
 * Build a zero-initialised TPacket, set packetType = PACKET_TYPE_RESPONSE,
 * command = resp, and params[0] = param.  Then call sendFrame().
 */
static void sendResponse(TResponseType resp, uint32_t param) {
    TPacket pkt;
    memset(&pkt, 0, sizeof(pkt));
    pkt.packetType = PACKET_TYPE_RESPONSE;
    pkt.command    = resp;
    pkt.params[0]  = param;
    sendFrame(&pkt);
}

/*
 * Send a RESP_STATUS packet with the current state in params[0].
 */
static void sendStatus(TState state) {
    sendResponse(RESP_STATUS, (uint32_t)state);
}

// =============================================================
// E-Stop state machine
// =============================================================

volatile TState buttonState = STATE_RUNNING;
volatile bool   stateChanged = false;

/*
 * TODO (Activity 1): Implement the E-Stop ISR.
 *
 * Fire on any logical change on the button pin.
 * State machine (see handout diagram):
 *   RUNNING + press (pin HIGH)  ->  STOPPED, set stateChanged = true
 *   STOPPED + release (pin LOW) ->  RUNNING, set stateChanged = true
 *
 * Debounce the button.  You will also need to enable this interrupt
 * in setup() -- check the ATMega2560 datasheet for the correct
 * registers for your chosen pin.
 */


// =============================================================
// Color sensor (TCS3200)
// =============================================================
const uint8_t BUTTON_PIN = 53; // Mega external interrupt pin
#define THRESHOLD 50           // 50 x 100us = 5 ms debounce

volatile unsigned long _timerTicks = 0;
volatile unsigned long _lastTime = 0;

/*
 * TODO (Activity 2): Implement the color sensor.
 *
 * Wire the TCS3200 to the Arduino Mega and configure the output pins
 * (S0, S1, S2, S3) and the frequency output pin.
 *
 * Use 20% output frequency scaling (S0=HIGH, S1=LOW).  This is the
 * required standardised setting; it gives a convenient measurement range and
 * ensures all implementations report the same physical quantity.
 *
 * Use a timer to count rising edges on the sensor output over a fixed
 * window (e.g. 100 ms) for each color channel (red, green, blue).
 * Convert the edge count to hertz before sending:
 *   frequency_Hz = edge_count / measurement_window_s
 * For a 100 ms window: frequency_Hz = edge_count * 10.
 *
 * Implement a function that measures all three channels and stores the
 * frequency in Hz in three variables.
 *
 * Define your own command and response types in packets.h (and matching
 * constants in pi_sensor.py), then handle the command in handleCommand()
 * and send back the channel frequencies (in Hz) in a response packet.
 *
 * Example skeleton:
 *
 *   static void readColorChannels(uint32_t *r, uint32_t *g, uint32_t *b) {
 *       // Set S2/S3 for each channel, measure edge count, multiply by 10
 *       *r = measureChannel(0, 0) * 10;  // red,   in Hz
 *       *g = measureChannel(1, 1) * 10;  // green, in Hz
 *       *b = measureChannel(0, 1) * 10;  // blue,  in Hz
 *   }
 */

// ---------------- COLOR SENSOR PINS ----------------

#define S0 4
#define S1 5
#define S2 6
#define S3 7
#define SENSOR_OUT 8

// ---------------- SENSOR MEASUREMENT ----------------

static uint32_t measureChannel(uint8_t s2, uint8_t s3) {

    digitalWrite(S2, s2);
    digitalWrite(S3, s3);

    uint32_t count = 0;
    unsigned long start = millis();

    while (millis() - start < 100) {
        if (digitalRead(SENSOR_OUT) == HIGH) {
            count++;
            while (digitalRead(SENSOR_OUT) == HIGH);
        }
    }

    return count;
}

static void readColorChannels(uint32_t *r, uint32_t *g, uint32_t *b) {

    *r = measureChannel(LOW, LOW) * 10;
    *g = measureChannel(HIGH, HIGH) * 10;
    *b = measureChannel(LOW, HIGH) * 10;
}

static void setupTimer() {
    cli();
    TCCR2A = (1 << WGM21);  // CTC mode
    TCCR2B = 0;             // stop timer for now
    TIMSK2 = (1 << OCIE2A); // enable compare match A interrupt
    TCNT2 = 0;
    OCR2A = 199; // 100 us at 16 MHz, prescaler 8
    sei();
}

static void startTimer() {
    TCCR2B = (1 << CS21); // prescaler 8
}

ISR(TIMER2_COMPA_vect) { _timerTicks++; }

void buttonISR() {
    unsigned long now = _timerTicks;

    if ((now - _lastTime) > THRESHOLD) {
        bool pressed = (digitalRead(BUTTON_PIN) == HIGH);

        if (buttonState == STATE_RUNNING && pressed) {
            buttonState = STATE_STOPPED;
            stateChanged = true;
        } else if (buttonState == STATE_STOPPED && !pressed) {
            buttonState = STATE_RUNNING;
            stateChanged = true;
        }
        _lastTime = now;
    }
}

// =============================================================
// Command handler
// =============================================================

/*
 * Dispatch incoming commands from the Pi.
 *
 * COMMAND_ESTOP is pre-implemented: it sets the Arduino to STATE_STOPPED
 * and sends back RESP_OK followed by a RESP_STATUS update.
 *
 * TODO (Activity 2): add a case for your color sensor command.
 *   Call your color-reading function, then send a response packet with
 *   the channel frequencies in Hz.
 */
static void handleCommand(const TPacket *cmd) {
    if (cmd->packetType != PACKET_TYPE_COMMAND) return;

    switch (cmd->command) {
        case COMMAND_ESTOP:
            cli();
            buttonState  = STATE_STOPPED;
            stateChanged = false;
            sei();
            {
                TPacket pkt;
                memset(&pkt, 0, sizeof(pkt));
                pkt.packetType = PACKET_TYPE_RESPONSE;
                pkt.command    = RESP_OK;
                strncpy(pkt.data, "This is a debug message", sizeof(pkt.data) - 1);
                pkt.data[sizeof(pkt.data) - 1] = '\0';
                sendFrame(&pkt);
            }
            sendStatus(STATE_STOPPED);
            break;

        // ---------------- COLOR SENSOR COMMAND ----------------

        case COMMAND_COLOR: {

            uint32_t r, g, b;
            readColorChannels(&r, &g, &b);

            TPacket pkt;
            memset(&pkt, 0, sizeof(pkt));
            pkt.packetType = PACKET_TYPE_RESPONSE;
            pkt.command = RESP_COLOR;
            pkt.params[0] = r;
            pkt.params[1] = g;
            pkt.params[2] = b;

            sendFrame(&pkt);

            break;
        }
    }
}

// =============================================================
// Arduino setup() and loop()
// =============================================================

void setup() {

#if USE_BAREMETAL_SERIAL
    usartInit(103);
#else
    Serial.begin(9600);
#endif

    // ----------- COLOR SENSOR PIN SETUP -----------

    pinMode(S0, OUTPUT);
    pinMode(S1, OUTPUT);
    pinMode(S2, OUTPUT);
    pinMode(S3, OUTPUT);
    pinMode(SENSOR_OUT, INPUT);
    digitalWrite(S0, HIGH);
    digitalWrite(S1, LOW);

     pinMode(BUTTON_PIN, INPUT); // assumes press = HIGH, release = LOW
     setupTimer();
     startTimer();
     attachInterrupt(digitalPinToInterrupt(BUTTON_PIN), buttonISR, CHANGE);

    sei();
}

void loop() {

    if (stateChanged) {
        cli();
        TState state = buttonState;
        stateChanged = false;
        sei();
        sendStatus(state);
    }

    TPacket incoming;
    if (receiveFrame(&incoming)) {
        handleCommand(&incoming);
    }
}