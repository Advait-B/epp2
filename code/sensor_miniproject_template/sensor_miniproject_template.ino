#include "packets.h"
#include "serial_driver.h"
#include <avr/interrupt.h>
#include <avr/io.h>

// =============================================================
// Packet helpers (pre-implemented for you)
// =============================================================

static void sendResponse(TResponseType resp, uint32_t param) {
    TPacket pkt;
    memset(&pkt, 0, sizeof(pkt));
    pkt.packetType = PACKET_TYPE_RESPONSE;
    pkt.command = resp;
    pkt.params[0] = param;
    sendFrame(&pkt);
}

static void sendStatus(TState state) {
    sendResponse(RESP_STATUS, (uint32_t)state);
}

// =============================================================
// E-Stop state machine
// =============================================================

volatile TState buttonState = STATE_RUNNING;
volatile bool stateChanged = false;

const uint8_t BUTTON_PIN = 53; // Mega external interrupt pin
#define THRESHOLD 50           // 50 x 100us = 5 ms debounce

volatile unsigned long _timerTicks = 0;
volatile unsigned long _lastTime = 0;

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

static void handleCommand(const TPacket *cmd) {
    if (cmd->packetType != PACKET_TYPE_COMMAND)
        return;

    switch (cmd->command) {
    case COMMAND_ESTOP:
        cli();
        buttonState = STATE_STOPPED;
        stateChanged = false;
        sei();
        sendResponse(RESP_OK, 0);
        sendStatus(STATE_STOPPED);
        break;
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
