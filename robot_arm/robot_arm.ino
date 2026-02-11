#include <Arduino.h>

// pin definitions (A0-A3 = PC0-PC3)
#define BASE_PIN 0
#define SHLD_PIN 1
#define ELBW_PIN 2
#define GRIP_PIN 3

// servo min & max pulse
#define MIN_PULSE 600
#define MAX_PULSE 2400

// empirically tested servo limits
#define BASE_MIN 0
#define BASE_MAX 175
#define SHLD_MIN 80
#define SHLD_MAX 155
#define ELBW_MIN 105
#define ELBW_MAX 175
#define GRIP_MIN 20
#define GRIP_MAX 50

// checkpoints for each servo's starting position in the 20ms period
#define BASE_CHECKPOINT 0
#define SHLD_CHECKPOINT 10000
#define ELBW_CHECKPOINT 20000
#define GRIP_CHECKPOINT 30000

// pulse widths in timer ticks (volatile for ISR)
volatile int pulse_widths[4];

// counter for the ISR state machine
volatile int stage = 0;

// movement tracking variables
int current_angles[4] = {90, 125, 90, 45};
int target_angles[4] = {90, 125, 90, 45};
unsigned long last_move_time[4] = {0, 0, 0, 0};

// velocity control: ms delay between each 1-degree step
int step_delay = 10;

// helper function to handle 3-digit parsing from 4-char strings
int parse3(const String *s) {
  if (!s) return -1;
  if (s->length() != 3) return -1;
  if (!isDigit(s->charAt(0)) || !isDigit(s->charAt(1)) || !isDigit(s->charAt(2))) return -1;
  return (s->charAt(0) - '0') * 100 + (s->charAt(1) - '0') * 10 + (s->charAt(2) - '0');
}

// clamp angle to physical limits
int constrainAngle(int idx, int angle) {
  switch (idx) {
    case BASE_PIN: return constrain(angle, BASE_MIN, BASE_MAX);
    case SHLD_PIN: return constrain(angle, SHLD_MIN, SHLD_MAX);
    case ELBW_PIN: return constrain(angle, ELBW_MIN, ELBW_MAX);
    case GRIP_PIN: return constrain(angle, GRIP_MIN, GRIP_MAX);
    default:       return constrain(angle, 0, 180);
  }
}

// convert degrees to timer ticks
int angleToPulse(int angle) {
  int us = map(angle, 0, 180, MIN_PULSE, MAX_PULSE);
  return us * 2; 
}

// set all targets to pre-defined home positions
void homeAll() {
  target_angles[0] = constrainAngle(0, 90);
  target_angles[1] = constrainAngle(1, 125);
  target_angles[2] = constrainAngle(2, 90);
  target_angles[3] = constrainAngle(3, 45);
}

void setup() {
  Serial.begin(115200);
  DDRC |= 0x0F;   // enable PC0 - PC3
  PORTC &= ~0x0F; // start with pins low

  // prime pulse widths so ISR has valid data on start
  for (int i = 0; i < 4; i++) {
    pulse_widths[i] = angleToPulse(current_angles[i]);
  }

  // timer 1 setup for 20ms period
  cli();
  TCCR1A = 0;
  TCCR1B = 0;
  TCNT1 = 0;
  OCR1A = 39999; // 20ms cycle
  OCR1B = 0;
  TCCR1B |= (1 << WGM12); // CTC mode
  TCCR1B |= (1 << CS11);  // prescaler 8
  TIMSK1 |= (1 << OCIE1A) | (1 << OCIE1B);
  sei();

  Serial.println("System Ready. Commands: Bddd, Sddd, Eddd, Gddd, Vddd, H");
}

// triggers every 20ms to restart the pulse sequence
ISR(TIMER1_COMPA_vect) {
  stage = 0;
}

// manages staggered pulses for 4 servos
ISR(TIMER1_COMPB_vect) {
  switch (stage) {
    case 0: PORTC |= (1 << BASE_PIN); OCR1B += pulse_widths[0]; break;
    case 1: PORTC &= ~(1 << BASE_PIN); OCR1B = SHLD_CHECKPOINT; break;
    case 2: PORTC |= (1 << SHLD_PIN); OCR1B += pulse_widths[1]; break;
    case 3: PORTC &= ~(1 << SHLD_PIN); OCR1B = ELBW_CHECKPOINT; break;
    case 4: PORTC |= (1 << ELBW_PIN); OCR1B += pulse_widths[2]; break;
    case 5: PORTC &= ~(1 << ELBW_PIN); OCR1B = GRIP_CHECKPOINT; break;
    case 6: PORTC |= (1 << GRIP_PIN); OCR1B += pulse_widths[3]; break;
    case 7: PORTC &= ~(1 << GRIP_PIN); OCR1B = BASE_CHECKPOINT; stage = -1; break;
  }
  stage++;
}

void loop() {
  // 1. handle serial commands
  if (Serial.available() > 0) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();
    
    if (cmd.length() > 0) {
      if (cmd == "H") {
        Serial.println("Homing...");
        homeAll();
      } 
      else if (cmd.length() != 4) {
        Serial.println("ERROR: Use 4-char format (e.g. B090)");
      } 
      else {
        char c = cmd.charAt(0);
        int val = parse3(&cmd.substring(1));

        if (val < 0) {
          Serial.println("ERROR: Invalid digits");
        } else {
          switch (c) {
            case 'B': target_angles[BASE_PIN] = constrainAngle(BASE_PIN, val); break;
            case 'S': target_angles[SHLD_PIN] = constrainAngle(SHLD_PIN, val); break;
            case 'E': target_angles[ELBW_PIN] = constrainAngle(ELBW_PIN, val); break;
            case 'G': target_angles[GRIP_PIN] = constrainAngle(GRIP_PIN, val); break;
            case 'V': 
              step_delay = constrain(val, 1, 999); 
              Serial.print("Velocity set to: "); Serial.println(step_delay);
              break;
            default: Serial.println("ERROR: Unknown Command"); break;
          }
        }
      }
    }
  }

  // 2. handle movement logic (velocity-controlled stepping)
  unsigned long current_time = millis();

  for (int i = 0; i < 4; i++) {
    // move 1 degree only if the step_delay (velocity) has passed
    if ((current_angles[i] != target_angles[i]) &&
        (current_time - last_move_time[i] >= (unsigned long)step_delay)) {
      
      if (current_angles[i] < target_angles[i]) {
        current_angles[i]++;
      } else {
        current_angles[i]--;
      }
 
      // update hardware pulse width atomically
      int new_pulse = angleToPulse(current_angles[i]);
      cli();
      pulse_widths[i] = new_pulse;
      sei();

      last_move_time[i] = current_time;
    }
  }
}