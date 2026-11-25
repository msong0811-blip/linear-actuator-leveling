#include <avr/interrupt.h>
#include <util/atomic.h>
#include <errno.h>
#include "TravelLimits.h"

// OFFLINE commissioning draft; no IMU or physical position feedback implemented.
// D2 -> PU+, D3 -> DR+. HIGH extends, LOW retracts per user observations.
// Coordinates are commanded pulses, not measured mechanical position.
// No automatic homing, startup motion, replay, or power/EN control.
const uint16_t PULSE_HZ = 160; // Nominal 1 mm/s, 6 motor rpm.
const uint16_t HEARTBEAT_MS = 500;
const uint16_t MAX_RUN_MS = 12000;
const uint8_t DIR_SETUP_MS = 10;
enum Phase : uint8_t { IDLE, SETTLE, STEPPING };
enum Event : uint8_t { NONE, COMPLETED, LIMIT_FAULT };
volatile Phase phase = IDLE;
volatile Event pendingEvent = NONE;
volatile int32_t position = 0;
volatile int32_t targetPosition = 0;
volatile int8_t directionStep = -1;
volatile uint16_t movePulses = 0;
volatile bool stepHigh = false;
volatile bool referenceValid = false;
bool referenceTakenThisBoot = false;
uint32_t lastHeartbeatMs = 0;
uint32_t startedMs = 0;
char command[40];
uint8_t commandLength = 0;
bool overflowed = false;

// Called with interrupts already disabled, including from ISR.
void disablePulseTimer() {
  TIMSK1 = 0;
  TCCR1B = 0;
  PORTD &= ~_BV(PD2);
  stepHigh = false;
  phase = IDLE;
}

ISR(TIMER1_COMPA_vect) {
  if (!referenceValid) {
    disablePulseTimer();
    pendingEvent = LIMIT_FAULT;
    return;
  }
  if (stepHigh) {
    PORTD &= ~_BV(PD2);
    stepHigh = false;
    if (position == targetPosition) {
      disablePulseTimer();
      pendingEvent = COMPLETED;
    }
    return;
  }
  const int32_t next = position + directionStep;
  if (position == targetPosition) {
    disablePulseTimer();
    pendingEvent = COMPLETED;
    return;
  }
  if (movePulses >= TravelLimits::MAX_MOVE_PULSES ||
      !TravelLimits::nextStepAllowed(position, next) ||
      (directionStep > 0 ? next > targetPosition : next < targetPosition)) {
    disablePulseTimer();
    referenceValid = false;
    pendingEvent = LIMIT_FAULT;
    return;
  }
  PORTD |= _BV(PD2);
  stepHigh = true;
  position = next;
  ++movePulses;
}

void reportStatus() {
  int32_t pos, target;
  uint16_t pulses;
  uint8_t state;
  bool valid;
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    pos = position; target = targetPosition; pulses = movePulses;
    state = phase; valid = referenceValid;
  }
  Serial.print(F("STATUS ref=")); Serial.print(valid ? 1 : 0);
  Serial.print(F(" phase=")); Serial.print(state);
  Serial.print(F(" pos=")); Serial.print(pos);
  Serial.print(F(" target=")); Serial.print(target);
  Serial.print(F(" pulses=")); Serial.print(pulses);
  Serial.println(F(" min=-12000 max=-800"));
}

void stopNow(const __FlashStringHelper *reason) {
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    // An interrupted loaded move needs a new physical reference, not replay.
    if (phase != IDLE) referenceValid = false;
    disablePulseTimer();
    pendingEvent = NONE;
  }
  Serial.print(F("STOPPED reason=")); Serial.println(reason);
}

bool parsePosition(const char *value, int32_t &parsed) {
  const char *scan = value;
  if (*scan == '-') ++scan;
  if (*scan == '\0') return false;
  for (; *scan; ++scan) if (*scan < '0' || *scan > '9') return false;
  errno = 0;
  char *end;
  const long result = strtol(value, &end, 10);
  if (errno == ERANGE || *end != '\0') return false;
  parsed = result;
  return true;
}

void moveTo(int32_t requested) {
  if (phase != IDLE || pendingEvent != NONE) {
    Serial.println(F("REFUSED busy"));
    return;
  }
  int32_t current;
  bool valid;
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) { current = position; valid = referenceValid; }
  const TravelLimits::Result check = TravelLimits::validate(valid, current, requested);
  if (check != TravelLimits::OK) {
    Serial.print(F("REFUSED target_check=")); Serial.println((uint8_t)check);
    return;
  }
  if (current == requested) {
    Serial.println(F("DONE unchanged"));
    return;
  }
  const int8_t direction = requested > current ? 1 : -1;
  digitalWrite(3, direction > 0 ? HIGH : LOW);
  startedMs = lastHeartbeatMs = millis();
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    movePulses = 0;
    stepHigh = false;
    targetPosition = requested;
    directionStep = direction;
    phase = SETTLE;
  }
  Serial.print(F("START target=")); Serial.print(requested);
  Serial.println(F(" hz=160"));
}

void handleCommand() {
  if (!strcmp(command, "STATUS")) reportStatus();
  else if (!strcmp(command, "KEEPALIVE")) lastHeartbeatMs = millis();
  else if (!strcmp(command, "STOP")) stopNow(F("command"));
  else if (!strcmp(command, "REFERENCE_MINUS60_CONFIRMED")) {
    // This assertion must follow an operator's physical maximum-extension check.
    // A host must never send it automatically after a restart or interruption.
    if (referenceTakenThisBoot || phase != IDLE) {
      Serial.println(F("REFUSED reference_already_taken"));
      return;
    }
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
      position = targetPosition = -9600;
      referenceValid = true;
    }
    referenceTakenThisBoot = true;
    Serial.println(F("REFERENCE pos=-9600 units=pulses"));
  } else if (!strncmp(command, "MOVE_PULSES ", 12)) {
    int32_t requested;
    if (!parsePosition(command + 12, requested)) Serial.println(F("REFUSED bad_number"));
    else moveTo(requested);
  } else Serial.println(F("REFUSED unknown_command"));
}


bool bootChecksPassed = false;
bool checkTravelLimits() {
  using namespace TravelLimits;
  if (validate(false,0,-800)!=NEED_REFERENCE || validate(true,0,-800)!=OK ||
      validate(true,0,-1600)!=OK || validate(true,0,-1601)!=TOO_FAR ||
      validate(true,0,-799)!=OUT_OF_RANGE || validate(true,0,-12001)!=OUT_OF_RANGE ||
      validate(true,-12001,-12000)!=BAD_POSITION || validate(true,1,-800)!=BAD_POSITION ||
      validate(true,-12000,-10400)!=OK || validate(true,-2400,-800)!=OK ||
      validate(true,-800,-800)!=OK || validate(true,-12000,-12000)!=OK) return false;
  for(int32_t p=-12000;p<=0;++p) {
    if(nextStepAllowed(p,p-1)!=(p>-12000)) return false;
    if(nextStepAllowed(p,p+1)!=(p<-800)) return false;
  }
  return true;
}

void setup() {
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    disablePulseTimer();
    TCCR1A = 0;
    referenceValid = false;
  }
  digitalWrite(2, LOW); digitalWrite(3, LOW);
  pinMode(2, OUTPUT); pinMode(3, OUTPUT);
  Serial.begin(115200);
  bootChecksPassed = checkTravelLimits();
  if (!bootChecksPassed) { Serial.println(F("FAULT boot_limits_test")); return; }
  Serial.println(F("LIMITS_TEST_PASS cases=24014"));
  Serial.println(F("ACTUATOR_LIMITED_READY v=3 ref=0 min=-12000 max=-800 max_move=1600"));
}

void loop() {
  if (!bootChecksPassed) return;
  const uint32_t now = millis();
  if (phase != IDLE && (uint32_t)(now - lastHeartbeatMs) >= HEARTBEAT_MS)
    stopNow(F("heartbeat_timeout"));
  if (phase != IDLE && (uint32_t)(now - startedMs) >= MAX_RUN_MS)
    stopNow(F("run_timeout"));
  Event event;
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) { event = pendingEvent; pendingEvent = NONE; }
  if (event == COMPLETED) { Serial.println(F("DONE")); reportStatus(); }
  else if (event == LIMIT_FAULT) { Serial.println(F("FAULT limit ref=0")); reportStatus(); }

  for (uint8_t n = 0; n < 32 && Serial.available(); ++n) {
    const char c = Serial.read();
    if (c == '\r') continue;
    if (c == '\n') {
      command[commandLength] = '\0';
      if (overflowed) Serial.println(F("REFUSED command_too_long"));
      else handleCommand();
      commandLength = 0; overflowed = false;
    } else if (commandLength < sizeof(command) - 1) command[commandLength++] = c;
    else overflowed = true;
  }

  if (phase == SETTLE && (uint32_t)(millis() - startedMs) >= DIR_SETUP_MS) {
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
      TIMSK1 = 0; TCCR1B = 0; TCCR1A = 0;
      TCNT1 = 0;
      OCR1A = (F_CPU / 8UL / 2UL / PULSE_HZ) - 1;
      TIFR1 = _BV(OCF1A) | _BV(OCF1B) | _BV(TOV1) | _BV(ICF1);
      phase = STEPPING;
      TIMSK1 = _BV(OCIE1A);
      TCCR1B = _BV(WGM12) | _BV(CS11);
    }
  }
}
