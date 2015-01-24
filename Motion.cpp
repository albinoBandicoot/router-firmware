#include "Motion.h"


volatile unsigned long lastActivation[3] = {0,0,0};


/* ISR for endstops */
ISR (PCINT0_vect) {
  unsigned long time = millis();
  if (digitalRead (XLIMIT) == LOW) lastActivation[0] = time;
  if (digitalRead (YLIMIT) == LOW) lastActivation[1] = time;
  if (digitalRead (ZLIMIT) == LOW) lastActivation[2] = time;
}

Motion::Motion (int x) {
  int i;
  for (i = 0; i < 3; i++) {
    pos[i] = 0;
    target[i] = 0;
  }
  state = STOPPED;
}

/* Set up pins for driving the axes and interrupt mask for the endstops */
void Motion::begin () {
  pinMode (XSTEP, OUTPUT);
  pinMode (YSTEP, OUTPUT);
  pinMode (ZSTEP, OUTPUT);
  pinMode (XDIR,  OUTPUT);
  pinMode (YDIR,  OUTPUT);
  pinMode (ZDIR,  OUTPUT);
  pinMode (ENABLE, OUTPUT);
  
  pinMode (RESUME, INPUT_PULLUP);
  pinMode (RESUME_LED, OUTPUT);

  pinMode (XLIMIT, INPUT_PULLUP);
  pinMode (YLIMIT, INPUT_PULLUP);
  pinMode (ZLIMIT, INPUT_PULLUP);
  
  PCMSK0 = bit(XLIMIT - 8) | bit(YLIMIT - 8) | bit(ZLIMIT - 8);  // set pin change interrupts mask to only include the limit switches
  PCIFR |= bit(PCIF0);  // clear flag by writing 1 to it
  PCICR |= bit(PCIE0);  // enable pin change interrupts on pins 8-13
}

char Motion::checkEndstops () {
  char res = 0;
  if (digitalRead (XLIMIT) == LOW) res |= 1;
  if (digitalRead (YLIMIT) == LOW) res |= 2;
  if (digitalRead (ZLIMIT) == LOW) res |= 4;
  unsigned long time = millis();
  int i;
  for (i=0; i < 3; i++) {
    if (time - lastActivation[i] < DEBOUNCE_TIME) res |= (1 << i);
  }
  return res;
}

volatile unsigned long stepcounts[3];
volatile boolean towards_endstop[3];
volatile char active_axes = 0;

volatile float ticks_done = 0;
volatile float ticks_total = 0;
volatile ulong steps_done[3];
volatile float amt_done = 0;  // ranges from 0 to 1 indicating progress along the move.


void Motion::startMove (pos_t xt, pos_t yt, pos_t zt, float feedrate){
  if (state == STOPPED) {
    state = MOVING;
    target[0] = xt;
    target[1] = yt;
    target[2] = zt;
    float diffs[3];
    int i;
    for (i=0; i < 3; i++) {
      diffs[i] = target[i] - pos[i];
      digitalWrite (DIR[i], (diffs[i] < 0 ^ INVERT[i]) ? HIGH : LOW);
    }
    
    // now compute delays, set up the timer, and initialize steptimes
    // first we need to determine feedrates along each axis. 
    float len = sqrt (diffs[0]*diffs[0] + diffs[1]*diffs[1] + diffs[2]*diffs[2]);
    float total_time = len / feedrate;
    ticks_total = ceil(total_time * MOTION_TIMER_BLOCK_FREQ);
    active_axes = 0;
    for (i=0; i < 3; i++) {
      if (diffs[i] != 0) {
        active_axes |= bit(i);
      }
      stepcounts[i] = (ulong) fabs (diffs[i] * STEPS_PER_UNIT[i]);
    }
    
    TCCR1A = 0;
    TCCR1B = bit(WGM12) | bit(CS10) | bit(CS11);  // CTC, 64x prescaling. Resolution = 4 usec, max time about 0.25 sec.
    OCR1A = MOTION_TIMER_COUNT_TARGET;
    TIMSK1 = bit (OCIE1A);
  }
}

// as currently written, this will break if feeds are fast enough that multiple steps could occur
// inside the same tick interval. At current values, this means nothing faster than 2500 steps/sec.
void Motion::tick () {  // called from ISR. Interrupts are disabled.
  ticks_done ++;
  amt_done = ticks_done / ticks_total;  // could just add 1/ticks_total each time, but roundoff...
  
  int i;
  char endst = checkEndstops();
  for (i=0; i < 3; i++) {
    if ((active_axes & (1 << i)) && floor(amt_done * stepcounts[i]) > steps_done[i]) {
      boolean es_ok;
      if (towards_endstop[i]) {
        es_ok = (endst & (1 << i)) == 0;
      } else {
        es_ok = (pos[i]*(1-amt_done) + target[i]*amt_done) < COORD_MAX[i];
      }
      if (es_ok) {
        pulse (STEP[i]);
        steps_done[i]++;
      } else {
        if (ENDSTOP_POLICY == EP_ABORT) {
          cleanup();
          // TODO: stop things and send a message about what happened.
        }
      }
    }
  }

  if (ticks_done == ticks_total) {
    cleanup();
  }
}

void Motion::cleanup () {
    // shut down the timer and clean everything up for the next move
  ticks_done = 0;
  int i;
  for (i = 0; i < 3; i++) {
    pos[i] = target[i];
    steps_done[i] = 0;
  }
  state = STOPPED;
  TIMSK1 = 0;
}

// FIXME: if axes are already homed, move them away until the switch turns off, then home them as usual.
void Motion::homeAxes (char axes){
  state = HOMING;
  
  homingMoveAway    (axes, HOMING_STEP_DELAY);  // if any axes are already in contact w/ endstop, move them off
  homingMoveTowards (axes, HOMING_STEP_DELAY);  // move all the axes to the endstops
  homingMoveAway    (axes, HOMING_STEP_DELAY);  // back off
  homingMoveTowards (axes, SLOW_HOMING_STEP_DELAY);  // come back in slowly
  
  pos[0] = 0;
  pos[1] = 0;
  pos[2] = 0;
}

void Motion::homingMoveTowards (char axes, int step_delay) {
  setDirections (true);
  char e = checkEndstops();
  while ((e & axes) != axes) {
    if (~(e & axes & 1)) pulse (XSTEP);
    if (~(e & axes & 2)) pulse (YSTEP);
    if (~(e & axes & 4)) pulse (ZSTEP);
    delayMicroseconds (step_delay);
    e = checkEndstops();
  }
}

void Motion::homingMoveAway (char axes, int step_delay) {
  setDirections (false);
  char e = checkEndstops();
  while ((e | ~axes) != ~axes) {
    if (e & axes & 1) pulse (XSTEP);
    if (e & axes & 2) pulse (YSTEP);
    if (e & axes & 4) pulse (ZSTEP);
    delayMicroseconds (step_delay);
    e = checkEndstops();
  }
}

void Motion::setDirections (boolean towards_endstops) {
  int i;
  for (i=0; i < 3; i++) {
    digitalWrite (DIR[i], (INVERT[i] ^ towards_endstops) ? HIGH : LOW);
  }
}

void pulse (int pin) {
  digitalWrite (pin, HIGH);
  delayMicroseconds (PULSE_WIDTH);
  digitalWrite (pin, LOW);
}
