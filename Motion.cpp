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
}

/* Set up pins for driving the axes and interrupt mask for the endstops */
void Motion::begin () {
  pinMode (XSTEP, OUTPUT);
  pinMode (YSTEP, OUTPUT);
  pinMode (ZSTEP, OUTPUT);
  pinMode (XDIR,  OUTPUT);
  pinMode (YDIR,  OUTPUT);
  pinMode (ZDIR,  OUTPUT);

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

volatile unsigned int delays[3];
volatile unsigned int stepcounts[3];
volatile char active_axes = 0;

// units are in steps, or steps/second. 
void Motion::runMove (pos_t xt, pos_t yt, pos_t zt, float feedrate){
  if (state == STOPPED) {
    state = MOVING;
    target[0] = xt;
    target[1] = yt;
    target[2] = zt;
    float diffs[3];
    diffs[0] = target[0] - pos[0];
    diffs[1] = target[1] - pos[1];
    diffs[2] = target[2] - pos[2];
    digitalWrite (XDIR, (diffs[0] < 0 ^ INVERT_X) ? HIGH: LOW);
    digitalWrite (YDIR, (diffs[1] < 0 ^ INVERT_Y) ? HIGH: LOW);
    digitalWrite (ZDIR, (diffs[2] < 0 ^ INVERT_Z) ? HIGH: LOW);
    
    // now compute delays, set up the timer, and initialize steptimes
    // first we need to determine feedrates along each axis. 
    float len = sqrt (diffs[0]*diffs[0] + diffs[1]*diffs[1] + diffs[2]*diffs[2]);
    int i;
    for (i=0; i < 3; i++) {
      if (diffs[i] != 0) {  // avoid divide by 0. We will have a 0 stepcount for stationary axes, so the value in delays won't matter
        float feed_sps = STEPS_PER_UNIT[i] * feedrate * diffs[i] / len;  // steps per second
        delays[i] = (int) (MOTION_TIMER_FREQ/feed_sps) - DELAY_COMPENSATION;  // determine delay in timer cycles
        active_axes |= bit(i);
      }
      stepcounts[i] = (int) fabs (diffs[i] * STEPS_PER_UNIT[i]);
    }
    
    
    TCCR1A = 0;
    TCCR1B = bit(WGM12) | bit(CS10) | bit(CS11);  // CTC, 64x prescaling. Resolution = 4 usec, max time about 0.25 sec.
    OCR1A = MIN(delays);
    
  }
}

ISR (TIMER1_COMPA_vect) {
  
}
// FIXME: if axes are already homed, move them away until the switch turns off, then home them as usual.
void Motion::homeAxes (char axes){
  state = HOMING;
  target[0] = 0;
  target[1] = 0;
  target[2] = 0;
  digitalWrite (XDIR, INVERT_X ? HIGH : LOW);
  digitalWrite (YDIR, INVERT_Y ? HIGH : LOW);
  digitalWrite (ZDIR, INVERT_Z ? HIGH : LOW);
  char e = checkEndstops();
  while (e != 7) {  // while some endstop is not pressed:
    if ( ~(e & 1) ) pulse (XSTEP);
    if ( ~(e & 2) ) pulse (YSTEP);
    if ( ~(e & 4) ) pulse (ZSTEP);
    delayMicroseconds (HOMING_DELAY_US_PER_STEP);
    e = checkEndstops();
  }  
}

void pulse (int pin) {
  digitalWrite (pin, HIGH);
  delayMicroseconds (PULSE_WIDTH);
  digitalWrite (pin, LOW);
}
