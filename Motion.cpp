#include "Motion.h"

/* Constructor for motion manager */
Motion::Motion (int x) {
  int i;
  for (i = 0; i < 3; i++) {
    pos[i] = 0;
    target[i] = 0;
  }
  state = STOPPED;
  resume = false;
  abort = ABORT_STATUS_CLEAR;
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
  digitalWrite (RESUME_LED, LOW);
  digitalWrite (ENABLE, HIGH);
  
  pinMode (SPEAKER, OUTPUT);

  pinMode (XLIMIT, INPUT_PULLUP);
  pinMode (YLIMIT, INPUT_PULLUP);
  pinMode (ZLIMIT, INPUT_PULLUP);
  
  // set up endstop interrupts
  PCMSK0 = bit(XLIMIT - 8) | bit(YLIMIT - 8) | bit(ZLIMIT - 8); // set pin change interrupts mask to only include the limit switches
  PCIFR |= bit(PCIF0);  // clear flag by writing 1 to it
  PCICR |= bit(PCIE0);  // enable pin change interrupts on pins 8-13
  
  // set up resume button interrupt
  PCMSK1 = bit (PCINT10);  // A2
  PCIFR |= bit(PCIF1);
  PCICR |= bit(PCIE1);
}

// these variables keep track of the last times the endstops and resume button went LOW (pressed) 
// for debouncing purposes
volatile unsigned long lastActivation[3] = {0,0,0};
volatile unsigned long lastResume = 0;

/* Interrupt handler for endstops. This will get triggered whenever one of the endstop
* pins changes state. All this does is checks the pins, and if one reads LOW, it updates
* lastActivation with the current time. */
ISR (PCINT0_vect) {
  unsigned long time = millis();
  if (digitalRead (XLIMIT) == LOW) lastActivation[0] = time;
  if (digitalRead (YLIMIT) == LOW) lastActivation[1] = time;
  if (digitalRead (ZLIMIT) == LOW) lastActivation[2] = time;
}

/* Similar for the resume button */
ISR (PCINT1_vect) {
  if (digitalRead (RESUME) == LOW) lastResume = millis();
}

/* This will get the current status of the endstops. Since the switches bounce, this
* can't just return the current result of digitalRead(?LIMIT). If a switch does read
* LOW, it's definitely being pressed. Otherwise, check how long ago it last read LOW
* (we know this from the ISR above) and if it's been less than DEBOUNCE_TIME milliseconds,
* consider the switch still pressed.
*
* The result is encoded in the low 3 bits of a byte as .....ZYX
* a 1 bit indicates the switch is pressed.
*/
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

/* This is a similar routine for the resume button. Here the goal is slightly 
* different - we want the operator to hold down the resume button for a while
* (currently set at 1 second) to protect against unintentional button presses
* and any random noise on the signal line. Resuming the router unexpectedly 
* could pose a distinct safety hazard, as it is likely that things are not 
* properly set up and that body parts are in close proximity to the spindle.
*/
boolean Motion::checkResume () {
  if (digitalRead (RESUME) == HIGH) {
    resume = false;
  } else {
    resume = millis() - lastResume > MIN_RESUME_PRESS_LENGTH;
  }
  return resume;
}
  

/* Here's the stuff that actually controls the steppers and makes a move happen */

volatile unsigned long stepcounts[3];
volatile boolean towards_endstop[3];
volatile char active_axes = 0;

volatile float ticks_done = 0;
volatile float ticks_total = 0;
volatile ulong steps_done[3];
volatile float amt_done = 0;  // ranges from 0 to 1 indicating progress along the move.


/* This will set up the targets etc. for a move and start the timer going. The
* actual motion control has to happen through interrupts, because we can't afford
* to have the software block for the duration of the move (suspending serial communication)
*/
void Motion::startMove (pos_t xt, pos_t yt, pos_t zt, float feedrate){
  if (state == STOPPED) {	// check to make sure we're not already moving or homing
    state = MOVING;
    target[0] = xt;
    target[1] = yt;
    target[2] = zt;
    float diffs[3];
    int i;
	// first compute how far we have to go, and figure out which direction
	// we need to turn the steppers and set the direction signals appropriately
    for (i=0; i < 3; i++) {
      diffs[i] = target[i] - pos[i];
      towards_endstop[i] = diffs[i] < 0;
      digitalWrite (DIR[i], (diffs[i] < 0 ^ INVERT[i]) ? HIGH : LOW);
    }
    
	// compute the length in mm of the line along which we're moving
    float len = sqrt (diffs[0]*diffs[0] + diffs[1]*diffs[1] + diffs[2]*diffs[2]);
    float total_time = len / feedrate;	// time in seconds for the move
    ticks_total = ceil(total_time * MOTION_TIMER_BLOCK_FREQ);	// time in timer ticks for the move
	// here we figure out which axes are active (are moving) and compute how many steps 
	// we need to take on each axis.
    active_axes = 0;
    for (i=0; i < 3; i++) {
      if (diffs[i] != 0) {
        active_axes |= bit(i);
      }
      stepcounts[i] = (ulong) fabs (diffs[i] * STEPS_PER_UNIT[i]);
    }
    
	// now we set up the timer. 
    TCCR1A = 0;
    TCCR1B = bit(WGM12) | bit(CS10) | bit(CS11);  // CTC, 64x prescaling. Resolution = 4 usec, max time about 0.25 sec.
    OCR1A = MOTION_TIMER_COUNT_TARGET;
    TIMSK1 = bit (OCIE1A);
  }
}

/* While a move is active, the motion timer will go off every MOTION_TIMER_BLOCK_FREQ microseconds.
* We use the 16 bit timer #1; this is also used for other things (wait commands, and the speaker).
* The ISR can be found in the main file (firmware.ino). 
*
* We compute the proportion of the way done we are (based on a running count of how many ticks have
* happened). For each axis, check if it's active (to avoid unnecssary computation) and then see if
* amt_done * target # of steps > # of steps done so far. If so, we need to make a step. First we 
* check the endstops (or the maximum coordinates, depending on the travel direction), then we run
* the pulse if everything was OK. 
*
* As currently written, this will break if feeds are fast enough that multiple steps could occur
* inside the same tick interval. At current values, this means nothing faster than 5000 steps/sec.
* Shortening the timer delay would to a certain extent increase resolution (think of it like
* aliasing of lines drawn on a raster display - shortening the timer gives you smaller pixels),
* but eventually bad things will start to happen. If it goes too short, no time will be left for
* other things (communications), or worse, it won't even have enough time for itself to execute 
* before the next interrupt comes in. Since interrupts are disabled here, what would happen is 
* the feedrate would not be as high as expected, and since this timer interrupt has a higher
* priority than the serial communication interrupt, no comms could happen during the move.
*/
void Motion::tick () {  // called from ISR. Interrupts are disabled.
  ticks_done ++;
  amt_done = ticks_done / ticks_total;  // could just add 1/ticks_total each time, 
  // which could be precomputed to avoid doing an expensive division each tick, shortening the
  // time interrupts are disabled, but accumulated roundoff error could be significant.
  
  int i;
  char endst = checkEndstops();
  for (i=0; i < 3; i++) {
    if ((active_axes & (1 << i)) && floor(amt_done * stepcounts[i]) > steps_done[i]) {	// then we need to step this axis
      boolean es_ok;
      if (towards_endstop[i]) {	// if we're going toward the endstop, check whether it's triggered
        es_ok = (endst & (1 << i)) == 0;	
      } else {	// if we're going away, check whether our position would put us past the end of the axis travel
        es_ok = (pos[i]*(1-amt_done) + target[i]*amt_done) < COORD_MAX[i];
      }
      if (es_ok) {
        pulse (STEP[i]);
        steps_done[i]++;
      } else {
		// we really should never hit an endstop during a move. You should have a really good 
		// reason to set the endstop policy to anything other than EP_ABORT, which will shut everything down. 
        if (ENDSTOP_POLICY == EP_ABORT) {
          cleanup();
          abort = ABORT_STATUS_ABORT;
          return;
        }
      }
    }
  }

  if (ticks_done == ticks_total) {	// then we're done with the move - we need to shut down the timers and stuff
    cleanup();
  }
}

// shut down the timer and clean everything up for the next move
void Motion::cleanup () {
  TIMSK1 = 0;
  ticks_done = 0;
  int i;
  for (i = 0; i < 3; i++) {
  // note that here we don't just set the position to the target: if moves don't consist of 
  // an integer number of steps, the target position is not actually exactly where we ended up. 
  // This error would accumulate over the course of the job, and with carefully constructed move 
  // sequences could grow by up to 1 step per move. After only a hundred moves the error would be
  // over 1 millimeter, which is completely unacceptable. 
    pos[i] += steps_done[i] * (1.0f/STEPS_PER_UNIT[i]) * (towards_endstop[i] ? -1 : 1);
    steps_done[i] = 0;
  }
  state = STOPPED;
}


/* Now we have the code for homing. This has to be separate from the main motion code because
* we want to interpret the endstop signals differently, and also the single homing command would
* expand to multiple moves, which would get kind of messy.
*/
void Motion::homeAxes (char axes){
  state = HOMING;
  
  homingMoveAway    (axes, HOMING_STEP_DELAY);  // if any axes are already in contact w/ endstop, move them off
  homingMoveTowards (axes, HOMING_STEP_DELAY);  // move all the axes to the endstops
  homingMoveAway    (axes, HOMING_STEP_DELAY);  // back off
  homingMoveTowards (axes, SLOW_HOMING_STEP_DELAY);  // come back in slowly
  
  pos[0] = 0;
  pos[1] = 0;
  pos[2] = 0;
  state = STOPPED;
}

/* This moves all axes simultaneously, stopping them as they hit the endstops. Notice that 
* these methods don't use the timer - the delay is just embedded here (delayMicroseconds() is
* not timer-based like delay() is). Also, here interrupts are enabled, which means that at any
* point communications could usurp control for a little while. This is good. No motion will
* be happening - at most there might be a slight pause or irregularity. */
void Motion::homingMoveTowards (char axes, int step_delay) {
  setDirections (true);
  char e = checkEndstops();
  while ((e & axes) != axes) {
    if (~e & axes & 1) pulse (XSTEP);
    if (~e & axes & 2) pulse (YSTEP);
    if (~e & axes & 4) pulse (ZSTEP);
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

// sends a pulse to the stepper driver. The stepper data sheet says the pulse 
// needs to be at least 5 microseconds wide. 
void pulse (int pin) {
  digitalWrite (pin, HIGH);
  delayMicroseconds (PULSE_WIDTH);
  digitalWrite (pin, LOW);
}
