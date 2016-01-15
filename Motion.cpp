#include "Motion.h"

/* Constructor for motion manager */
Motion::Motion (int x) {
  int i;
  for (i = 0; i < 3; i++) {
    steppos[i] = 0;
    abspos[i] = 0;
    vpos[i] = 0;
    worigin[i] = 0;
  }
  setRotation (0);
  feedrate = 2.0f;  // a sane default just in case
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
//  pinMode (SPINDLE_POWER, OUTPUT);
  pinMode (SPINDLE_SPEED, INPUT);
  
//  digitalWrite (SPINDLE_POWER, LOW);
  
  pinMode (RESUME, INPUT_PULLUP);
  pinMode (RESUME_LED, OUTPUT);
  digitalWrite (RESUME_LED, LOW);
  digitalWrite (ENABLE, HIGH);
  
  pinMode (SPEAKER, OUTPUT);

  pinMode (XLIMIT_MIN, INPUT_PULLUP);
  pinMode (YLIMIT_MIN, INPUT_PULLUP);
  pinMode (ZLIMIT_MIN, INPUT_PULLUP);
  pinMode (XLIMIT_MAX, INPUT_PULLUP);
  pinMode (YLIMIT_MAX, INPUT_PULLUP);
  pinMode (ZLIMIT_MAX, INPUT_PULLUP);
  
  pinMode (EDGEFINDER, INPUT_PULLUP);
  
  // set up endstop interrupts
  PCMSK0 = 0x3f;  // 00111111   --- all pins in group are now used
  //PCMSK0 = bit(XLIMIT - 8) | bit(YLIMIT - 8) | bit(ZLIMIT - 8); // set pin change interrupts mask to only include the limit switches
  PCIFR |= bit(PCIF0);  // clear flag by writing 1 to it
  PCICR |= bit(PCIE0);  // enable pin change interrupts on pins 8-13
  
  // set up resume button interrupt
  PCMSK1 = bit (PCINT10);  // A2
  PCIFR |= bit(PCIF1);
  PCICR |= bit(PCIE1);
}

// these variables keep track of the last times the endstops and resume button went LOW (pressed) 
// for debouncing purposes
volatile unsigned long lastActivation[6] = {0,0,0,0,0,0};
volatile unsigned long lastResume = 0;

/* Interrupt handler for endstops. This will get triggered whenever one of the endstop
* pins changes state. All this does is checks the pins, and if one reads LOW, it updates
* lastActivation with the current time. */
ISR (PCINT0_vect) {
  unsigned long time = millis();
  if (digitalRead (XLIMIT_MIN) == LOW) lastActivation[0] = time;
  if (digitalRead (YLIMIT_MIN) == LOW) lastActivation[1] = time;
  if (digitalRead (ZLIMIT_MIN) == LOW) lastActivation[2] = time;
  if (digitalRead (XLIMIT_MAX) == LOW) lastActivation[3] = time;
  if (digitalRead (YLIMIT_MAX) == LOW) lastActivation[4] = time;
  if (digitalRead (ZLIMIT_MAX) == LOW) lastActivation[5] = time;
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
* The result is encoded in the low 6 bits of a byte as ..ZYXzyx
* where the low 3 bits are the MIN endstops, and the next 3 bits are the MAX endstops.
* a 1 bit indicates the switch is pressed.
*/
char Motion::checkEndstops () {
  char res = 0;
  unsigned long time = millis();
  int i;
  for (i=0; i < 6; i++) {
    if (digitalRead (XLIMIT_MIN + i) == LOW || time - lastActivation[i] < DEBOUNCE_TIME) res |= (1 << i);
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

boolean Motion::checkEdgefinder () {
  return true;
}
  
void Motion::skewcomp (pos_t &x, pos_t &y) {
  y -= x * SKEW;  // 50% chance the sign is wrong
}

void Motion::wocscomp (pos_t &x, pos_t &y, pos_t &z) {
  pos_t xsave = x;
  x = rot_matrix[0][0]*x + rot_matrix[0][1]*y;
  y = rot_matrix[1][0]*xsave + rot_matrix[1][1]*y;
  x -= worigin[0];
  y -= worigin[1];
  z -= worigin[2];
}

void Motion::setWorkingOrigin (float dx, float dy) {
  // TODO: implement me!
}

void Motion::setRotation (float theta) {
  rotation = theta;
  rot_matrix[0][0] = cos(theta);
  rot_matrix[0][1] = -sin(theta);
  rot_matrix[1][0] = sin(theta);
  rot_matrix[1][1] = cos(theta);
}

/* Here's the stuff that actually controls the steppers and makes a move happen */

volatile stepcount_t relsteppos[3];  // holds the relative # of steps from the beginning of the move
volatile stepcount_t linearmove_deltas[3];
volatile boolean towards_endstop[3];
volatile char active_axes = 0;

/* Helix params */
volatile float stheta, dtheta, radius, pitch;
volatile stepcount_t cpoint[3];
volatile char prevdirs[3];

volatile float ticks_done = 0;
volatile float ticks_total = 0;
volatile float time_done = 0;  // ranges from 0 to 1 indicating time progress along the move.
volatile float amt_done = 0;   // ranges from 0 to 1 indicating position progress


/* This will set up the targets etc. for a linear move and start the timer going. The
* actual motion control has to happen through interrupts, because we can't afford
* to have the software block for the duration of the move (suspending serial communication)
*
* The coordinates are specified in fully transformed space, but we need to compute actual 
* step differences. First we de-rotate, then translate so the origin is at the true origin
* rather than the working origin, then de-skew and convert to steps.
*/
void Motion::startLinearMove (pos_t xt, pos_t yt, pos_t zt, float tfeed){
  if (state == STOPPED) {	// check to make sure we're not already moving or homing
    state = MOVING_LINEAR;
    target_feedrate = tfeed;
    
    wocscomp (xt, yt, zt);
    skewcomp (xt, yt);
    
    pos_t diffs[3];
    diffs[0] = xt - steppos[0] * UNITS_PER_STEP[0];
    diffs[1] = yt - steppos[1] * UNITS_PER_STEP[1];
    diffs[2] = zt = steppos[2] * UNITS_PER_STEP[2];
    
    stepcount_t target[3];
    target[0] = (stepcount_t) (xt * STEPS_PER_UNIT[0]);
    target[1] = (stepcount_t) (yt * STEPS_PER_UNIT[1]);
    target[2] = (stepcount_t) (zt * STEPS_PER_UNIT[2]);
    
    int i;
	// first compute how far we have to go, and figure out which direction
	// we need to turn the steppers and set the direction signals appropriately
    for (i=0; i < 3; i++) {
      towards_endstop[i] = diffs[i] < 0;
      digitalWrite (DIR[i], (diffs[i] < 0 ^ INVERT[i]) ? HIGH : LOW);
    }
    
	// compute the length in mm of the line along which we're moving
    float len = sqrt (diffs[0]*diffs[0] + diffs[1]*diffs[1] + diffs[2]*diffs[2]);  // this is in mm
    float total_time = len / ((feedrate+target_feedrate) / 2);	// time in seconds for the move
    ticks_total = ceil(total_time * MOTION_TIMER_BLOCK_FREQ);	// time in timer ticks for the move
	// here we figure out which axes are active (are moving) and compute how many steps 
	// we need to take on each axis.
    active_axes = 0;
    for (i=0; i < 3; i++) {
      if (diffs[i] != 0) {
        active_axes |= bit(i);
      }
      linearmove_deltas[i] = target[i] - steppos[i];
    }
    
	// now we set up the timer. 
    TCCR1A = 0;
    TCCR1B = bit(WGM12) | bit(CS10) | bit(CS11);  // CTC, 64x prescaling. Resolution = 4 usec, max time about 0.25 sec.
    OCR1A = MOTION_TIMER_COUNT_TARGET;
    TIMSK1 = bit (OCIE1A);
  }
}

/* NOTE: as it currently stands, helical moves are NOT SKEW-CORRECTED */
void Motion::startHelicalMove (float r, float sth, float dth, float lead, float tfeed) {
  if (state == STOPPED) {
    state = MOVING_HELICAL;
    target_feedrate = tfeed;
    active_axes = (lead == 0) ? 3 : 7;  // Z only active if lead is nonzero; X and Y always active
   
    sth += rotation;  // compensate for rotation. Is the sign correct here?
   
    // set the global helix parameters for access in tick_helical
    pitch = lead * 0.15915494f;  // we want delta Z per radian, not per revolution
    stheta = sth;
    dtheta = dth;
    radius = r;
    
    /* The arc length of a helix is given by T * sqrt(r^2 + b^2), where T is the # of radians of rotation,
     * r is the radius, and b = lead/2pi. This corresponds to the parameterization t --> (r*cos(t), r*sin(t), bt) 
     * Unfortunately, this does not take into acount any differences introduced by skew compensation. */
    
    float len = abs (dtheta * sqrt(r*r + pitch*pitch));
    float total_time = len / ((feedrate + target_feedrate) / 2);
    ticks_total = ceil (total_time * HELIX_MOTION_TIMER_BLOCK_FREQ);
    
    cpoint[0] = abspos[0] - r * cos(stheta);
    cpoint[1] = abspos[1] - r * sin(stheta);
    cpoint[2] = abspos[2];  // current Z
    
    prevdirs[0] = 0;
    prevdirs[1] = 0;
    prevdirs[2] = 0;
    
    /* Timer setup */
    TCCR1A = 0;
    TCCR1B = bit(WGM12) | bit(CS10) | bit(CS11);  // CTC, 64x prescaling. Resolution = 4 usec, max time about 0.25 sec.
    OCR1A = HELIX_MOTION_TIMER_COUNT_TARGET;
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
* priority than the serial communication interrupt, no comms could happen during the move. (or
* perhaps the timer would get confused and shut down after one cycle, as it did with the music
* thing)
*/
void Motion::tick_linear () {  // called from ISR. Interrupts are disabled.
  ticks_done ++;
  time_done = ticks_done / ticks_total;  // could just add 1/ticks_total each time, 
  // which could be precomputed to avoid doing an expensive division each tick, shortening the
  // time interrupts are disabled, but accumulated roundoff error could be significant.
  amt_done = (feedrate*(time_done-1) + target_feedrate) / (feedrate + target_feedrate);  // we may be accelerating, so compute the proportion of the move we should have completed by this time
  // now compute the 
  
  int i;
  char endst = checkEndstops();
  for (i=0; i < 3; i++) {
    if ((active_axes & (1 << i)) && floor(amt_done * linearmove_deltas[i]) > relsteppos[i]) {	// then we need to step this axis
      boolean es_ok = (endst & (9 << i)) == 0;  // mask is 1001, which picks out the MIN and MAX for this axis
      /*
      boolean es_ok;
      if (towards_endstop[i]) {	// if we're going toward the endstop, check whether it's triggered
        es_ok = (endst & (1 << i)) == 0;	
      } else {	// if we're going away, check whether our position would put us past the end of the axis travel
        es_ok = steppos[i] + relsteppos[i] < COORD_MAX[i]; 
      }
      */
      if (es_ok) {
        pulse (STEP[i]);
        relsteppos[i] += (linearmove_deltas[i] > 0) ? 1 : -1;
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

void Motion::tick_helical () {
  ticks_done ++;
  time_done = ticks_done / ticks_total;
  amt_done =  (feedrate*(time_done-1) + target_feedrate) / (feedrate + target_feedrate);
  
  float theta = stheta + amt_done * dtheta;
  
  stepcount_t tar[3];
  tar[0] = cpoint[0] + (stepcount_t) (radius * cos(theta) * STEPS_PER_UNIT[0]);
  tar[1] = cpoint[1] + (stepcount_t) (radius * sin(theta) * STEPS_PER_UNIT[1]);
  tar[2] = (stepcount_t) (cpoint[2] + (pitch * amt_done * dtheta * STEPS_PER_UNIT[2]));

  int i;
  char endst = checkEndstops();
  for (i=0; i < 3; i++) {
    if (active_axes & (1 << i)) {
      if ((endst & (9 << i)) == 0) {
        stepcount_t delta = tar[i] - (steppos[i] + relsteppos[i]);
        if (delta != 0) {
          if (delta != prevdirs[i]) {
            digitalWrite (DIR[i], (delta > 0) ? HIGH: LOW);  // CHECK SIGN
            prevdirs[i] = delta > 0 ? 1 : -1;
          }
          pulse (STEP[i]);
          relsteppos[i] += delta;
        }
      }
    }
  }
}

void Motion::tick () {
  if (state == MOVING_LINEAR) {
    tick_linear();
  } else if (state == MOVING_HELICAL) {
    tick_helical();
  }
}

// shut down the timer and clean everything up for the next move
void Motion::cleanup () {
  TIMSK1 = 0;
  ticks_done = 0;
  time_done = 0;
  amt_done = 0;
  feedrate = target_feedrate;
  int i;
  for (i = 0; i < 3; i++) {
    steppos[i] += relsteppos[i]; 
    relsteppos[i] = 0;
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
  
  int i = 0;
  for (i=0; i < 3; i++) {
    if (axes & (1 << i)) {
      steppos[i] = 0;
      vpos[i] = 0;
      abspos[i] = 0;
    }
  }
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
    if (~e & axes & HOME_ENDSTOP_MASK[0]) pulse (XSTEP);
    if (~e & axes & HOME_ENDSTOP_MASK[1]) pulse (YSTEP);
    if (~e & axes & HOME_ENDSTOP_MASK[2]) pulse (ZSTEP);
    delayMicroseconds (step_delay);
    e = checkEndstops();
  }
}

void Motion::homingMoveAway (char axes, int step_delay) {
  setDirections (false);
  char e = checkEndstops();
  while ((e | ~axes) != ~axes) {
    if (e & axes & HOME_ENDSTOP_MASK[0]) pulse (XSTEP);
    if (e & axes & HOME_ENDSTOP_MASK[1]) pulse (YSTEP);
    if (e & axes & HOME_ENDSTOP_MASK[2]) pulse (ZSTEP);
    delayMicroseconds (step_delay);
    e = checkEndstops();
  }
}

/* Edgefinding */
// TODO: 
// - also need to update abspos and vpos after edgefinding

void Motion::ef_mov (stepcount_t target, char axis) {
  stepcount_t delta = target - steppos[axis];
  //digitalWrite (DIR[axis], ((delta > 0) ^ INVERT[axis]) ? HIGH : LOW);  // CHECK SIGN
  int delay_us = (int) (1000000.0f / (EDGEFIND_TRAVEL_FEEDRATE * STEPS_PER_UNIT[axis]));
  while (steppos[axis] != target) {
    if (checkEndstops()) {
      cleanup();
      abort = ABORT_STATUS_ABORT;
      return;
    }
    pulse (STEP[axis]);
    delayMicroseconds (delay_us);
    steppos[axis] += (delta > 0) ? 1 : -1;
  }
}

void Motion::edgefind (float max_travel, char axis) {
  digitalWrite (DIR[axis], (max_travel > 0 ^ INVERT[axis]) ? HIGH : LOW);  // CHECK SIGN
  char e = checkEdgefinder ();
  int delay_us = (int) (1000000.0f / (EDGEFIND_FEEDRATE * STEPS_PER_UNIT[axis]));
  stepcount_t max_travel_steps = STEPS_PER_UNIT[axis] * abs(max_travel);
  stepcount_t dist_travelled = 0;
  while (!e && dist_travelled < max_travel_steps) {
    pulse (STEP[axis]);
    delayMicroseconds (delay_us);
    e = checkEdgefinder ();
    dist_travelled ++;
  }
}

void Motion::edgefindMidpoint (float max_travel1, float max_travel2, float length, boolean zlift, char axis) {
  edgefind (max_travel1, axis);
  stepcount_t start_pos = steppos[axis];
  
  stepcount_t zpos = steppos[2];
  if (zlift) ef_mov (ZLIFT_STEPPOS, 2);
  ef_mov (steppos[axis] + (stepcount_t) (length * STEPS_PER_UNIT[axis]), axis);
  if (zlift) ef_mov (zpos, 2);
  
  edgefind (max_travel2, axis);
  stepcount_t end_pos = steppos[axis];
  
  if (zlift) ef_mov (ZLIFT_STEPPOS, 2);
  ef_mov ((start_pos + end_pos)/2, axis);
}

void Motion::edgefind2 (float max_travel, float backoff, float length, boolean zlift, char long_axis) {
  char short_axis = (long_axis + 1) % 2;
  edgefind (max_travel, short_axis);
  ef_mov ( (stepcount_t) ((backoff * STEPS_PER_UNIT[short_axis]) * (max_travel > 0 ? -1 : 1)), short_axis);
  stepcount_t pos1 = steppos[short_axis];
  
  stepcount_t zpos = steppos[2];
  if (zlift) ef_mov (ZLIFT_STEPPOS, 2);
  ef_mov (steppos[long_axis] + (stepcount_t) (length * STEPS_PER_UNIT[long_axis]), long_axis);
  if (zlift) ef_mov (zpos, 2);
  
  edgefind (max_travel, short_axis);
  stepcount_t pos2 = steppos[short_axis];
  float sdiff = (pos2 - pos1) * UNITS_PER_STEP[short_axis];
  
  // do we really want to set the rotation here?
  setRotation (-tan(sdiff/length));  // CHECK SIGN
  
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
