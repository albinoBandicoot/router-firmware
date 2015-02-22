#ifndef MOTION_H
#define MOTION_H

#include "pins.h"
#include "configuration.h"
#include "RingInput.h"

//typedef unsigned int pos_t
#define pos_t float
#define time_t unsigned long
#define ulong unsigned long

#define STOPPED 0
#define MOVING 1
#define HOMING 2

#define MIN_RESUME_PRESS_LENGTH 1000  // milliseconds the resume button must be held down
#define DEBOUNCE_TIME 80  // milliseconds, for endstops
#define PULSE_WIDTH 8    // microseconds, for sending step pulses to the stepper drivers
//#define DELAY_COMPENSATION 25  // microseconds, to account for the pulse delay and calculation time
#define MOTION_TIMER_FREQ (F_CPU >> 6)  // using 64x prescale ==> 4 usec/tick
#define MOTION_TIMER_COUNT_TARGET 50   // 200 usec between. reduce until it fails then back off
#define MOTION_TIMER_BLOCK_FREQ (MOTION_TIMER_FREQ / MOTION_TIMER_COUNT_TARGET)

#define MIN(X) (X[0] < X[1] ? (X[1] < X[2] ? X[0] : (X[2] < X[0] ? X[2] : X[0])) : (X[1] < X[2] ? X[1] : X[2]))

#define WAIT_MODE 0
#define MOTION_MODE 1
#define BEEP_MODE 2

#define ABORT_STATUS_CLEAR 0
#define ABORT_STATUS_ABORT 1
#define ABORT_STATUS_RESET 2  // waiting for the resume button to be pushed.
void pulse (int);

class Motion {
  
  public:
    pos_t pos[3];	// current position of the router, in mm
    pos_t target[3];	// target position of currently running move
    char state;			// will be one of STOPPED, MOVING, or HOMING
    char abort;			// will be one of the ABORT_STATUS_xx above. 
    boolean resume;
    
    Motion (int);
    void begin();
    char checkEndstops (); 
    boolean checkResume ();
    void startMove (pos_t xt, pos_t yt, pos_t zt, float feedrate);
    void homeAxes (char axes);
    void tick ();
  
    void cleanup();
  
  private:   
    void homingMoveTowards(char axes, int del);
    void homingMoveAway(char axes, int del);
    void setDirections (boolean towards);
};

#endif
