#ifndef MOTION_H
#define MOTION_H

#include "pins.h"
#include "configuration.h"
#include "RingInput.h"

//typedef unsigned int pos_t
#define pos_t float
#define time_t unsigned long
#define ulong unsigned long
#define stepcount_t long

#define STOPPED 0
#define MOVING_LINEAR 1
#define MOVING_HELICAL 2
#define HOMING 3

#define MIN_RESUME_PRESS_LENGTH 1000  // milliseconds the resume button must be held down
#define DEBOUNCE_TIME 80  // milliseconds, for endstops
#define PULSE_WIDTH 8    // microseconds, for sending step pulses to the stepper drivers
#define MOTION_TIMER_FREQ (F_CPU >> 6)  // using 64x prescale ==> 4 usec/tick
#define MOTION_TIMER_COUNT_TARGET 50   // 200 usec between. reduce until it fails then back off
#define MOTION_TIMER_BLOCK_FREQ (MOTION_TIMER_FREQ / MOTION_TIMER_COUNT_TARGET)

/* The helical moves take more computation; we might need to use coarser resolution */
#define HELIX_MOTION_TIMER_COUNT_TARGET 50  // 200 usec (4 usec/tick)
#define HELIX_MOTION_TIMER_BLOCK_FREQ (MOTION_TIMER_FREQ / HELIX_MOTION_TIMER_COUNT_TARGET)

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
    stepcount_t steppos[3];  // exact number of steps moved. no funny business
    pos_t abspos[3];   // steppos, converted to mm and skew-corrected
    pos_t vpos[3];     // keeps track of where the router *should* be (this is to avoid accumulating discretization errors)
                       // after a move is set up, this is updated to the value it should have at the end of the move.
    pos_t worigin[3];  // position of the working origin
    float feedrate;    // current feedrate, mm/s
    float target_feedrate;
    float rotation;    // amount of rotation compensation; radians. This is the correction, not the deviation.
    float rot_matrix[2][2];  // matrix implementing the correction.
    
    char state;			// will be one of STOPPED, MOVING, or HOMING
    char abort;			// will be one of the ABORT_STATUS_xx above. 
    boolean resume;
    
    Motion (int);
    void begin();
    char checkEndstops (); 
    boolean checkResume ();
    boolean checkEdgefinder ();
    
    void skewcomp (pos_t &x, pos_t &y);
    void wocscomp (pos_t &x, pos_t &y, pos_t &z);  // working origin coordinate system compensation. does the wocs -> global direction.
    
    void setWorkingOrigin (pos_t dx, pos_t dy);  // set the working origin to current loc + offsets (offsets need to be derotated & deskewed)
    void setRotation (float theta);              // will populate the rot_matrix
    
    void edgefind (float max_travel, char axis);
    void edgefindMidpoint (float max_travel1, float max_travel2, float length, boolean liftz, char axis);
    void edgefind2 (float max_travel, float backoff, float length, boolean liftz, char axis);
    
    void startLinearMove (pos_t xt, pos_t yt, pos_t zt, float feedrate);
    void startHelicalMove (float r, float stheta, float etheta, float lead, float feedrate);
    
    void homeAxes (char axes);
    void tick ();
    void tick_linear ();
    void tick_helical ();
  
    void cleanup();
  
  private:   
    void homingMoveTowards(char axes, int del);
    void homingMoveAway(char axes, int del);
    void setDirections (boolean towards);
    void ef_mov (stepcount_t target, char axis);
};

#endif
