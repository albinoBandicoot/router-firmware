#ifndef MOTION_H
#define MOTION_H

#include "pins.h"
#include "configuration.h"
#include "RingInput.h"

//typedef unsigned int pos_t
#define pos_t float
#define time_t unsigned long

#define STOPPED 0
#define MOVING 1
#define HOMING 2

#define DEBOUNCE_TIME 80  // milliseconds, for endstops
#define PULSE_WIDTH 20    // microseconds, for sending step pulses to the stepper drivers
#define DELAY_COMPENSATION 25  // microseconds, to account for the pulse delay and calculation time
#define MOTION_TIMER_FREQ (F_CPU >> 6)  // using 64x prescale

#define MIN(X) (X[0] < X[1] ? (X[1] < X[2] ? X[0] : (X[2] < X[0] ? X[2] : X[0])) : (X[1] < X[2] ? X[1] : X[2]))

void pulse (int);

class Motion {
  
  public:
    pos_t pos[3];
    pos_t target[3];
    char state;
    
    Motion (int);
    void begin();
    char checkEndstops (); 
    void runMove (pos_t xt, pos_t yt, pos_t zt, float feedrate);
    void homeAxes (char axes);
};

#endif
