#ifndef MOTION_H
#define MOTION_H

#include "pins.h"
#include "configuration.h"
#include "RingInput.h"

//typedef unsigned int pos_t
#define pos_t unsigned int

#define STOPPED 0
#define MOVING 1
#define HOMING 2

#define XAXIS_BIT 1
#define YAXIS_BIT 2
#define ZAXIS_BIT 4

#define DEBOUNCE_TIME 80
#define PULSE_WIDTH 20
#define HOMING_DELAY (1000000 / HOMING_FEEDRATE)

void pulse (int);

class Motion {
  
  public:
    pos_t pos[3];
    pos_t target[3];
    unsigned int delays[3];
    char state;
    
    Motion ();
    char checkEndstops (); 
    void runMove (pos_t xt, pos_t yt, pos_t zt, int feedrate);
    void homeAxes (char axes);
};

#endif
