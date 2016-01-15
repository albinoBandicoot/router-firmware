/* Defines the input and output pins for the router */

#ifndef PINS_H
#define PINS_H

#define XSTEP  2
#define YSTEP  3
#define ZSTEP  4
#define XDIR   5
#define YDIR   6
#define ZDIR   7
#define ENABLE  A5

const int STEP[3] = {XSTEP, YSTEP, ZSTEP};
const int DIR[3]  = {XDIR, YDIR, ZDIR};


// note: all limit switch pins must be in the 8-13 group
#define XLIMIT_MIN  8
#define YLIMIT_MIN  9
#define ZLIMIT_MIN  10
#define XLIMIT_MAX  11
#define YLIMIT_MAX  12
#define ZLIMIT_MAX  13

// the resume pin must NOT be in the 8-13 group
#define RESUME A2  // if you change this: make sure to update the interrupt setup in Motion.cpp
#define RESUME_LED A1

#define SPINDLE_SPEED A3  // for reading; can't set speed through this.
#define SPEAKER A4

//#define SPINDLE_POWER -1
#define EDGEFINDER A0


#endif
