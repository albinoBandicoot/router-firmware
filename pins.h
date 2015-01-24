/* Defines the input and output pins for the router */

#ifndef PINS_H
#define PINS_H

#define XSTEP  3
#define YSTEP  4
#define ZSTEP  5
#define XDIR   6
#define YDIR   7
#define ZDIR   8
#define ENABLE  2

const int STEP[3] = {XSTEP, YSTEP, ZSTEP};
const int DIR[3]  = {XDIR, YDIR, ZDIR};

// note: all limit switch pins must be in the 8-13 group
#define XLIMIT  10
#define YLIMIT  11
#define ZLIMIT  12

#define RESUME 9
#define RESUME_LED 13

#define SPINDLE_SPEED A0  // for reading; can't set speed through this.
#define SPEAKER A1


#endif
