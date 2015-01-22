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

// note: all limit switch pins must be in the 8-13 group
#define XLIMIT  10
#define YLIMIT  11
#define ZLIMIT  12

//#define PIN_SPINDLE_SPEED
#define PIEZO 13


#endif
