/* Configuration for the router. Includes things like bed size, acceleration
 * settings, etc.
 *
 * All units are in millimeters. In the 16-bit fixed point position representation, 
 * each step is 0.01mm = 0.000393 inches (4 ten-thousandths). 
*/

#define BAUDRATE 9600

const unsigned int STEPS_PER_UNIT[3] =  {80, 80, 320};
const float UNITS_PER_STEP[3] = {0.0125f, 0.0125f, 0.003125f};  // keep these in sync with STEPS_PER_UNIT!!

#define XMAX 375
#define YMAX 390
#define ZMAX 80
const unsigned int COORD_MAX[3] = {XMAX * STEPS_PER_UNIT[0], YMAX*STEPS_PER_UNIT[1], ZMAX*STEPS_PER_UNIT[2]};

const int HOME_ENDSTOP_MASK[3] = {1, 2, 4};  // [ . . Zmax Ymax Xmax Zmin Ymin Xmin ]

#define HOMING_STEP_DELAY 2000			// = 6.25 mm/sec for x and y axes, 1.5625 mm/sec for Z
#define SLOW_HOMING_STEP_DELAY 8000		// = 1.5625 mm/sec for x and y, 0.39 mm/sec for Z
#define EDGEFIND_FEEDRATE 1.0f    // mm/s
#define EDGEFIND_TRAVEL_FEEDRATE 4.0f
#define ZLIFT_POS  1.0f          // this is the Z coordinate to which the Z axis will be lifted, eg. on edgefinding that requires a lift
#define ZLIFT_STEPPOS  ((long) (ZLIFT_POS * STEPS_PER_UNIT[2]))

/* The SKEW parameter is used to accommodate for any non-perpendicularity between the X and Y axes. If the axes
are perfectly aligned, SKEW should be 0. Positive skew means travelling in the +X direction causes the Y to become
more positive relative to a square. The SKEW value is the change in Y relative to square per unit change in X.

Measuring this accurately can be done as follows:

Put a pen in the router and have it draw an L shape with equal-length legs. Measure the diagonal. The skew is then:
< insert math here >
*/
#define SKEW 0

#define INVERT_X true
#define INVERT_Y true
#define INVERT_Z true

const bool INVERT[3] = {INVERT_X, INVERT_Y, INVERT_Z};

// endstop policies: what to do if an endstop is hit during the middle of a move. Almost certainly you want to abort.
#define EP_ABORT 0
#define EP_IGNORE 1
#define EP_CLIP 2

#define ENDSTOP_POLICY EP_ABORT
