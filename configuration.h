/* Configuration for the router. Includes things like bed size, acceleration
 * settings, etc.
 *
 * All units are in millimeters. In the 16-bit fixed point position representation, 
 * each step is 0.01mm = 0.000393 inches (4 ten-thousandths). 
*/

#define BAUDRATE 9600

#define XMAX 375
#define YMAX 390
#define ZMAX 80
const unsigned int COORD_MAX[3] = {XMAX, YMAX, ZMAX};

const unsigned int STEPS_PER_UNIT[3] =  {80, 80, 320};

#define HOMING_STEP_DELAY 2000			// = 6.25 mm/sec for x and y axes, 1.5625 mm/sec for Z
#define SLOW_HOMING_STEP_DELAY 8000		// = 1.5625 mm/sec for x and y, 0.39 mm/sec for Z

#define INVERT_X true
#define INVERT_Y true
#define INVERT_Z true

const bool INVERT[3] = {INVERT_X, INVERT_Y, INVERT_Z};

// endstop policies: what to do if an endstop is hit during the middle of a move. Almost certainly you want to abort.
#define EP_ABORT 0
#define EP_IGNORE 1
#define EP_CLIP 2

#define ENDSTOP_POLICY EP_ABORT
