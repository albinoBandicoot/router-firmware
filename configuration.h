/* Configuration for the router. Includes things like bed size, acceleration
 * settings, etc.
 *
 * All units are in millimeters. In the 16-bit fixed point position representation, 
 * each step is 0.01mm = 0.000393 inches (4 ten-thousandths). 
*/

#define BAUDRATE 9600

#define XMAX 390
#define YMAX 390
#define ZMAX 100

const unsigned int STEPS_PER_UNIT[3] =  {160, 80, 320};

#define HOMING_STEP_DELAY 1000
#define SLOW_HOMING_STEP_DELAY 4000

const unsigned int COORD_MAX[3] = {390, 390, 100};

#define INVERT_X false
#define INVERT_Y false
#define INVERT_Z false

const bool INVERT[3] = {INVERT_X, INVERT_Y, INVERT_Z};

// endstop policies: what to do if an endstop is hit during the middle of a move. Almost certainly you want to abort.
#define EP_ABORT 0
#define EP_IGNORE 1
#define EP_CLIP 2

#define ENDSTOP_POLICY EP_ABORT
