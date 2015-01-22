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

const unsigned int STEPS_PER_UNIT[3] =  {80, 80, 320};

#define HOMING_DELAY_US_PER_STEP 1000
const unsigned int MAX_FEEDRATE[3] = {25, 25, 20};

const unsigned int COORD_MAX[3] = {390, 390, 100};

#define INVERT_X false
#define INVERT_Y false
#define INVERT_Z false
