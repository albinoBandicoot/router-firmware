/* Configuration for the router. Includes things like bed size, acceleration
 * settings, etc.
 *
 * All units in the firmware are in STEPS! Configure steps/mm in the configuration
 * for the HOST. However, if steps/mm changes (for instance, if microstepping
 * changes), make sure to update the bed sizes here!
*/

#define BAUDRATE 115200

#define XSTEPS_PER_MM 80
#define YSTEPS_PER_MM 80
#define ZSTEPS_PER_MM 320

#define XMAX (XSTEPS_PER_MM * 390)
#define YMAX (YSTEPS_PER_MM * 390)
#define ZMAX (ZSTEPS_PER_MM * 100)

#define HOMING_FEEDRATE 1000

#define XMAX_FEEDRATE (XSTEPS_PER_MM * 25)
#define YMAX_FEEDRATE (YSTEPS_PER_MM * 25)
#define ZMAX_FEEDRATE (ZSTEPS_PER_MM * 20)

#define INVERT_X false
#define INVERT_Y false
#define INVERT_Z false
