#include "main.h"
using namespace pros;
#ifndef _GLOBALS_
#define _GLOBALS_

namespace glb {
    #define TEST1 16
    #define TEST3 8
    #define TEST2 20

    Motor flywheel(TEST1, E_MOTOR_GEARSET_06, 1);
    Motor flywheel2(TEST3,E_MOTOR_GEARSET_06, 1);
    Motor IDX(TEST2, 1);

    Controller con (E_CONTROLLER_MASTER);
}
#endif