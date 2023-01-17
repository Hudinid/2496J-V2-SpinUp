#include "main.h"
using namespace pros;
#ifndef _GLOBALS_
#define _GLOBALS_

namespace glb {
    //#define TEST1 21
    //#define TEST3 8
    //#define TEST2 2
    #define F_1 17
    #define P_RF 18
    #define P_RM 20
    #define P_RB 13
    #define P_LF 14
    #define P_LM 16
    #define P_LB 15
    #define P_INTAKE 12
    

    Motor RF (P_RF, E_MOTOR_GEARSET_06);
    Motor RM (P_RM, E_MOTOR_GEARSET_06);
    Motor RB (P_RB, E_MOTOR_GEARSET_06);
    Motor LF (P_LF, E_MOTOR_GEARSET_06,1);
    Motor LM (P_LM, E_MOTOR_GEARSET_06,1);
    Motor LB (P_LB, E_MOTOR_GEARSET_06,1);
    Motor INTAKE (P_INTAKE, E_MOTOR_GEARSET_06);
    Motor F1 (F_1, E_MOTOR_GEARSET_06);

    //Motor flywheel(TEST1, E_MOTOR_GEARSET_06, 1);
    //Motor flywheel2(TEST3,E_MOTOR_GEARSET_06, 1);
    //Motor IDX(TEST2, 1);

    Controller con (E_CONTROLLER_MASTER);

    
}
#endif