#include "main.h"
#include "pros/adi.hpp"
#include "pros/optical.hpp"
using namespace pros;
#ifndef _GLOBALS_
#define _GLOBALS_

namespace glb {
    //#define TEST1 21
    //#define TEST3 8
    //#define TEST2 2
    #define F_1 17
    #define P_RF 18
    #define P_RM 11
    #define P_RB 13
    #define P_LF 14
    #define P_LM 16
    #define P_LB 12
    #define P_INTAKE 19
    #define P_IMU 9
    #define P_OPTICAL 21
    #define P_OPTICAL2 20
    #define P_INTAKEPISTON 'A'
    #define P_ANGLERPISTON 'B'
    #define P_AUTONSELECTOR 'C'
    
    

    Motor RF (P_RF, E_MOTOR_GEARSET_06);
    Motor RM (P_RM, E_MOTOR_GEARSET_06);
    Motor RB (P_RB, E_MOTOR_GEARSET_06);
    Motor LF (P_LF, E_MOTOR_GEARSET_06,1);
    Motor LM (P_LM, E_MOTOR_GEARSET_06,1);
    Motor LB (P_LB, E_MOTOR_GEARSET_06,1);
    Motor INTAKE (P_INTAKE, E_MOTOR_GEARSET_06);
    Motor F1 (F_1, E_MOTOR_GEARSET_06, 1);
    ADIDigitalOut intakePiston(P_INTAKEPISTON);
    ADIDigitalOut anglerPiston(P_ANGLERPISTON);
    ADIDigitalIn autonSelector(P_AUTONSELECTOR);

    //Motor flywheel(TEST1, E_MOTOR_GEARSET_06, 1);
    //Motor flywheel2(TEST3,E_MOTOR_GEARSET_06, 1);
    //Motor IDX(TEST2, 1);

    Controller con (E_CONTROLLER_MASTER);

    Imu imu (P_IMU);

    Optical optical(P_OPTICAL);
    Optical optical2(P_OPTICAL2);
    
}
#endif