#include "main.h"
#include "global.h"
#include <chrono>
#include <cmath>
#include <iostream>
using namespace glb;
#ifndef _PID_
#define _PID_


void straightDrive(int target) {
   
   target *= 28.65;
   double kP = 0.6;
   double kI = 0.01;
   double kD = 0.0;
   int integral = 0;
   int derivative = 0;
   int error;
   int prev_error;
   double power;
   int powerAdj = 5;
   double powerAdjConst = 0;

   int current_pos = (RM.get_position() + RB.get_position() + RF.get_position() + LM.get_position() + LB.get_position() + LF.get_position())/6;
   imu.set_heading(90);
   while(abs(error) >= 15) {
     current_pos = (RM.get_position() + RF.get_position() + RB.get_position() + LF.get_position() + LM.get_position() + LB.get_position())/6;
     error = target - current_pos;
     integral += error;
     if(error == 0){
       integral = 0;
     }
     if(integral > 3000){
       integral = 0;
     }

     derivative = error - prev_error;
     prev_error = error;
     power = kP*error + integral*kI + derivative*kD;
     if(power < 0){
       if(power < -127) {
         power = -127;
       }
     }
     else{
       if(power > 0){
         if(power > 127) {
           power = 127;
         }
       }
     }
     powerAdj = (imu.get_heading()-90) * powerAdjConst;
     RM.move(power); RF.move(power); RB.move(power); LF.move(power); LB.move(power); LM.move(power);
     delay(10);
   }
   RF.move(0);
   RM.move(0);
   RB.move(0);
   LF.move(0);
   LM.move(0);
   LB.move(0);
  }


#endif