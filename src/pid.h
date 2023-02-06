#include "main.h"
#include "global.h"
#include <chrono>
#include <cmath>
#include <list>
#include <iostream>
using namespace glb;
using namespace std;
using namespace pros;
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
   double powerAdjConst = 1;

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
     RM.move(power); RF.move(power); RB.move(power); LF.move(power - powerAdj); LB.move(power - powerAdj); LM.move(power - powerAdj);
     delay(10);
   }
   RF.move(0);
   RM.move(0);
   RB.move(0);
   LF.move(0);
   LM.move(0);
   LB.move(0);
  }

  void drive(int target) {
    double kP = 0;
    double kI = 0;
    double kD = 0;
    int integral = 0;
    int derivative = 0;
    int error = 0;
    int currLocation = 0;
    int power = 0;
    int prev_error; 
  }

  void spinFlywheel(int target) {
    float kP = 1.3; // 0.5 has a -13 to 1 range // 0.75
	  float kI = 0.22; // 0.05
	  float kD = 0.0;
	  float kV = 0.2;
	  float integral = 0;
	  float derivative = 0;
    int error = 0;
    int prev_error = 0;
    int currSpeed = 0;
    int flyPower = 0;
    int newCount = 0;
    bool hitToggleFSpeed = false;
    list<int> values;
    int vectorSize = 50;
    float divideSum = 0;
    float actValue = 0;
    float tempSum = 0;
    list<int>::iterator it;
    int indice = 0;

    while(true) {
			currSpeed = (F1.get_actual_velocity());
			
			//pushing value to back of the arraylist
			values.push_back(currSpeed);


			if(values.size() > vectorSize) {
				values.pop_front();
			}

			for(it = values.begin(); it != values.end(); it++){
				float multiplier = 0.01 * indice*indice;

				tempSum += (multiplier * *it);
				divideSum += multiplier;

				indice++;
			}

			actValue = tempSum / divideSum;

			prev_error = error;

			error = target - actValue;

			if(abs(error) < 25) { 
				integral += error * 0.01;
			}
			else {
				integral = 0;
			}

			derivative = error - prev_error;

			flyPower = kV * target + kP*error + kI * integral + kD * derivative;

			F1.move(flyPower);

		
			
			indice = 1;
			tempSum = 0;
			divideSum = 0;

			delay(10);
		}
		
  }


#endif