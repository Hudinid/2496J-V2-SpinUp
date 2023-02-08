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

float kp;
float ki;
float kd;
float power = 0;
float derivative = 0;
float integral = 0;
float error = 0;
float prev_error = 0;
float prev_derivative;
int stored_error; //global
int stored_target; //global
int stored_enc_avg; //global
int stored_runtime; //global
int stored_min_max[20]; //global
int stored_time[20]; //global
int stored_imu;
int moveCount = 0;

#define STRAIGHT_KP 6
#define STRAIGHT_KI 1
#define STRAIGHT_KD 1
#define INTEGRAL_KI 5
#define MAX_INTEGRAL 20
#define COUNT_CONST 50
#define TURN_KP 12 // 9.9 // 2
#define TURN_KI 1 // 0.1 // 0.1
#define TURN_KD 24  // 35 // 20
#define MAXTIME 300
#define MINSPEED 0



void spinToBlue() {
    double hue = optical.get_hue(); 
    optical.set_led_pwm(25);
    while(hue < 80 || hue > 220) {
        hue = optical.get_hue();
        INTAKE.move(55);
        delay(5);
    }
    INTAKE.move(0);

}

void spinToRed() {
    
    double hue = optical.get_hue(); 
    optical.set_led_pwm(25);
    while(hue > 70) {
        hue = optical.get_hue();
        INTAKE.move(55);
        delay(5);
    }

    INTAKE.move(0);
}


void spinFlywheel(int target) {
    float wkP = 1.3; // 0.5 has a -13 to 1 range // 0.75
    float wkI = 0.22; // 0.05
    float wkD = 0.0;
    float wkV = 0.2;
    float wIntegral = 0;
    float wDerivative = 0;
    int wError = 0;
    int wPrev_error = 0;
    int wCurrSpeed = 0;
    int wFlyPower = 0;
    int wNewCount = 0;
    bool hitToggleFSpeed = false;
    list<int> values;
    int vectorSize = 50;
    float divideSum = 0;
    float actValue = 0;
    float tempSum = 0;
    list<int>::iterator it;
    int indice = 0;

    while(true) {
			wCurrSpeed = (F1.get_actual_velocity());
			
			//pushing value to back of the arraylist
			values.push_back(wCurrSpeed);


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

			wPrev_error = wError;

			wError = target - actValue;

			if(abs(wError) < 25) { 
				integral += error * 0.01;
			}
			else {
				integral = 0;
			}

			derivative = error - prev_error;

			wFlyPower = wkV * target + wkP*error + wkI * integral + wkD * derivative;

			F1.move(wFlyPower);

		
			
			indice = 1;
			tempSum = 0;
			divideSum = 0;

			delay(10);
		}
  }

void moveIntake(int speed) { 
    INTAKE.move(speed);
}

void stopIntake() {
    INTAKE.move(0);
}

int signOf(int num) {
    if (num > 0) return 1;
    if (num < 0) return -1;
    return 0;
}

bool sameSign(int num1, int num2)
{
    return num1 >= 0 && num2 >= 0 || num1 < 0 && num2 < 0;
}

void chas_move(int left_power, int right_power){
    RF.move(right_power);
    RB.move(right_power);
    RM.move(right_power);
    LF.move(left_power);
    LB.move(left_power);
    LM.move(left_power);
}

void reset_encoders(){
    RF.tare_position();
    RM.tare_position();
    RB.tare_position();
    LF.tare_position();
    LM.tare_position();
    LB.tare_position(); 
}

void setValues(float p_kp, float p_ki, float p_kd){
    kp = p_kp;
    ki = p_ki; 
    kd = p_kd; 
}

float calc(float target, float input, float maxI, int integralKI) {
    prev_error = error;   
    error = target - input;

    prev_derivative = derivative; derivative = error - prev_error; 

    if(abs(error) < integralKI) {
        integral += error;
    }
    else {
        integral = 0; 
    }

    if (integral >= 0) {
        integral = min(integral, maxI);
    }
    else {
        integral = max(integral,-maxI);
    }
    
    if (abs(kp*error) <= MINSPEED) {
        power = signOf(error)*MINSPEED + ki*integral + kd*derivative;
    } 
    else {
        power = kp*error + ki*integral + kd*derivative;
    }

    return power;
}

void pidmove (int target) {
    setValues(STRAIGHT_KP,STRAIGHT_KI,STRAIGHT_KD);
    reset_encoders();
    int count = 0;
    float encoder_average;
    float voltage;
    float error = 0;
    int runtime_count = 0;


    while(true){
        encoder_average = (LF.get_position() + RF.get_position())/2; //Average encoder value for left front and right front chassis motors
        voltage = calc(target, encoder_average, MAX_INTEGRAL, INTEGRAL_KI); //Setting voltage to the PID power
        chas_move(voltage, voltage); //Moving chassis based on voltage value
        error = target-encoder_average; //Creating error
        if (abs(error) <= 50) count++; //Incrementing count if error is less than 50
        if (!sameSign(prev_derivative, derivative)) {
            // linear[moveCount].cStore(error, stored_runtime);
        }
        if (count >= COUNT_CONST) {
            stored_error = error;
            stored_enc_avg = ((LF.get_position() + RF.get_position())/2);
            break;
        }
        if (runtime_count % 5 == 0 && !(runtime_count % 10 == 0)) {
            con.clear();
        } else if (runtime_count % 10 == 0) {
            con.print(1,0,"Error: %f", error);
        }
        stored_runtime += 10;
        runtime_count++;
        pros::delay(10);
    }
    // linear[moveCount].store(target, stored_runtime);
    chas_move(0,0);
}
void pidturn (float target){
    setValues(TURN_KP,TURN_KI,TURN_KD);
    // switch(turnType) {
    //     case 0:
    //         setValues(TURN_KP,TURN_KI,TURN_KD);
    //         break;
    //     case 1:
    //         setValues(TURN_KP1,TURN_KI1,TURN_KD1);
    //         break;
    //     case 2:
    //         setValues(TURN_KP2,TURN_KI2,TURN_KD2);
    //         break;
    // } 
    float position; 
    float start; 
    float voltage;
    int count = 0;
    int arr_count = 0;
    int turn_time = 0;
    int runtime_count = 0;
    int timeout = 0;


    start = imu.get_rotation();

    while(true) { 
        position = imu.get_rotation();

        voltage = calc(target, position, MAX_INTEGRAL, INTEGRAL_KI);

        chas_move(voltage, -voltage);

        if (abs(target-position) <= 0.75) count++;
        if (count >= COUNT_CONST) break; //|| runtime_count >= MAX_RUNTIME

        if (abs(target-position) <= 2) timeout++;
        if(timeout >= MAXTIME) break;

        if (runtime_count % 5 == 0 && !(runtime_count % 10 == 0)) {
            con.clear();
        } else if (runtime_count % 10 == 0) {
            con.print(0,0,"Header: %f", imu.get_rotation());
            con.print(1,0,"Kp:%f, Ki:%f", TURN_KP, TURN_KI);
            con.print(2,0,"Kd:%f", TURN_KD);
        }
        runtime_count++;
        pros::delay(10);

    }
    chas_move(0,0);
}








#endif