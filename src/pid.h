#include "main.h"
#include "global.h"
#include "pros/rtos.h"
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
int tTarget = 0;

int stored_error; //global
int stored_target; //global
int stored_enc_avg; //global
int stored_runtime; //global
int stored_min_max[20]; //global
int stored_time[20]; //global
int stored_imu;
int moveCount = 0;
bool intakeP = true;
bool flywheelP = false;


#define STRAIGHT_KP 6
#define STRAIGHT_KI 1
#define STRAIGHT_KD 1
#define INTEGRAL_KI 5
#define MAX_INTEGRAL 20
#define COUNT_CONST 50
#define TURN_KP 9.75 // 9.9 // 2
#define TURN_KI 1 // 0.1 // 0.1
#define TURN_KD 84 // 35 // 20
#define MAXTIME 300
#define MINSPEED 0



void spinToBlue() {
    double hue = optical.get_hue();
    // double hue2 = optical2.get_hue(); 
    optical.set_led_pwm(25);
    // optical2.set_led_pwm(25);

    while(hue < 80 || hue > 220) {
        hue = optical.get_hue();
        // hue2 = optical.get_hue();

        INTAKE.move(55);
        delay(5);
    }
    INTAKE.move(0);

}

void spinToRed() {
    
    double hue = optical.get_hue(); 
    // double hue2 = optical2.get_hue();
    optical.set_led_pwm(25);
    // optical2.set_led_pwm(25);

    while(hue > 70) {
        hue = optical.get_hue();
        // hue2 = optical.get_hue();

        INTAKE.move(100);
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
            
        }
        runtime_count++;
        pros::delay(10);

    }
    chas_move(0,0);
}


void straightDrive(int target) {
    reset_encoders();
    target *= 28.65;
    double dKP = 1;
    double dKI = 0.001;
    double dKD = 0.06;

    double tI = 42;

    double adjust = 0;
    double tError = 0;
    double tIntegral = 0;
    int dIntegral = 0;
    int dDerivative = 0;
    int dError = 0;
    int dPrev_Error = 0;
    int dPower = 0;
    int powerAdj = 5;
    double powerAdjConstant = 33.525;
    int currentPos = (LF.get_position() + LM.get_position() + LB.get_position() + RF.get_position() + RM.get_position() + RB.get_position()) / 6;
    int count = 0;
    int timeout = 0;
    double limiter = 0.01;
    int newCount = 0;
    double currHeading = 0;

    imu.set_heading(90);


    while (true) {
        currentPos = (LF.get_position() + LM.get_position() + LB.get_position() + RF.get_position() + RM.get_position() + RB.get_position()) / 6;
        
        dError = target-currentPos;
        if(abs(dError) < 750) {
            dIntegral += dError;        
        }

        dDerivative = (dError - dPrev_Error) * 100;
        dPrev_Error = dError;

        dPower = dKP * dError + dIntegral * dKI + dDerivative * dKD;

        if(dPower < 0) {
            dPower = min(dPower, -127);
        }
        else {
            dPower = min(dPower, 127);
        }
        
        currHeading = imu.get_heading();
        // 359 4
        //4 8 
        // 4 359
        //tError = fabs(initHeading - currHeading) < 90 ? initHeading - currHeading : 360 - max(initHeading, currHeading) + min(initHeading, currHeading) * (currHeading > initHeading ? 1 : -1);
        tError = imu.get_heading() - 90;
        tIntegral += tError / 100; 
        dPower *= limiter;
        // powerAdj = (imu.get_heading() - 90) * powerAdjConstant;
        powerAdj = abs(dPower) / 80.0 * tIntegral * tI;

        chas_move(dPower - powerAdj, dPower + powerAdj);
       

        if(limiter >= 1) {
            limiter = 1;
        }
        else {
            limiter += 0.05;
        }

        if (abs(target-currentPos) <= 150) count++;
        if (count >= COUNT_CONST) break; //|| runtime_count >= MAX_RUNTIME

        if (abs(target-currentPos) <= 300) timeout++;
        if(timeout >= MAXTIME) break;
        newCount++;
        delay(10);
    }
    chas_move(0,0);
}

void toggleIntakePiston() {
    if(intakeP) { 
        intakeP = !intakeP;
        intakePiston.set_value(false);
    }
    else {
        intakeP = !intakeP;
        intakePiston.set_value(true);
    }
}

void toggleFlywheelPiston() {
    if(flywheelP) {
        flywheelP = !flywheelP;
        anglerPiston.set_value(false);
    }
    else {
        flywheelP = !flywheelP;
        anglerPiston.set_value(true);
    }
}

void toggleExpansion() {
    expansion.set_value(true);
}

void fireFlywheel(int rep) {
    for(int i = 0; i < rep; i ++) {
        moveIntake(-127);
        delay(200);
        moveIntake(0);
        
        delay(750);
        //timmy was here
    }
}


void taskFlywheel() {
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
    //and here

    while(true) {
			wCurrSpeed = (F1.get_actual_velocity());
			
			//pushing value to back of the arraylist
            if(wNewCount%50 == 0) {
                con.print(0, 0, "Flywheel Temp: %f", F1.get_temperature());
            }
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

			wError = tTarget - actValue;

			if(abs(wError) < 25) { 
				wIntegral += wError * 0.01;
			}
			else {
				wIntegral = 0;
			}

			derivative = wError - wPrev_error;

			wFlyPower = wkV * tTarget + wkP*wError + wkI * wIntegral + wkD * wDerivative;

			F1.move(wFlyPower);

		
			
			indice = 1;
			tempSum = 0;
			divideSum = 0;
            wNewCount++;
			delay(10);
		}
  }

  void setTarget(int target) {
    tTarget = target;
    //hello
  }
void redRightGreed() {
    setTarget(510);
    Task flywheel(taskFlywheel, TASK_PRIORITY_DEFAULT
	, TASK_STACK_DEPTH_DEFAULT, "flywheelTask");
    delay(100);
    
    moveIntake(127); // pick up and fire 3 discs
    straightDrive(28);
    delay(100);
    pidturn(28);
    delay(575);
    fireFlywheel(3);

    setTarget(495);
    pidturn(90);
    toggleIntakePiston();
    // chas_move(40, 40);
    
    moveIntake(127);
    straightDrive(20);
    // delay(550);
    // chas_move(0,0);
    toggleIntakePiston();
    delay(1200);
//anime women
    straightDrive(-8); // back up to shoot
    delay(500);
    pidturn(22);
    delay(425);
    fireFlywheel(2);
    delay(100);
    pidturn(-21);
    moveIntake(-127);
    chas_move(-100,-100);
    flywheel.remove();
}

void redRight() {
    setTarget(508);
    Task flywheel(taskFlywheel, TASK_PRIORITY_DEFAULT
	, TASK_STACK_DEPTH_DEFAULT, "flywheelTask");
    delay(100);
    
    moveIntake(127); // pick up and fire 3 discs
    straightDrive(25);
    delay(500);
    pidturn(28);
    delay(1000);
    fireFlywheel(3);

    pidturn(0); // back to starting position
    straightDrive(-20);

    pidturn(-90); // turn go, turn go toggle
    straightDrive(-38);
    pidturn(0);
    chas_move(-40,-40);
    moveIntake(-127);
    delay(1000);
    moveIntake(0);
    chas_move(0,0);


    setTarget(0);
    flywheel.remove();
}

void redLeftGreed() {
    setTarget(590);
    Task flywheel(taskFlywheel, TASK_PRIORITY_DEFAULT
	, TASK_STACK_DEPTH_DEFAULT, "flywheelTask");

    //AHHHHHH slay!
    
    delay(100);
    
    moveIntake(127); // toggle roller
    
    chas_move(-30,-30);

    // spinToRed();
    delay(500);
    
    straightDrive(14);

    delay(100);
    
    moveIntake(0);
    
    pidturn(-7); // turn and fire
    fireFlywheel(2);
    
    delay(500);
    // setTarget(510);
    
    

    setTarget(502);

    toggleIntakePiston(); // intake second half
    
    moveIntake(127); //timmy sin/cos <- fr fr (for realsies)

    delay(100);

    pidturn(46);

    // chas_move(40, 40);
    straightDrive(25);

    delay(500);

    // chas_move(0, 0);
    toggleIntakePiston();
    

    delay(3300);

    pidturn(-15);
    delay(300);    
    fireFlywheel(3); // fire the three picked up
    
    delay(1000);

    setTarget(0);
    flywheel.remove();

}

void redLeft() {
    
    setTarget(590);
    Task flywheel(taskFlywheel, TASK_PRIORITY_DEFAULT
	, TASK_STACK_DEPTH_DEFAULT, "flywheelTask");

    //AHHHHHH slay!
    
    delay(100);
    
    moveIntake(127); // toggle roller
    
    chas_move(-30,-30);

    // spinToRed();
    delay(500);
    
    straightDrive(14);

    delay(100);
    
    moveIntake(0);
    
    pidturn(-7); // turn and fire
    fireFlywheel(2);
    
    delay(500);
    // setTarget(510);
    
    

    setTarget(502);

    toggleIntakePiston(); // intake second half
    
    moveIntake(127); //timmy sin/cos <- fr fr (for realsies)

    delay(100);

    pidturn(46);

    // chas_move(40, 40);
    straightDrive(25);

    delay(500);

    // chas_move(0, 0);
    toggleIntakePiston();
    

    delay(3300);

    pidturn(-15);
    delay(300);    
    fireFlywheel(3); // fire the three picked up
    
    delay(1000);

    setTarget(0);
    flywheel.remove();

}
void soloAwp() {
    
    setTarget(525);
    Task flywheel(taskFlywheel, TASK_PRIORITY_DEFAULT
	, TASK_STACK_DEPTH_DEFAULT, "flywheelTask");
    moveIntake(-127);
    chas_move(-20, -20);
    delay(380); // roller
    

    pidturn(0);
    
    moveIntake(127);
    toggleIntakePiston();

    straightDrive(45);
    delay(10);
    toggleIntakePiston();
    delay(500);
    straightDrive(43);
    delay(100);
    pidturn(-76);
    delay(1400);
    fireFlywheel(3);

    delay(100);
    pidturn(0);
    moveIntake(127);
    straightDrive(120);
    delay(150);

    pidturn(180);

    moveIntake(-127);
    chas_move(-127, -127);

    setTarget(0);
    flywheel.remove();
}


void skills() {
    Task flywheel(taskFlywheel, TASK_PRIORITY_DEFAULT
	, TASK_STACK_DEPTH_DEFAULT, "flywheelTask");
	setTarget(600);
	delay(2000);
	moveIntake(127);
	straightDrive(-2);
	delay(350);
	
	moveIntake(0);
	straightDrive(8);
	pidturn(-3);
	fireFlywheel(2);
	delay(100);
	moveIntake(0);
	pidturn(0);
	delay(100);
	intakePiston.set_value(false);
	reset_encoders();
	//timmy was here
	
	chas_move(100, 100);
	delay(200);
	chas_move(50, 50);
	delay(50);
	chas_move(0,0);
	moveIntake(127);
	delay(200);

	intakePiston.set_value(true);
	delay(2000);

	chas_move(50, 50);
	delay(200);
	chas_move(0,0);

	pidturn(90);

	
	setTarget(550);

	chas_move(-80,-80);
	delay(850);
	chas_move(0,0);

	delay(650);

	chas_move(-10,-10);
	delay(100);

	chas_move(0,0);

	delay(300);
	
	
	chas_move(100, 100);
	delay(300);

	chas_move(0,0);

	pidturn(5);
	
	fireFlywheel(3);

	chas_move(-50,-50);
	delay(400);
	chas_move(0,0);

	pidturn(45);

	expansion.set_value(true);

	
}




#endif