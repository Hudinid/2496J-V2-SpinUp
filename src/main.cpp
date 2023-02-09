#include "main.h"
#include "global.h"
#include "pid.h"
#include "pros/misc.h"
#include <iostream>
#include <fstream>
#include <list>
#include <iostream>

using namespace pros;
using namespace std;	
using namespace glb;

/**
 * A callback function for LLEMU's center button.
 *
 * When this callback is fired, it will toggle line 2 of the LCD text between
 * "I was pressed!" and nothing.
 */
void on_center_button() {
	
}

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
	
}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize() {}

/**
 * Runs the user autonomous code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the autonomous
 * mode. Alternatively, this function may be called in initialize or opcontrol
 * for non-competition testing purposes.
 *
 * If the robot is disabled or communications is lost, the autonomous task
 * will be stopped. Re-enabling the robot will restart the task, not re-start it
 * from where it left off.
 */
void autonomous() {
	imu.reset();
	while(imu.is_calibrating()) delay(5);

	// pidturn(180);
	straightDrive(100);
}

/**
 * Runs the operator control code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the operator
 * control mode.
 *
 * If no competition control is connected, this function will run immediately
 * following initialize().
 *
 * If the robot is disabled or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart the
 * task, not resume it from where it left off.
 */ 

 // USE TASKS FOR FLYWHEEL PID
void opcontrol() {
	int flySpeed = 420;
	bool toggleFlyWheel = false;
	bool hitFlyWheelToggle = false;
	int flywheelSpeeds = 1;
	int count2 = 0;
	int count = 0;
	int setFSpeed = 0;
	bool PIntakeActive = false;
	bool PIntakeButton = false;
	bool PAnglerActive = false;
	bool PAnglerButton = false;

	con.clear();

	float kP = 1.3; // 0.5 has a -13 to 1 range // 0.75
	float kI = 0.22; // 0.05
	float kD = 0.0;
	float kV = 0.2;
	float integral = 0;
	float derivative = 0;
	
	int target = 470;

	int error = 0;
	int prev_error = 0;

	int currSpeed = 0;
	
	int flyPower = 0;
	

	int newCount = 0;
	bool hitToggleFSpeed = false;
	list<int> values;
	int vectorSize = 50;

	float divideSum = 0;
	//target is 500
	
	float actValue = 0;
	float tempSum = 0;

	list<int>::iterator it;
	int indice = 0;

	while (true) {

		int power = con.get_analog(ANALOG_LEFT_Y); // left joystick y axis is power
		int valForTurn = con.get_analog(ANALOG_RIGHT_X); // right joystick x axis controls turn


		// double turn = (abs(valForTurn) * valForTurn / 75);
		double turn = valForTurn; 
		
		int left = power + turn; // implement turning
		int right = power - turn; 

		RF.move(right);
		RM.move(right);
		RB.move(right); // hi
		LF.move(left);
		LM.move(left);
		LB.move(left);	


		if(count%50 == 0) {
			con.print(0, 0, "Flywheel Speed: %d", target);
		}

		if(toggleFlyWheel && con.get_digital(E_CONTROLLER_DIGITAL_R2)) {
			F1.move_velocity(600);
			
			count2 ++;
			if(count2 % 1000) {
				con.rumble(".");
			}
			
		} 
		else if(toggleFlyWheel) {
			count2 ++;
			if(count2 % 1000) {
				con.rumble(".");
			}
			currSpeed = (F1.get_actual_velocity());
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

			count2 ++;
			if(count2 % 1000) {
				con.rumble(".");
			}

			delay(5);
		}
		else F1.move(0);


		if(con.get_digital(E_CONTROLLER_DIGITAL_R1)) { // then allow for manual control through R1 and R2
				INTAKE.move(127);
		}
		else if(con.get_digital(E_CONTROLLER_DIGITAL_R2)){
			INTAKE.move(-127);
		}
		else {
			INTAKE.move(0);
		}


		if(con.get_digital(E_CONTROLLER_DIGITAL_L1)) { // toggle the automatic flywheel
			if(!hitFlyWheelToggle) { 
				hitFlyWheelToggle = true;
				toggleFlyWheel = !toggleFlyWheel;
			}
		}
		// else hitFlyWheelToggle = false;

// //mm robot yes monke
		else if(con.get_digital(E_CONTROLLER_DIGITAL_UP)) {
			if(!hitFlyWheelToggle) {
				hitFlyWheelToggle = true;
				target += 10;
				if(target > 600) {
					target = 0;
				}
			}
		}

		else if(con.get_digital(E_CONTROLLER_DIGITAL_DOWN)) { 
			if(!hitFlyWheelToggle) {
				hitFlyWheelToggle = true;
				target -= 10;
				if(target < 0) { 
				    target = 600;
				}
			}
		}

		else hitFlyWheelToggle = false;

		if(con.get_digital(E_CONTROLLER_DIGITAL_Y)) {
			con.rumble(".");
			
			if(!hitToggleFSpeed) {
				hitToggleFSpeed = true;
				setFSpeed ++;
				if(setFSpeed >= flywheelSpeeds) { 
					setFSpeed = 0;
					target = 420;
					
				}
			}

		}
		else hitToggleFSpeed = false;

		if(con.get_digital(E_CONTROLLER_DIGITAL_B)) {
			if(!PIntakeButton) {
				PIntakeButton = true;
				if(PIntakeActive) {
					PIntakeActive = !PIntakeActive;
					intakePiston.set_value(false);
				}
				else {
					PIntakeActive = !PIntakeActive;
					intakePiston.set_value(true);
				}
			}
		}
		else PIntakeButton = false;
		// not pid code

		if(con.get_digital(E_CONTROLLER_DIGITAL_L2)) {
			if(!PAnglerButton) {
				PAnglerButton = true;
				if(PAnglerActive) {
					PAnglerActive = !PAnglerActive;
					anglerPiston.set_value(false);
				}
				else {
					PAnglerActive = !PAnglerActive;
					anglerPiston.set_value(true);
				}
			}
		}
		else PAnglerButton = false;
	
		


		//pid code
		// if(toggleFlyWheel) {
		// 	currSpeed = (F1.get_actual_velocity());
			
		// 	//pushing value to back of the arraylist
		// 	values.push_back(currSpeed);


		// 	if(values.size() > vectorSize) {
		// 		values.pop_front();
		// 	}

		// 	for(it = values.begin(); it != values.end(); it++){
		// 		float multiplier = 0.01 * indice*indice;

		// 		tempSum += (multiplier * *it);
		// 		divideSum += multiplier;

		// 		indice++;
		// 	}

		// 	actValue = tempSum / divideSum;

		// 	prev_error = error;

		// 	error = target - actValue;

		// 	if(abs(error) < 25) { 
		// 		integral += error * 0.01;
		// 	}
		// 	else {
		// 		integral = 0;
		// 	}

		// 	derivative = error - prev_error;

		// 	flyPower = kV * target + kP*error + kI * integral + kD * derivative;

		// 	F1.move(flyPower);

		// 	// printf("%f\n", actValue);
		// 	// delay(5);			

		// 	printf("%f\n", actValue);
		// 	printf("%d\n", flyPower);
		// 	printf("%d\n", error);
		// 	printf("%f\n", tempSum);
		// 	printf("%f\n", kI*integral);
		// 	printf("%f\n", kP*error);
			
		
			
		// 	indice = 1;
		// 	tempSum = 0;
		// 	divideSum = 0;

		// 	delay(5);

		// }
		// else {
		// 	F1.move(0);
		// }	
			// delay(5);

		count ++;
		newCount++;
		pros::delay(10);
	}
}
