#include "main.h"
#include "global.h"
#include "pid.h"
#include "pros/misc.h"
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
void autonomous() {}

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
void opcontrol() {
	int flySpeed = 500;
	bool toggleFlyWheel = false;
	bool hitFlyWheelToggle = false;
	int count = 0;
	con.clear();

	float kP = 0.8; // 0.5 has a -13 to 1 range
	float kI = 0.5;
	float kD = 0;
	int integral = 0;
	int derivative = 0;
	
	int target = 500;
	int error = 0;
	int prev_error = 0;

	double currSpeed = 0;
	int constant = 106; // 500 / 600 * 127
	int power = 0;

	//target is 500
	while (true) {
		currSpeed = (flywheel.get_actual_velocity() + flywheel2.get_actual_velocity())/2;
		prev_error = error;
		error = target - currSpeed;

		if(error > 0 && abs(error) < 2) { 
			integral += error;
		}
		// else if(error < 0 && abs(error) < 2) {
		// 	integral += error;
		// }
		else {
			integral = 0;
		}

		derivative = error - prev_error;

		power = kP*error + kI * integral + kD * derivative + constant;

		flywheel.move(power);
		flywheel2.move(power);

		if(count % 50 == 0 && count % 100 != 0 && count % 150 != 0) {
			con.print(0, 0, "Act. Vel: %f", (currSpeed));
		}
		if(count % 100 == 0 && count % 150 != 0) {
			con.print(1, 0, "Power: %d", power);
		}
		if(count % 150 == 0) { 
			con.print(2, 0, "Error: %d", error);
		}
// 		if(con.get_digital(E_CONTROLLER_DIGITAL_L1)) { // toggle the automatic flywheel
// 			if(!hitFlyWheelToggle) { 
// 				hitFlyWheelToggle = true;
// 				toggleFlyWheel = !toggleFlyWheel;
// 			}
// 		}
// //mm robot yes monke
// 		else if(con.get_digital(E_CONTROLLER_DIGITAL_UP)) {
// 			if(!hitFlyWheelToggle) {
// 				hitFlyWheelToggle = true;
// 				flySpeed += 10;
// 				if(flySpeed > 600) {
// 					flySpeed = 0;
// 				}
// 			}
// 		}

// 		else if(con.get_digital(E_CONTROLLER_DIGITAL_DOWN)) { 
// 			if(!hitFlyWheelToggle) {
// 				hitFlyWheelToggle = true;
// 				flySpeed -= 10;
// 				if(flySpeed < 0) { 
// 				    flySpeed = 600;
// 				}
// 			}
// 		}
// 		else hitFlyWheelToggle = false;

// 		if(toggleFlyWheel) {
// 			flywheel.move(127);
// 			flywheel2.move(127);
			
// 		} 
// 		else {
// 			flywheel.move(0);
// 			flywheel2.move(0);
// 		}

// 		if(con.get_digital(E_CONTROLLER_DIGITAL_A)) {
// 			IDX.move_relative(425,127);
// 		}

		
		

		count ++;
		pros::delay(5);
	}
}
