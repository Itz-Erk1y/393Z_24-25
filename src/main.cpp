#include "main.h"
#include "robot.h"
#include "pid.h"
#include "auton.h"


using namespace pros;
using namespace std;


/**
 * A callback function for LLEMU's center button.
 *
 * When this callback is fired, it will toggle line 2 of the LCD text between
 * "I was pressed!" and nothing.
 */
void on_center_button() {
	static bool pressed = false;
	pressed = !pressed;
	if (pressed) {
		pros::lcd::set_text(2, "I was pressed!");
	} else {
		pros::lcd::clear_line(2);
	}
}

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
	pros::lcd::initialize();
	pros::lcd::set_text(1, "Hello PROS User!");

	pros::lcd::register_btn1_cb(on_center_button);
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
int atn =5;
int pressed = 0;
string autstr;

void competition_initialize() {
	while(true){
		if (selec.get_value() == true) {
			pressed ++;
		} else {
	  		pressed = 0;
		}

		if (pressed == 1){
			atn++;
		}

if (atn == 0){
	autstr = "NONE";
	con.print(0, 0, "Aut 0: %s", autstr);
}
else if (atn == 1){
	autstr = "BLUE RIGHT";
	con.print(0, 0, "Aut 1: %s", autstr);
}
else if (atn == 2){
	autstr = "BLUE LEFT";
	con.print(0, 0, "Aut 2: %s", autstr);
}
else if (atn == 3){
	autstr = "RED RIGHT";
	con.print(0, 0, "Aut 3: %s", autstr);
}
else if (atn == 4){
	autstr = "RED LEFT";
	con.print(0, 0, "Aut 4: %s", autstr);
}
else if (atn == 5){
	autstr = "SKILLS";
	con.print(0, 0, "Aut 5: %s", autstr);
}
else if (atn == 6){
	atn = 0;
}

con.clear();
}
}

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
bool pistonToggle = false;

int time = 0;

bool fishToggle;
int macro = 0;

	while (true) {
if (selec.get_value() == true) {
			pressed ++;
		} else {
	  		pressed = 0;
		}

		if (pressed == 1){
			atn++;
		}

if (atn == 0){
	autstr = "NONE";

}
else if (atn == 1){
	autstr = "BLUE RIGHT";

}
else if (atn == 2){
	autstr = "BLUE LEFT";
}
else if (atn == 3){
	autstr = "RED RIGHT";

}
else if (atn == 4){
	autstr = "RED LEFT";
}
else if (atn == 5){
	autstr = "SKILLS";
	con.print(0, 0, "Aut 5: %s", autstr);
}
else if (atn == 6){
	atn = 0;
}

	//printing stuff
		if (time % 50 == 0 && time % 100 != 0 && time % 150 != 0){
			con.print(0, 0, "AUTON: %s			",autstr);
			//con.print(0, 0, "imu: %f	  	  ", imu.get_heading());
		} else if (time % 100 == 0 && time % 150 != 0){
			con.print(1, 0, "error: %f		    	", float(error));
		} else if (time % 150 == 0){
			con.print(2, 0, "time2: %f			", float(time2));
		}

		int power = con.get_analog(ANALOG_LEFT_Y);
		int RX = con.get_analog(ANALOG_RIGHT_X);
		int turn = int(abs(RX) * RX / 120);

		int left = power + turn;
		int right = power - turn;



		LF.move(left);
		LM.move(left);
		LB.move(left);
		RF.move(right);
		RF.move(right);
		RB.move(right);


	if   (con.get_digital(E_CONTROLLER_DIGITAL_R1)) {
		INTAKE.move(127);
	}
	else if (con.get_digital(E_CONTROLLER_DIGITAL_R2)) {
		INTAKE.move(-127);
	}
	else {
		INTAKE.move(0);
	}


	if   (con.get_digital(E_CONTROLLER_DIGITAL_L1)) {
		FISH.move(127);
		fishToggle = false;
	}
	else if (con.get_digital(E_CONTROLLER_DIGITAL_L2)) {
		FISH.move(-127);
		fishToggle = false;
	}


	if   (fishToggle == true){
		if (macro == 1){
			FISH.move(calcPIDlift(9000, roto.get_angle(),0,0));
		} else if (macro == 2){
			FISH.move(calcPIDlift(15000, roto.get_angle(),0,0));
		} else {
			macro = 1;
		}
	}

	if(con.get_digital_new_press(E_CONTROLLER_DIGITAL_X)) {
		macro++;
		fishToggle = true;
	}



	if   (con.get_digital_new_press(E_CONTROLLER_DIGITAL_DOWN)){
		pistonToggle = !pistonToggle;
	}

	piston.set_value(pistonToggle);

	if (con.get_digital_new_press(E_CONTROLLER_DIGITAL_X)) {
		driveStraight(1000);
	}

		time += 1;
		delay(1);


	}

}



