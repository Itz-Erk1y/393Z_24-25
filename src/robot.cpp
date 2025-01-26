#include "api.h"
#include "main.h"
#include "robot.h"



#define LF_PORT 4
#define LM_PORT 3
#define LB_PORT 2
#define IMU_PORT 7
#define RF_PORT 1
#define RM_PORT 6
#define RB_PORT 13
#define INTAKE_PORT 5
#define RAMP_PORT 9
#define ROTO_PORT 16
#define FISH_PORT 17

//rotation sensor
pros::Rotation roto(ROTO_PORT);

pros::Motor LF (LF_PORT, pros::E_MOTOR_GEARSET_06, false);
pros::Motor LM (LM_PORT, pros::E_MOTOR_GEARSET_06, false);
pros::Motor LB (LB_PORT, pros::E_MOTOR_GEARSET_06, false);
pros::Motor RF (RF_PORT, pros::E_MOTOR_GEARSET_06, true);
pros::Motor RM (RM_PORT, pros::E_MOTOR_GEARSET_06, true);
pros::Motor RB (RB_PORT, pros::E_MOTOR_GEARSET_06, true);
pros::Motor FISH (FISH_PORT, pros::E_MOTOR_GEARSET_06, true);



// controller
pros::Controller con (pros::E_CONTROLLER_MASTER);

//Intake
pros::Motor INTAKE (INTAKE_PORT, pros::E_MOTOR_GEARSET_06, false);

// Ramp
pros::Motor RAMP (RAMP_PORT, pros::E_MOTOR_GEARSET_06, false);

//Mogo
pros::ADIDigitalOut mogo ('A', false);

//piston
pros::ADIDigitalOut piston ('B', false);

//auton selector
pros:: ADIDigitalIn selec ('C');

//
pros::Imu imu (IMU_PORT);


