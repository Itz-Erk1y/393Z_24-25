#include "main.h"
#include "robot.h"
#include "pid.h"
#include "api.h"
#include "auton.h"

using namespace std;
using namespace pros;


void autonomous() {

    if (atn == 0) {

    } else if (atn == 1) {

    } else if (atn == 2) {

    } else if (atn == 3) {

    } else if (atn == 4) {

    } else if (atn == 5) {

// First full goal
    driveStraight2(-200);
    driveTurn2(-90);
    driveClampS(-500 ,100, 70);
    RAMP.move(127);
    INTAKE.move(127);
    driveTurn2(180);
    driveStraight2(700);
    driveTurn2(135);
    driveStraight2(1010);
    driveTurn2(-35);
    driveStraight2(500);
    driveTurn2(0);
    driveStraight2(400);
    driveStraight2(300);
    driveTurn2(135);
    driveStraight2(300);
    driveTurn2(-135);
    driveStraight2(-300);
    mogo.set_value(false);
// Second full goal
    driveStraight2;
    driveTurn2(-90);
    driveClampS(-2000, -100, -70);
    RAMP.move(127);
    INTAKE.move(127);
     driveTurn2(180);
    driveStraight2(700);
    driveTurn2(-135);
    driveStraight2(1010);
    driveTurn2(35);
    driveStraight2(500);
    driveTurn2(0);
    driveStraight2(400);
    driveStraight2(300);
    driveTurn2(-135);
    driveStraight2(300);
    driveTurn2(135);
    driveStraight2(-300);
    mogo.set_value(false);
// 3rd full goal
    driveTurn2(-145);
    driveStraight2(2000);
    RAMP.move(0);
    driveTurn2(35);
    mogo.set_value(true);
    RAMP.move(127);
    driveTurn2(70);
    driveStraight2(650);
    driveTurn2(180);
    driveStraight2(200);
    driveTurn2(45); //exact do not change
    driveStraight2(250);
    driveTurn2(-45);
    driveStraight2(-200);
    mogo.set_value(false);
    // Final goal
    driveStraight2(600);
    driveTurn2(-115);
    driveStraight2(2500);
    driveStraight2(-200);

    }





    // driveClampS(-1300, 100, 90);
    // driveTurn2(-45);
    // INTAKE.move(127);
    // driveStraight2(1300)

    // driveStraight2(-1500); //go into devices, then click on 
    // //one of the chassis motors, then change it to a blue cartrige 
    // //(6:1) then reset the degrees by clicking it, then move the robot 
    // //the desired disance and read the degrees in order to get the number to
    // // plug into the code

    //robot starts off in the position that your auton will start, then you turn the bot on and wait for the 
    // imu to callibrate, then you turn the robot to the desired position/angle and read the position on
    //the brain. If the position is greater than 180, you must first subtract 360 from it, if not leave 
    // the number alone. Next input that number into the code and it should work.

    // piston.set_value(true); //if you want it to be released, change true to false
    // delay(400);
    // driveTurn2(-19); //change degree whenever you want
    // driveStraight2(1000); //change whenever you want

}