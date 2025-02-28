#include "main.h"
#include "api.h"

//header guards
#ifndef PIDH
#define PIDH

#define STRAIGHT_KP 2
#define STRAIGHT_KI 0
#define STRAIGHT_KD 2

#define STRAIGHT_INTEGRAL_KI 40
#define STRAIGHT_MAX_INTEGRAL 14.5

#define ARC_HEADING_KP 1
#define ARC_HEADING_KI 0
#define ARC_HEADING_KD 0

#define ARC_HEADING_INTERGAL_KI 0
#define ARC_HEADING_MAX_INTERGAL 0

#define TURN_KP 1
#define TURN_KI 0
#define TURN_KD 0

#define TURN_INTEGRAL_KI 30
#define TURN_MAX_INTEGRAL 25

#define LIFT_KP 1
#define LIFT_KI 0
#define LIFT_KD 0

#define LIFT_INTEGRAL_KI 30
#define LIFT_MAX_INTEGRAL 25


extern void driveStraight(int target);
extern void driveStraight2(int target);
extern void driveTurn(int target);
extern void driveTurn2(int target);
extern void driveClampS(int target, int clamoDistance, int speed);
extern float error;
extern int time2;
extern void driveArcL(double theta, double radius, int timeout);


extern double calcPIDlift(double target, double input, int intergralKi, int manIntergal);
















#endif