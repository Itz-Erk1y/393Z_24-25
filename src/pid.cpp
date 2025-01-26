#include "main.h"
#include "robot.h"
#include "pid.h"
#include "api.h"


using namespace pros;
using namespace c;
using namespace std;


double vKp;
double vKi;
double vKd;
float error;
double prevError;
int intergal;
int derivative;
int time2;
double power;

//calc2
double vKp2;
double vKi2;
double vKd2;
float error2;
double prevError2;
int intergal2;
int derivative2;
int time22;
double power2;

//calc
double vKp3;
double vKi3;
double vKd3;
float error3;
double prevError3;
int intergal3;
int derivative3;
int time23;
double power3;

//roto
double vKplift;
double vKilift;
double vKdlift;
float errorlift;
double prevErrorlift;
int intergallift;
int derivativelift;
int time2lift;
double powerlift;

void setConstants(double kp, double ki, double kd){
    vKp = kp;
    vKi = ki;
    vKd = kd;
}

void resetEncoders() {
    LF.tare_position();
    LM.tare_position();
    LB.tare_position();
    RF.tare_position();
    RM.tare_position();
    RB.tare_position();
}
void chasMove(int voltageLF, int voltageLM, int voltageLB, int voltageRF, int voltageRM, int voltageRB){
        LF.move(voltageLF);
        LM.move(voltageLM);
        LB.move(voltageLB);
        RF.move(voltageRF);
        RM.move(voltageRM);
        RB.move(voltageRB);
}


double calcPID(double target, double input, int integralKi,int maxIntegral){
    int integral;

    prevError = error; 
    error = target - input;
    
    if(abs(error) < integralKi) {
        integral += error;
    } else {
        integral = 0;
    }

    if(integral >= 0) {
        integral = min(integral, maxIntegral);
    } else {
        integral = max(integral, -maxIntegral);
    }

    derivative = error - prevError;

    power = (vKp * error) + (vKd * integral) + (vKd * derivative);
    
    return power;
}


double calcPID2(double target, double input, int integralKi,int maxIntegral){
    int integral2;

    prevError2 = error2; 
    error2 = target - input;
    
    if(abs(error) < integralKi) {
        integral2 += error2;
    } else {
        integral2 = 0;
    }

    if(integral2 >= 0) {
        integral2 = min(integral2, maxIntegral);
    } else {
        integral2 = max(integral2, -maxIntegral);
    }

    derivative2 = error2 - prevError2;

    power2 = (vKp * error2) + (vKd * integral2) + (vKd * derivative2);
    
    return power2;
}

double calcPID3(double target, double input, int integralKi,int maxIntegral){
    int integral3;

    prevError3 = error3; 
    error3 = target - input;
    
    if(abs(error) < integralKi) {
        integral3 += error3;
    } else {
        integral3 = 0;
    }

    if(integral3 >= 0) {
        integral3 = min(integral3, maxIntegral);
    } else {
        integral3 = max(integral3, -maxIntegral);
    }

    derivative3 = error3 - prevError3;

    power3 = (vKp * error3) + (vKd * integral3) + (vKd * derivative3);
    
    return power3;
}

double calcPIDlift(double target, double input, int integralKi,int maxIntegral){
    int integrallift;

    prevErrorlift = errorlift; 
    errorlift = target - input;
    
    if(abs(error) < integralKi) {
        integrallift += errorlift;
    } else {
        integrallift = 0;
    }

    if(integrallift >= 0) {
        integrallift = min(integrallift, maxIntegral);
    } else {
        integrallift = max(integrallift, -maxIntegral);
    }

    derivativelift = errorlift - prevErrorlift;

    powerlift = (vKp * errorlift) + (vKd * integrallift) + (vKd * derivativelift);
    
    return powerlift;
}


void driveStraight(int target) {
    int timeout = 30000;

    double x = 0;
    x = double(abs(target));
    timeout = (0 * pow(x,5)) +(0 * pow(x,4)) + (0 * pow(x, 3)) + (0 * pow(x, 2)) + (0 * x) + 361.746;
    
    imu.tare();

    double voltage;
    double encoderAvg;
    int count = 0;
    double init_heading = imu.get_heading();
    double heading_error = 0;
    int cycle = 0;
    time2 = 0;
    
resetEncoders();
setConstants(STRAIGHT_KP, STRAIGHT_KI, STRAIGHT_KD);

    while(true) {

        encoderAvg = (LB.get_position() + RB.get_position()) / 2;
        voltage = calcPID(target, encoderAvg, STRAIGHT_INTEGRAL_KI, STRAIGHT_MAX_INTEGRAL);

        if(imu.get_heading() < 180){
            heading_error = init_heading - imu.get_heading();
        }
        else {
            heading_error =((360 - imu.get_heading()) - init_heading);
        }
        
        heading_error = 0;

        // anything above 127 gets set to 127

        if(voltage > 127){
            voltage = 127;
        } else if (voltage < -127){
            voltage = -127;
        }
        
        chasMove((voltage + heading_error ), (voltage + heading_error), (voltage + heading_error), (voltage - heading_error), (voltage - heading_error), (voltage - heading_error));
        if (abs(target - encoderAvg) <= 4) count++;
        if (count >= 20 || time2 > timeout){
            // break;
        }
   
        // || = or

        if (time2 % 50 == 0 && time2 % 100 != 0 && time2 % 150 != 0){
            con.print(0, 0, "ERROR: %f      ", float(error));
        } else if (time2 % 100 == 0 && time2 % 150){
            con.print(1, 0, "EncoderAvg: %f        ", float(encoderAvg));
        } else if (time2 % 150 == 0){
            con.print(2, 0, "Voltage: %f        ", float(voltage + heading_error));
        }

        delay (10);
        time2 += 10;

    }      
LF.brake();
LM.brake();
LB.brake();
RF.brake();
RM.brake();
RB.brake();


}


void driveStraight2(int target) {
    int timeout = 30000;

    double x = 0;
    x = double(abs(target));
    timeout = (0 * pow(x,5)) +(0 * pow(x,4)) + (0 * pow(x, 3)) + (0 * pow(x, 2)) + (0 * x) + 361.746;


    double voltage;
    double encoderAvg;
    int count = 0;
    double init_heading = imu.get_heading();
    double heading_error = 0;
    int cycle = 0;
    time2 = 0;
    
resetEncoders();
setConstants(STRAIGHT_KP, STRAIGHT_KI, STRAIGHT_KD);

    while(true) {

        encoderAvg = (LB.get_position() + RB.get_position()) / 2;
        voltage = calcPID(target, encoderAvg, STRAIGHT_INTEGRAL_KI, STRAIGHT_MAX_INTEGRAL);

        if(imu.get_heading() < 180){
            heading_error = init_heading - imu.get_heading();
        }
        else {
            heading_error =((360 - imu.get_heading()) - init_heading);
        }
        
        heading_error = 0;

        // anything above 127 gets set to 127

        if(voltage > 127){
            voltage = 127;
        } else if (voltage < -127){
            voltage = -127;
        }
        
        chasMove((voltage + heading_error ), (voltage + heading_error), (voltage + heading_error), (voltage - heading_error), (voltage - heading_error), (voltage - heading_error));
        if (abs(target - encoderAvg) <= 4) count++;
        if (count >= 20 || time2 > timeout){
            // break;
        }
   
        // || = or

        if (time2 % 50 == 0 && time2 % 100 != 0 && time2 % 150 != 0){
            con.print(0, 0, "ERROR: %f      ", float(error));
        } else if (time2 % 100 == 0 && time2 % 150){
            con.print(1, 0, "EncoderAvg: %f        ", float(encoderAvg));
        } else if (time2 % 150 == 0){
            con.print(2, 0, "Voltage: %f        ", float(voltage + heading_error));
        }

        delay (10);
        time2 += 10;

    }      
LF.brake();
LM.brake();
LB.brake();
RF.brake();
RM.brake();
RB.brake();


}

void driveClampS(int target, int clampDistance, int speed) {
    int timeout = 30000;

    double x = 0;
    x = double(abs(target));
    timeout = (0 * pow(x,5)) +(0 * pow(x,4)) + (0 * pow(x, 3)) + (0 * pow(x, 2)) + (0 * x) + 361.746;


    double voltage;
    double encoderAvg;
    int count = 0;
    double init_heading = imu.get_heading();
    double heading_error = 0;
    int cycle = 0;
    time2 = 0;
    
resetEncoders();
setConstants(STRAIGHT_KP, STRAIGHT_KI, STRAIGHT_KD);

    while(true) {

        encoderAvg = (LB.get_position() + RB.get_position()) / 2;
        voltage = calcPID(target, encoderAvg, STRAIGHT_INTEGRAL_KI, STRAIGHT_MAX_INTEGRAL);

        if(imu.get_heading() < 180){
            heading_error = init_heading - imu.get_heading();
        }
        else {
            heading_error =((360 - imu.get_heading()) - init_heading);
        }
        
        heading_error = 0;

        // anything above 127 gets set to 127

        // if(voltage > 127){
        //     voltage = 127;
        // } else if (voltage < -127){
        //     voltage = -127;
        // }
        
        if (abs(error) < clampDistance){
            mogo.set_value(true);
        }

        if(voltage > 127 * double(speed)/100.0){
            voltage = 127 * double(speed)/100.0;
        } else if (voltage < -127 *double(speed)/100.0){
            voltage = -127 * double(speed)/100.0;
        }

        chasMove((voltage + heading_error ), (voltage + heading_error), (voltage + heading_error), (voltage - heading_error), (voltage - heading_error), (voltage - heading_error));
        if (abs(target - encoderAvg) <= 4) count++;
        if (count >= 20 || time2 > timeout){
            // break;
        }
   
        // || = or

        if (time2 % 50 == 0 && time2 % 100 != 0 && time2 % 150 != 0){
            con.print(0, 0, "ERROR: %f      ", float(error));
        } else if (time2 % 100 == 0 && time2 % 150){
            con.print(1, 0, "EncoderAvg: %f        ", float(encoderAvg));
        } else if (time2 % 150 == 0){
            con.print(2, 0, "Voltage: %f        ", float(voltage + heading_error));
        }

        delay (10);
        time2 += 10;

    }      
LF.brake();
LM.brake();
LB.brake();
RF.brake();
RM.brake();
RB.brake();


}



void driveTurn(int target) {
double voltage;
double position;
int count = 0;
time2 = 0;


    setConstants(TURN_KP, TURN_KI, TURN_KD);

    int timeout = 30000;

    double x = 0;
    x = double(abs(target));
    timeout = (0 * pow(x,5)) +(0 * pow(x,4)) + (0 * pow(x, 3)) + (0 * pow(x, 2)) + (0 * x) + 361.746;
    

    imu.tare_heading(); 

    while(true) {

        position = imu.get_heading();

        if (position > 180) {
            position = position - 360; // =^.,.^= S
        }

        voltage = calcPID(target, position,TURN_INTEGRAL_KI, TURN_MAX_INTEGRAL);

        chasMove(voltage, voltage, voltage, -voltage, -voltage, -voltage);

        if (abs(target - position) <= 0.5) count++;
        if (count >= 20 || time2 > timeout){
            break;
        }

         if (time2 % 50 == 0 && time2 % 100 != 0 && time2 % 150 != 0){
            con.print(0, 0, "ERROR: %f      ", float(error));
        } else if (time2 % 100 == 0 && time2 % 150){
            con.print(1, 0, "IMU: %f        ", float(imu.get_heading()));
        } else if (time2 % 150 == 0){
            con.print(2, 0, "Time: %f        ", float(time2));
        }

        time2 += 10;
        delay(10);

    }











}



void driveTurn2(int target) {
double voltage;
double position;
int count = 0;
time2 = 0;
int turnv = 0;

position = imu.get_heading();

if (position > 180){
    position = position-360;
}

if((target < 0) && (position > 0)){
    if((position - target) >= 180){
        target = target + 360;
        position = imu.get_heading();
        turnv = (abs(position) + abs(target));
    }
} else if ((target > 0) && (position < 0)){
    
    if ((target - position) >= 180){
        position = imu.get_heading();
        turnv = abs(abs(position) - abs(target));
    } else {
        turnv = (abs(position) + target);
    }
}

    setConstants(TURN_KP, TURN_KI, TURN_KD);

      int timeout = 30000;

    double x = 0;
    x = double(abs(turnv));
    timeout = (0 * pow(x,5)) +(0 * pow(x,4)) + (0 * pow(x, 3)) + (0 * pow(x, 2)) + (0 * x) + 361.746;
    


    while(true) {

       position = imu.get_heading();

if (position > 180){
    position = position-360;
}

if((target < 0) && (position > 0)){
    if((position - target) >= 180){
        target = target + 360;
        position = imu.get_heading();
        turnv = (abs(position) + abs(target));
    }
} else if ((target > 0) && (position < 0)){
    
    if ((target - position) >= 180){
        position = imu.get_heading();
        turnv = abs(abs(position) - abs(target));
    } else {
        turnv = (abs(position) + target);
    }
}

        voltage = calcPID(target, position,TURN_INTEGRAL_KI, TURN_MAX_INTEGRAL);

        chasMove(voltage, voltage, voltage, -voltage, -voltage, -voltage);

        if (abs(target - position) <= 0.5) count++;
        if (count >= 20 || time2 > timeout){
            break;
        }

         if (time2 % 50 == 0 && time2 % 100 != 0 && time2 % 150 != 0){
            con.print(0, 0, "ERROR: %f      ", float(error));
        } else if (time2 % 100 == 0 && time2 % 150){
            con.print(1, 0, "IMU: %f        ", float(imu.get_heading()));
        } else if (time2 % 150 == 0){
            con.print(2, 0, "Time: %f        ", float(time2));
        }

        time2 += 10;
        delay(10);

    }


}

void driveArcL(double theta, double Radius, int timeout){
    setConstants(STRAIGHT_KP, STRAIGHT_KI, STRAIGHT_KD);


    double ltarget = 0;
    double rtarget = 0;
    double pi = 3.14159265359;
    double init_heading = imu.get_heading();
    int count = 0;
    time2 = 0;
    resetEncoders();
    con.clear();
    ltarget = double((theta / 360) * 2 * pi * Radius);
    rtarget = double((theta / 360) * 2 * pi * (Radius + 400));

    while (true){
        double encoderAvgL = (LF.get_position() + LB.get_position()) / 2;
        double encoderAvgR = (RF.get_position() + RB.get_position()) / 2;
        double leftcorrect = -(encoderAvgL * 360) / (2 * pi * Radius);

        if(init_heading > 180){
            init_heading = init_heading - 360;
        }

        double position = imu.get_heading();

        if (position > 180){
            position = position - 360;
        }

        if((leftcorrect < 0) && (position > 0)){
            if((position - leftcorrect) >= 180){
                leftcorrect = leftcorrect + 360;
                position = imu.get_heading();
            }
        } else if ((leftcorrect > 0) && (position < 0)){
            if((leftcorrect - position) >= 180){
                position = imu.get_heading();
            }
        }

        setConstants(STRAIGHT_KP, STRAIGHT_KI, STRAIGHT_KD);
        int voltageL = calcPID(ltarget, encoderAvgL, STRAIGHT_INTEGRAL_KI, STRAIGHT_MAX_INTEGRAL);
        if(voltageL > 127){
            voltageL = 127;
        } else if (voltageL < -127){
            voltageL = 127;
        }
        int voltageR = calcPID2(rtarget, encoderAvgR, STRAIGHT_INTEGRAL_KI, STRAIGHT_MAX_INTEGRAL);
        if(voltageR > 127){
            voltageR = 127;
        } else if (voltageR < -127){
            voltageR = 127;
        }

         setConstants(ARC_HEADING_KP, ARC_HEADING_INTERGAL_KI, ARC_HEADING_KD);
         int fix = calcPID3((init_heading + leftcorrect), position, ARC_HEADING_INTERGAL_KI, ARC_HEADING_MAX_INTERGAL);

        chasMove((voltageL +fix), (voltageL + fix), (voltageL + fix), (voltageR - fix), (voltageR - fix), (voltageR - fix));
        if ((abs(ltarget - encoderAvgL) <= 4) && (abs(rtarget - encoderAvgR)<= 4)) count++;
        if (count >= 20 || time2 > timeout){
            // break; 
            // ^ off when tuning, on when done.
        }
        if (time2 % 50 == 0 && time2 % 100 != 0 && time2 % 150 != 0){
            con.print(0, 0, "ERROR: %f      ", float(error));
        } else if (time2 % 100 == 0 && time2 % 150){
            con.print(1, 0, "fix: %f        ", float((fix)));
        } else if (time2 % 150 == 0){
            con.print(2, 0, "Time: %f        ", float(time2));
        }

        time2 += 10;
        delay(10);

    }
}



void driveArcR(double theta, double Radius, int timeout){
    setConstants(STRAIGHT_KP, STRAIGHT_KI, STRAIGHT_KD);


    double ltarget = 0;
    double rtarget = 0;
    double pi = 3.14159265359;
    double init_heading = imu.get_heading();
    int count = 0;
    time2 = 0;
    resetEncoders();
    con.clear();
    ltarget = double((theta / 360) * 2 * pi * (Radius + 400) );
    rtarget = double((theta / 360) * 2 * pi * Radius);

    while (true){
        double encoderAvgL = (LF.get_position() + LB.get_position()) / 2;
        double encoderAvgR = (RF.get_position() + RB.get_position()) / 2;
        double rightcorrect = -(encoderAvgL * 360) / (2 * pi * Radius);

        if(init_heading > 180){
            init_heading = init_heading - 360;
        }

        double position = imu.get_heading();

        if (position > 180){
            position = position - 360;
        }

        if(((init_heading + rightcorrect) < 0) && (position > 0)){
            if((position - (init_heading + rightcorrect)) >= 180){
                init_heading = init_heading + 360;
                position = imu.get_heading();
            }
        } else if (((init_heading+ rightcorrect) > 0) && (position < 0)){
            if(((init_heading + rightcorrect) - position) >= 180){
                position = imu.get_heading();
            }
        }

        setConstants(STRAIGHT_KP, STRAIGHT_KI, STRAIGHT_KD);
        int voltageL = calcPID(ltarget, encoderAvgL, STRAIGHT_INTEGRAL_KI, STRAIGHT_MAX_INTEGRAL);
        if(voltageL > 127){
            voltageL = 127;
        } else if (voltageL < -127){
            voltageL = 127;
        }
        int voltageR = calcPID2(rtarget, encoderAvgR, STRAIGHT_INTEGRAL_KI, STRAIGHT_MAX_INTEGRAL);
        if(voltageR > 127){
            voltageR = 127;
        } else if (voltageR < -127){
            voltageR = 127;
        }

         setConstants(ARC_HEADING_KP, ARC_HEADING_INTERGAL_KI, ARC_HEADING_KD);
         int fix = calcPID3((init_heading + rightcorrect), position, ARC_HEADING_INTERGAL_KI, ARC_HEADING_MAX_INTERGAL);

        chasMove((voltageL +fix), (voltageL + fix), (voltageL + fix), (voltageR - fix), (voltageR - fix), (voltageR - fix));
        if ((abs(ltarget - encoderAvgL) <= 4) && (abs(rtarget - encoderAvgR)<= 4)) count++;
        if (count >= 20 || time2 > timeout){
            // break; 
            // ^ off when tuning, on when done.
        }
        if (time2 % 50 == 0 && time2 % 100 != 0 && time2 % 150 != 0){
            con.print(0, 0, "ERROR: %f      ", float(error));
        } else if (time2 % 100 == 0 && time2 % 150){
            con.print(1, 0, "fix: %f        ", float((fix)));
        } else if (time2 % 150 == 0){
            con.print(2, 0, "Time: %f        ", float(time2));
        }

        time2 += 10;
        delay(10);

    }
}
void driveArcLF(double theta, double Radius, int timeout){
    setConstants(STRAIGHT_KP, STRAIGHT_KI, STRAIGHT_KD);

    int trueTheta = theta;
    double ltarget = 0;
    double rtarget = 0;
    double ltargetFinal = 0;
    double rtargetFinal = 0;
    double pi = 3.14159265359;
    double init_heading = imu.get_heading();
    bool over = false;
    int count = 0;
    time2 = 0;
    resetEncoders();
    con.clear();
    
    
    ltargetFinal = double((theta / 360) * 2 * pi * Radius);
    rtargetFinal = double((theta / 360) * 2 * pi * (Radius + 400));
    if(theta > 0){
        theta = theta +45;
    } else {
        theta = theta -45;
    }
    ltarget = double((theta / 360) * 2 * pi * Radius);
    rtarget = double((theta / 360) * 2 * pi * (Radius + 400));

    while (true){
        double encoderAvgL = (LF.get_position() + LB.get_position()) / 2;
        double encoderAvgR = (RF.get_position() + RB.get_position()) / 2;
        double leftcorrect = -(encoderAvgL * 360) / (2 * pi * Radius);

        if(init_heading > 180){
            init_heading = init_heading - 360;
        }

        double position = imu.get_heading();

        if (position > 180){
            position = position - 360;
        }

        if((leftcorrect < 0) && (position > 0)){
            if((position - leftcorrect) >= 180){
                leftcorrect = leftcorrect + 360;
                position = imu.get_heading();
            }
        } else if ((leftcorrect > 0) && (position < 0)){
            if((leftcorrect - position) >= 180){
                position = imu.get_heading();
            }
        }

        setConstants(STRAIGHT_KP, STRAIGHT_KI, STRAIGHT_KD);
        int voltageL = calcPID(ltarget, encoderAvgL, STRAIGHT_INTEGRAL_KI, STRAIGHT_MAX_INTEGRAL);
        if(voltageL > 127){
            voltageL = 127;
        } else if (voltageL < -127){
            voltageL = 127;
        }
        int voltageR = calcPID2(rtarget, encoderAvgR, STRAIGHT_INTEGRAL_KI, STRAIGHT_MAX_INTEGRAL);
        if(voltageR > 127){
            voltageR = 127;
        } else if (voltageR < -127){
            voltageR = 127;
        }

         setConstants(ARC_HEADING_KP, ARC_HEADING_INTERGAL_KI, ARC_HEADING_KD);
         int fix = calcPID3((init_heading + leftcorrect), position, ARC_HEADING_INTERGAL_KI, ARC_HEADING_MAX_INTERGAL);

        chasMove((voltageL +fix), (voltageL + fix), (voltageL + fix), (voltageR - fix), (voltageR - fix), (voltageR - fix));
        // if ((abs(ltarget - encoderAvgL) <= 4) && (abs(rtarget - encoderAvgR)<= 4)) count++;
        // if (count >= 20 || time2 > timeout){
        //     // break; 
        //     // ^ off when tuning, on when done.
        // }

        if(abs((init_heading - position)) > trueTheta){
            over = true;
        }

        if (over || time2 > timeout){
            break;
        }

        if (time2 % 50 == 0 && time2 % 100 != 0 && time2 % 150 != 0){
            con.print(0, 0, "ERROR: %f      ", float(error));
        } else if (time2 % 100 == 0 && time2 % 150){
            con.print(1, 0, "fix: %f        ", float((fix)));
        } else if (time2 % 150 == 0){
            con.print(2, 0, "Time: %f        ", float(time2));
        }

        time2 += 10;
        delay(10);

    }
}
