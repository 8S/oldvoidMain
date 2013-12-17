#ifndef _BOTPROPS_H_
#define _BOTPROPS_H_

enum LEG{LEFT=0,RIGHT=1};
enum AXIS{X,Y,Z};

enum legMotors {FOOT_ROLL=0,FOOT_PITCH,KNEE1,KNEE2,HIP_PITCH,HIP_ROLL,HIP_YAW,LEG_MOTOR_COUNT};
const int legMotorIDs[][7] = {{0,1,2,3,4,5,6},{20,21,22,23,24,25,26}};
const int legDriveMode[][7] = {{0,0,0,0,0,0,0},{1,1,1,1,1,1,1}};
const int legMotorZeros[][7] = {{2048,2048,2048,2048,2048,2048,2048},{2048,2048,2048,2048,2048,2048,2048}};
const int FEET_CORNERS = 4;
const float feetCornerOffset[][FEET_CORNERS][3] ={{{58,107.1/2+65.9,75},{58,107.1/2+65.9,-45},{58,-107.1/2-27,75},{58,-107.1/2-27,-45}},{{58,107.1/2+65.9,75},{58,107.1/2+65.9,-45},{58,-107.1/2-27,75},{58,-107.1/2-27,-45}}};
const int LEG_SLAVE_COUNT = 2;
const int legSlaveMotors[] = {FOOT_PITCH,HIP_PITCH};

const double LengthLink1 = 170.0;
const double LengthLink2 = 69.5;
const double LengthLink3 = 170.0;
const double hipLength = 130.0;

const double delta=4096.0/360.0;
	
const double legHeight = LengthLink1+LengthLink2+LengthLink3;	

enum handMotors{SHOULDER, HAND_MOTOR_COUNT};




#endif
