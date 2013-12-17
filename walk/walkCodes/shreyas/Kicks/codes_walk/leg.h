#ifndef _LEG_H
#define _LEG_H
#include "motor.h"
#include "communication.h"
#include "commondefswalk.h"
#include <stdio.h>
#include <math.h>

class Leg
{

private:
	
	Communication* comm;
	Motor* m[LEG_MOTOR_COUNT];
	
	float legCoods[AXES+1];
	float x;
	float y;
	float z;
	float theta;
	
	double feetCoods[FEET_CORNERS][AXES];
	double jointCoods[LEG_MOTOR_COUNT][AXES];
	double COM[AXES];

public:

	int leg;
	Leg(int leg, Communication* comm, int ids[], int offsets[], int driveMode[], int zeros[]);
	~Leg();
	void setIDList(int* idList, int n);
	bool* pingLeg(bool pingStatus[]);
	int pingLeg();
	int pingMotor(int id);
	int getLoad();
	int getMotorLoad(int motorID);
	int setMotorGainP(int p, int motorSelect);
	int motorLedOn(int id);
	int motorAddChange(int id, int change);
	
	int syncLeg();
	int setGoalPositionSync();
	int setSpeed(int speed);
	int setMotorSpeed(int id, int speed);
	
	const double (&(getFeetCoods()))[FEET_CORNERS][AXES];
	int runIK(double x, double y, double z, double t4);
	const float (&(getLegCoods())) [AXES+1];  
	const double (&getLegCOM()) [AXES];
	const double (&runFK(Motor* mot[])) [LEG_MOTOR_COUNT][AXES];
	const double (&runFK()) [LEG_MOTOR_COUNT][AXES];
	
	int GP(int id);
	double*  getSensedCurrentAmps();	
};
#endif
