#ifndef _WALK_H_
#define _WALK_H_

#include "AcYut.h"
#include "xsens/imu.h"
#include <stdlib.h>



enum {SSP,DSP};

class Walk
{
	private:
	AcYut* bot;
	LEG leg;
	int hi ;
	// Y Data //
	double legYin;
	double supLegYin;
	double veloYin;
	double veloYfi;
	
	// Z Data //
	double legZin;
	double supLegZin;
	double veloZin;
	double veloZfi;
	double zMax;
	
	// Turn Data //
	double legRotin;
	double legRotfi;
	double supLegRotin;
	double supLegRotfi;
	
	// X Data //
	double lift;
	
	public:
	Walk(AcYut* bot);
	int kick();
	int dribble();
	int dribble2();
	int start();
	float accelerate();
	float velocity();
	float decelerate();
	float turnleft(float theta);
	float turnright(float theta);
};




#endif
