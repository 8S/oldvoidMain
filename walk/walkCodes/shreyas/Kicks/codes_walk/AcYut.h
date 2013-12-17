#ifndef _ACYUT_H_
#define _ACYUT_H_

#include "commondefswalk.h"
#include "communication.h"
#include "leg.h"
#include "motor.h"
#include "hand.h"
#include <time.h>
#include <stdio.h>
#include <iostream>
#include "xsens/imu.h"

#define ACYUT_MOTOR 31

typedef pointArray supportPolygon;

class AcYut
{

private:
	int offsets[ACYUT_MOTOR];
	Communication* comm;
	double COM[AXES];
	double rotCOM[AXES];
	static const double legHeight = 390;
	supportPolygon poly;
	int polyPoints;
	double feetCoods[2][4][AXES];
	
public:
	Imu* imu;
	Leg *right_leg, *left_leg;
	Leg* leg[2];
	Hand *right_hand, *left_hand;
	
	AcYut(Communication* comm, Imu* imu);
	AcYut(Communication* comm);
	~AcYut();
	void initialize();
	int updateBot();
	int pingBot();
	
	double* getWorldFrameCoods(double coods[AXES],double ans[AXES]);
	int getFeetCoods(int leg);
	int getFeetCoods();
	const double (&(getCOM())) [AXES];
	const double (&(getRotCOM()))[AXES];
	const supportPolygon calcSupportPolygon();
	int clearPoS();
	int storeCOM2(int n);
	int printCOM();
	int printRotCOM();
	int writeSensedCurrent(FILE *&fp);
};


#endif
