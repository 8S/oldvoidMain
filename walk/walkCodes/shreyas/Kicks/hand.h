#ifndef HANDH
#define HANDH
#include "motor.h"
#include "communication.h"
#include "commondefswalk.h"

#include <stdio.h>
#include <math.h>
#define HAND_MOTORS_COUNT 4

class Hand
{
private:
	Communication *comm;
	Motor* m[HAND_MOTORS_COUNT];

	int hand;
public:
	Hand(int hand, Communication* comm);
	void setIDList(int* idList, int n);
	int setSpeed(int speed);
	int getLoad();
	int* pingHand();
	int getMotorLoad(int motorID);
	void init();
	void setGoalPositionSync(int* pos);
};

#endif
