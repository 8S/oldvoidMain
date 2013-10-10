#include "AcYut.h"
#include "communication.h"
#include "walk.h"
#include "xsens/imu.h"
#include <signal.h>
#include <fstream>

using namespace std;

void* walk_thread(void*)
{
	Communication comm;
	AcYut bot(&comm,&imu);
	Walk walk(&bot);
	
	while(1)
	{

		pthread_mutex_lock(&mutex_walkstr);
		WalkStructure local = walkstr;
		walkstr.isFresh = false;
		pthread_mutex_unlock(&mutex_walkstr);
	
		if(local.isFresh)
		{
			ch = walkstr.instr;
		}
		else
		{
			ch = 'x';
		}
		switch(ch)
		{
			case 'w': 	walk.accelerate();
						break;
			case 'd':	walk.turnright(15);
						continue;
			case 'a':	walk.turnleft(15);
						continue;
			case 'x': 	walk.decelerate();
			 			break;
 			case 'l': 	walk.start();
						continue;
 			default:
			break;
		}
		walk.dribble();
	}
	cvWaitKey();
}

