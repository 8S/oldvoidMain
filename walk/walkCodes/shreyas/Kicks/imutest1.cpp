#include "AcYut.h"
#include "communication.h"
#include "walk.h"
#include "xsens/imu.h"
#include <signal.h>

int main()
{
	Imu imu;
	imu.init();
	
	Communication comm;
	AcYut bot(&comm,&imu);
	
	double t=0;
	int fps=100;
	
	int delay=10000;
	
	while(1)
	{	
		const double (&COM)[AXES] = bot.getRotCOM();
	//	printf("%lf \t %lf \t %lf \n",COM[0],COM[1],COM[2]);
		printf("%lf \t %lf \t %lf \n",imu.roll,imu.pitch,imu.yaw);
	}
	
	return 0;
};
