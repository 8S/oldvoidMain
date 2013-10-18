#include "AcYut.h"
#include "communication.h"
#include "walk.h"
#include "xsens/imu.h"
#include <signal.h>

bool quit = false;

void doquit(int para)
{
	quit = true;
};


float scurve(float in,float fi,float t, float tot)
{
	float frac=(float)t/(float)tot;
	if(frac>1)
	frac=1;
	float retfrac=frac*frac*(3-2*frac);
	float mval=in + (fi-in)*retfrac;
	return mval;
}


float scurve2(float in,float fi,float t, float tot)
{
	float frac=t/tot;
	float ret_frac=6*pow(frac,5)-15*pow(frac,4)+10*pow(frac,3);
	return in+(fi-in)*ret_frac;
}
/*
int balanceStatic(AcYut& bot, int phase, int leg = DSP)
{
	sleep(1);
	const double (&COM)[AXES] = bot.getRotCOM(); 
	//printf("COM X\t%3.3lf\tY\t%3.3lf\tZ\t%3.3lf\n",COM[0],COM[1],COM[2]);
	printf("R\t%3.3lf\tP\t%3.3lf\n",bot.imu->roll,bot.imu->pitch);
	const supportPolygon poly = bot.calcSupportPolygon();
	const Point center = getPolyCentroid(bot.calcSupportPolygon());
	printf("\n\n");
	float error[AXES] = {0};
	float corr[AXES] = {0};
	error[Y] = COM[Y] - center.y;
	error[Z] = COM[Z] - center.x;
	corr[Y] = 0.2 * error[Y];
	corr[Z] = 0.2 * error[Z];
	
	const float (&leftLeg)[AXES+1] = bot.left_leg->getLegCoods();
	const float (&rightLeg)[AXES+1] = bot.right_leg->getLegCoods();
	printf("Left Leg\t%lf\t%lf\t%lf\n",leftLeg[X],leftLeg[Y],leftLeg[Z],leftLeg[4]);
	printf("Right Leg\t%lf\t%lf\t%lf\n",rightLeg[X],rightLeg[Y],rightLeg[Z],rightLeg[4]);
	 
	
	bot.right_leg->runIK(rightLeg[X],rightLeg[Y] + corr[Y],rightLeg[Z],rightLeg[4]);
	bot.updateBot();
	printf("COMY\t%3.3lf\tZ\t%3.3lf\n",COM[1],COM[2]);
	printf("CenY\t%3.3lf\tX\t%3.3lf\n",center.y,center.x);
	printf("CorY\t%3.3lf\tZ\t%3.3lf\n",corr[Y],corr[Z]);
	for(int i=0;i<100;i++);
	printf("\n");
	return 1;
	
}

int kick(AcYut *bot)
{
	float kickTime = 1.0;
	int fps = 80;
	float frameTime	= (float)1.0/(float)fps;
	int sleep = frameTime * 1000000;
	printf("FrameTime\t%lf\n",frameTime);
	float kickHeight = 40;
	float botHeight  = 390;
	float wtShift    = 50;
	float wtShiftTime= 0.5;
	float liftTime   = 0.5;
	float counterBalance = 15;
	
	
	float yStart=0, yrStart=0, zStart=0, zrStart=0;
	float x,xr,y,yr,z,zr;
	for(float time=0;time<wtShiftTime;time+=frameTime)
	{
		x = botHeight;
		xr= botHeight;
		y = yStart;
		yr= yrStart;
		z = scurve2(zStart, wtShift, time, wtShiftTime);
		zr= scurve2(zrStart, -wtShift, time, wtShiftTime);
		printf("Time\t%lf\tX\t%lf\tY\t%lf\tZ\t%lf\tXR\t%lf\tYR\t%lf\tZR\t%lf\n",time,x,y,z,xr,yr,zr);
	
		bot->right_leg->runIK(x,y,z,0);
		bot->left_leg->runIK(xr,yr,zr,0);
		bot->updateBot();
		
		usleep(sleep);
	}
	
	for(float time=0;time<liftTime;time+=frameTime)
	{
		x = scurve2(botHeight, botHeight - kickHeight, time, liftTime);
		xr= botHeight;
		y = yStart;
		yr= yrStart;
		z = scurve2(wtShift,wtShift+counterBalance,time,liftTime);
		zr= scurve2(-wtShift,-wtShift - counterBalance, time, liftTime);
	
		printf("Time\t%lf\tX\t%lf\tY\t%lf\tZ\t%lf\tXR\t%lf\tYR\t%lf\tZR\t%lf\n",time,x,y,z,xr,yr,zr);
		bot->right_leg->runIK(x,y,z,0);
		bot->left_leg->runIK(xr,yr,zr,0);
		bot->updateBot();
		
		usleep(sleep);
		
	} 		
	
};

void test(AcYut *bot)
{

	int xmax = 390;
	int xmin = 360;
	int ymax = 30;
	int ymin = -30;
	float time = 0.5;
	float frameTime = 1.0/60;
	
	float t = 0;
	float x=xmax, y= ymin;
	for(t=0;t<time;t+=frameTime)
	{
		x = linear(xmax,xmin,t,time);
		y = linear(ymin,ymax,t,time);
		bot->right_leg->runIK(x,y,0,0);
		bot->updateBot();
		usleep(16670);
		printf("test x %lf y %lf\n",x,y);
		getchar();
	}
	
	for(t=0;t<time;t+=frameTime)
	{
		x = linear(xmin,xmax,t,time);
		y = linear(ymax,ymin,t,time);
		bot->right_leg->runIK(x,y,0,0);
		bot->updateBot();
		usleep(16670);
		printf("test x %lf y %lf\n",x,y);
		getchar();
	}
	
	
	
	
	

}
*/
int main()
{
	Imu imu;
	imu.init();
	
//	(void) signal(SIGINT,doquit);	
	Communication comm;
	AcYut bot(&comm,&imu);
	Walk walk(&bot);
	
	while(walk.velocity()*1.5 <=200)
		{
			walk.accelerate();
			walk.dribble();
		}
	while(1)
	{
		walk.dribble();
	}
	return 0;
	
	
};



