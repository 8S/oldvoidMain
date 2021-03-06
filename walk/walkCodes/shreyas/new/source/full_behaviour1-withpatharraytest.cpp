#include <stdio.h>
#include <math.h>
#include <sys/time.h>
#include <unistd.h>
#include <stdlib.h>
#include <inttypes.h>
#include <string.h>  /* String function definitions */
#include <fcntl.h>   /* File control definitions */
#include <errno.h>   /* Error number definitions */
#include <termios.h> /* POSIX terminal control definitions */
#include <time.h>
#include <ftdi.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <pthread.h>
#include <fstream>
#include <errno.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <netdb.h>

#include <pthread.h>
#include <string.h>
//path as it was on 21-9-2012(no initial orientation- only obs avoidance and final orientation) will only work on testbot

#define GOALKEEPER 0//keep value as 0 for normal bot

//#define TESTINGBOT
//#define GAMECONTROLLERACTIVE
// #define SWITCHACTIVE
#define WIFITRANSMISSION
#include "commondefs.h"
#include "headmotor.h"
#include "camcapture.h"
#include "camcontrol.h"
#include "featuredetection.h"
#include "localize.h"
#include "defines.h"
#include "path.hpp"

// #ifndef TESTINGBOT
// #include "walk_thread.cpp"
// #endif

#ifdef SWITCHACTIVE
#include "acyut_gpio_func.cpp"
#endif

#ifdef GAMECONTROLLERACTIVE
#include "gamecontrollerfunc.h"
#include "gamecontrollerobj.h"
#endif

#ifdef TESTINGBOT
#include "testbot.c"
#endif




//TYPE DEFINITIONS
//Also see commondefs.h

enum WalkCommands {INIT,WALK,ENCIRCLE,GETUPFRONT,GETUPBACK,KICK};//behaviour will calculate r and theta from path and continue

typedef struct
{
	float r;
	float theta;
	float encircleAngle;
	char instr;
	MotionModel mm;
} walkstructure;


enum Goal{BLUEGOAL=0,YELLOWGOAL=1};

//OTHER DEFINITIONS
#define NO_OF_FRAMES_TO_WAIT_B4_KICKOFF 50
#define MINenAngle 15 //minimum angle for encircling

//For GOALKEEPER//values are set after testing//keep testing...keep updating... atleast after every major change(in IPcodes/motor/camera/acyutsystem)
#define GKFALLDISTTHRESHOLD 50//cm//the distance after which it is critical to save, so if velocity>didntmovevel then dive
#define GKMINFRAMES2GO 20//frames//minimum frames remaining for initiating dive, if the no of frames for ball to come close(ballR/velocity) is less than this, initiate dive
#define MINBALLANGLE4FALL 15//degrees//the angle above which dive will be towards right,below -(thisangle) it will be towards left
#define DIDNOTMOVEVELOCITY 1// cm/frame //velocity threshold,below this it will velocity, the ball is as good as still

//distances in cm

#define LM_PREVFACT 0.5
#define LM_WT 0.72165
#define B_PREVFACT 0.5
#define B_WT 1.0
#define B_WT2 1
#define TH 0.6
#define TL 0.03

//VARIABLE DECLARATIONS
//State currentstate=INITIAL;
BallReturns b;
GoalReturns g;
WalkCommands bodycommand;
Flags globalflags;
Goal mygoal=YELLOWGOAL;//0=blue,1=yellow
//int flag_encircle;
int Flag_moving=0;//0 if not walking
int Flag_moving_local;//for walkthread
int Flag_moving_mainlocal;//for main

pthread_mutex_t mutex_walkflag=PTHREAD_MUTEX_INITIALIZER;
//pthread_mutex_t mutex_encircleflag=PTHREAD_MUTEX_INITIALIZER;
pthread_mutex_t mutex_walkstr=PTHREAD_MUTEX_INITIALIZER;
pthread_mutex_t mutex_bodycommand=PTHREAD_MUTEX_INITIALIZER;
pthread_mutex_t mutex_changewalkflag=PTHREAD_MUTEX_INITIALIZER;
walkstructure walkstr;
walkstructure prevwalkstr;
CamCapture capture;
FeatureDetection fd(capture);
HeadMotor hdmtr;
CamControl camcont(capture);
Localize loc;
float bwt,lmwt;
int ballnotinframe;
MotionModel localmm;
double conf;
PathStructure pathstr;
Path pathobj;
float prevballr,velocity;

//WALK RELATED
int global_CHANGEWALKFLAG;

#ifdef GAMECONTROLLERACTIVE
GameControllerAcyut GC;
#endif

#ifdef WIFITRANSMISSION
#include "wificommunication.cpp"
#endif


inline void stopWalk()
{
	pthread_mutex_lock(&mutex_walkflag);
	Flag_moving=0;
	pthread_mutex_unlock(&mutex_walkflag);
}
//int walk3(int leg,float yin,float yfi,float yrin,float yrfi, float zin, float zfi,float zrin, float zrfi, float thetain, float thetafi,float thetarin, float thetarfi) //walk charli

#ifndef TESTINGBOT
#define MINTHETA 15.0//degrees
#define STEPLEN 15//cm

//

//TODO: Resolve how to do Motion Model for Acyut Encircle

//
int callEncircle()
{
	printf("Supposed to Encircle\n");
}

#endif


#ifdef TESTINGBOT
#include "testbotfunctions.cpp"
#endif
#ifndef TESTINGBOT
WalkCommands localbodycommand;
walkstructure localwalkstr;

void read_walk_data()
{
	pthread_mutex_lock(&mutex_walkflag);
	Flag_moving_local=Flag_moving;
	pthread_mutex_unlock(&mutex_walkflag);

	pthread_mutex_lock(&mutex_bodycommand);
	localbodycommand=bodycommand;
	pthread_mutex_unlock(&mutex_bodycommand);

	pthread_mutex_lock(&mutex_walkstr);
	localwalkstr=walkstr;
	prevwalkstr=walkstr;
	walkstr.r=0;
	walkstr.theta=0;
	pthread_mutex_unlock(&mutex_walkstr);
}
#include "walk_thread.cpp"
// void* walk_thread(void*)
// {
// 	//printf("Call_walk working1\n");
// 	double r,theta,theta1;
// 	float *state= endState;

// 	state[YR]=lYin;
//  	state[Y]=slYin;
//  	state[YV]=vYin;
//  	state[ZR]=lZin;
//  	state[Z]=slZin;
//  	state[ZV]=-vZin;
// 	int flag=0;
// 	int tempFlag = 0;

// 	while(TRUE)
// 	{

		
// 		tempFlag++;
// 		read_walk_data();
		
// 		// pthread_mutex_lock(&mutex_walkflag);
// 		// Flag_moving_local=Flag_moving;
// 		// pthread_mutex_unlock(&mutex_walkflag);

// 		r=localwalkstr.r;
// 		theta=localwalkstr.theta;
// 		//printf("[walk] running\n");

// 		//check for init/walk haere
// 		//if(localbodycommand==WALK){then walk} else if ==INIT initialise and  readwait till flagmoving is one
// 		if(Flag_moving_local==1)
// 		{
			
// 			if(localbodycommand==WALK)
// 			{	
// 				printf("``````````````````````````````````````~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~`````````````````````````````````\n[Walk] Received Command WALK r=%lf theta=%lf\n",localwalkstr.r,localwalkstr.theta);
// 				theta1=abs(theta);
// 				float  r1=r;
// 				float x=0;
// 				float turn_dist1=0,turn_dist2=0;
// 				float walk_dist1=0,walk_dist2=0;
// 				// state = dribble(flag,state[YR],state[Y],state[YV],state[YV],state[ZR],state[Z],-state[ZV],state[ZV],0,0,0,0);
// 				// x=x+state[Y]-state[YR];
// 				// flag=1-flag;
// 				// state = dribble(flag,state[YR],state[Y],state[YV],state[YV],state[ZR],state[Z],-state[ZV],state[ZV],0,0,0,0);
// 				// x=x+state[Y]-state[YR];
// 				// flag=1-flag;
// 				if(theta<0&&flag==1)
// 				{
// 				state = dribble(flag,state[YR],state[Y],state[YV],state[YV],state[ZR],state[Z],-state[ZV],state[ZV],0,0,0,0);
// 				x=x+state[Y]-state[YR];
// 				flag=1-flag;
// 				}
// 				else if(theta>0&&flag==0)
// 				{
// 				state = dribble(flag,state[YR],state[Y],state[YV],state[YV],state[ZR],state[Z],-state[ZV],state[ZV],0,0,0,0);
// 				x=x+state[Y]-state[YR];
// 				flag=1-flag;	
// 				}
// 				while(theta1>20)
// 				{
// 					if(theta>0)
// 					{
// 					state = dribble(flag,state[YR],state[Y],state[YV],vTTar,state[ZR],state[Z],-state[ZV],state[ZV],0,20,0,0);
// 					flag=1-flag;
// 					x=x+state[Y]-state[YR];
// 					turn_dist1=state[Y]-state[YR];
// 					state = dribble(flag,state[YR],state[Y],state[YV],vTTar,state[ZR],state[Z],-state[ZV],state[ZV],state[RR],0,state[R],0);
// 					flag=1-flag;
// 					x=x+state[Y]-state[YR];
// 					turn_dist2=state[Y]-state[YR];
// 					}
// 					else if(theta<0)
// 					{
// 					state = dribble(flag,state[YR],state[Y],state[YV],vTTar,state[ZR],state[Z],-state[ZV],state[ZV],0,20,0,0);
// 					flag=1-flag;
// 					x=x+state[Y]-state[YR];
// 					turn_dist1=state[Y]-state[YR];
// 					state = dribble(flag,state[YR],state[Y],state[YV],vTTar,state[ZR],state[Z],-state[ZV],state[ZV],state[RR],0,state[R],0);
// 					flag=1-flag;
// 					x=x+state[Y]-state[YR];
// 					turn_dist2=state[Y]-state[YR];
// 					}
// 					theta1=theta1-20;
// 					pthread_mutex_lock(&mutex_walkstr);
// 					walkstr.mm.theta=(theta>0?1:-1)*20;
// 					walkstr.mm.r=turn_dist1+turn_dist2;
// 					walkstr.mm.updated=1;
// 					pthread_mutex_unlock(&mutex_walkstr);
// 				}

// 				if(theta>0)
// 				{
// 					state = dribble(flag,state[YR],state[Y],state[YV],vTTar,state[ZR],state[Z],-state[ZV],state[ZV],0,theta1,0,0);
// 					flag=1-flag;
// 					x=x+state[Y]-state[YR];
// 					turn_dist1=state[Y]-state[YR];
// 					state = dribble(flag,state[YR],state[Y],state[YV],vTTar,state[ZR],state[Z],-state[ZV],state[ZV],state[RR],0,state[R],0);
// 					flag=1-flag;
// 					x=x+state[Y]-state[YR];
// 					turn_dist2=state[Y]-state[YR];
// 					pthread_mutex_lock(&mutex_walkstr);
// 					walkstr.mm.theta=(theta>0?1:-1)*20;
// 					walkstr.mm.r=turn_dist1+turn_dist2;
// 					walkstr.mm.updated=1;
// 					pthread_mutex_unlock(&mutex_walkstr);
// 				}
// 				else if(theta<0)
// 				{
// 					state = dribble(flag,state[YR],state[Y],state[YV],vTTar,state[ZR],state[Z],-state[ZV],state[ZV],0,theta1,0,0);
// 					flag=1-flag;
// 					x=x+state[Y]-state[YR];
// 					turn_dist1=state[Y]-state[YR];
// 					state = dribble(flag,state[YR],state[Y],state[YV],vTTar,state[ZR],state[Z],-state[ZV],state[ZV],state[RR],0,state[R],0);
// 					flag=1-flag;
// 					x=x+state[Y]-state[YR];
// 					turn_dist2=state[Y]-state[YR];
// 					pthread_mutex_lock(&mutex_walkstr);
// 					walkstr.mm.theta=(theta>0?1:-1)*20;
// 					walkstr.mm.r=turn_dist1+turn_dist2;
// 					walkstr.mm.updated=1;
// 					pthread_mutex_unlock(&mutex_walkstr);
// 				}

// 				while(x<r1)
// 				{
// 					state = dribble(flag,state[YR],state[Y],state[YV],vTTar,state[ZR],state[Z],-state[ZV],state[ZV],0,0,0,0);
// 					x=x+state[Y]-state[YR];
// 					flag=1-flag;
// 					walk_dist1=state[Y]-state[YR];

// 					state = dribble(flag,state[YR],state[Y],state[YV],vTTar,state[ZR],state[Z],-state[ZV],state[ZV],state[RR],0,state[R],0);
// 					x=x+state[Y]-state[YR];
// 					flag=1-flag;
// 					walk_dist2=state[Y]-state[YR];
// 					//Return Motion Model data
					
// 					pthread_mutex_lock(&mutex_walkstr);
// 					walkstr.mm.r=walk_dist1+walk_dist2;
// 					walkstr.mm.updated=1;
// 					pthread_mutex_unlock(&mutex_walkstr);

// 					int breakflag=0;
				
// 					pthread_mutex_lock(&mutex_changewalkflag);
// 					if(global_CHANGEWALKFLAG==1)
// 					{
// 						//endwalk(flag,state[YR],state[Y],state[ZR],state[Z]);
// 						global_CHANGEWALKFLAG=0;
// 						breakflag=1;	
// 					}
// 					pthread_mutex_unlock(&mutex_changewalkflag);
// 					if(breakflag==1){break;}
// 					printf("!!!!!!!!!!!!!!! x = %lf \n\n\n\n\n\n\n\n", x);

					
// 				}
// 				stopWalk();
// 			}//if command==WALK
// 			else if(localbodycommand==INIT)
// 			{
// 				initialize_acyut();
// 				stopWalk();
// 			}
// 		}
// 		else
// 		{
// 			usleep(20000);
// 			continue;
// 		}
// 	}//while ends
// 	printf("***\n");
// 	cvWaitKey();
// }	

#endif


#ifdef SWITCHACTIVE
void check_switch()
{
	int localswitch1;
	pthread_mutex_lock(&mutex_switch);
	localswitch1=switch1;
	pthread_mutex_unlock(&mutex_switch);

	if(localswitch1!=1)
	{
		printf("[behaviour]  Switch is OFF\n");
		exit(0);
	}
	printf("[behaviour]  Switch is On\n");
}
#endif

inline void updateWalkFlag()
{
	pthread_mutex_lock(&mutex_walkflag);
	Flag_moving_mainlocal=Flag_moving;
	pthread_mutex_unlock(&mutex_walkflag);
}

void updatewt(double ballratio)
{
	lmwt=LM_WT*(1-conf);//lmwt should be greater
	bwt=B_WT*(ballratio);//*(1-ballnotinframe);
	
}

#define STEP_WALKBEHAV 10
#define STEP_TURNBEHAV 7.0

double getCapped(double var,double threshold)
{
	if(var>=threshold)
		return threshold;
	else
		return var;
}

void moveAcYut(double r,double theta,double enc_angle,WalkCommands com,Coords *orientArray=NULL)//This Function sends data to walk thread using mutexlocks
{
	pthread_mutex_lock(&mutex_walkflag);
	int localflagmoving=Flag_moving;
	pthread_mutex_unlock(&mutex_walkflag);
	printf("\n[behaviour] Called moveAcYut r %lf theta %lf enc_angle %lf WalkCommand %d\n_____________________________",r,theta,enc_angle,com);
	// if(localflagmoving==0)
	{    
		printf("Move acyut flag=0\n");
		if(com==WALK)
		{
			pthread_mutex_lock(&mutex_walkstr);
			if(fabs(theta) > 15.0)
			{
				if(theta < 0)
					walkstr.instr = 'a';
				else
					walkstr.instr = 'd';
			}
			else
			{
				if(r < 20.0)
					walkstr.instr = 's';
				else
					walkstr.instr = 'w';
			}
			

			printf("~~~Command sent is %c\n", walkstr.instr);
			walkstr.r=getCapped(r, STEP_WALKBEHAV)*10.0;
			walkstr.theta=theta;
			walkstr.encircleAngle=0;
			pthread_mutex_unlock(&mutex_walkstr);
		}
		else if(com==ENCIRCLE)
		{
			pthread_mutex_lock(&mutex_walkstr);
			if(enc_angle>=10)
			walkstr.instr='q';//left
			else if(enc_angle<=-10)
			walkstr.instr='e';//right
			pthread_mutex_unlock(&mutex_walkstr);
		}
		else if(com==KICK)
		{
			pthread_mutex_lock(&mutex_walkstr);
			walkstr.instr='x';
			pthread_mutex_unlock(&mutex_walkstr);
		}
		else if(com==ORIENT)
		{
			pthread_mutex_lock(&mutex_walkstr);
			walkstr.orientArray=orientArray;
			pthread_mutex_unlock(&mutex_walkstr);
		}
		pthread_mutex_lock(&mutex_bodycommand);
		bodycommand=com;
		pthread_mutex_unlock(&mutex_bodycommand);

		pthread_mutex_lock(&mutex_walkflag);
		Flag_moving=1;
		pthread_mutex_unlock(&mutex_walkflag);
	}
}

void initiateDive()
{
	if(fd.ball.theta>MINBALLANGLE4FALL)
	{
		//FALLRIGHT
		printf("[behaviour] FALLRIGHT\n");
		cvWaitKey();
	}
	else if(fd.ball.theta<-MINBALLANGLE4FALL)
	{
		//FALLLEFT
		printf("[behaviour] FALLLEFT\n");
		cvWaitKey();
	}
}
void convertPathData()
{
	//Path uses a coordinate system which rotates along with acyut.
	//in that coordinate system, acyut always faces positive X axis and **+Y axis lies to its right**
	pathstr.n_obstacles=fd.o.size();
	for(int i=0;i<fd.o.size();i++)//assuming right positive and left neagative for theta
	{
		pathstr.absObstacles[i].x=(fd.o[i].distance)*cos(deg2rad(fd.o[i].angle));
		pathstr.absObstacles[i].y=(fd.o[i].distance)*sin(deg2rad(fd.o[i].angle));
	}

	for (int i = 0; i < pathstr.n_obstacles; ++i)
	{
		printf("Sent To Path: obstacle %d : %lf %lf\n", i, pathstr.absObstacles[i].x, pathstr.absObstacles[i].y);
	}
	AbsCoords goalcoords=loc.getGoalCoords(mygoal);
	double tempx=goalcoords.x-loc.selfX;
	double tempy=goalcoords.y-loc.selfY;
	pathstr.goal.x= (tempx*cos(deg2rad(loc.selfAngle))) - (tempy* sin(deg2rad(loc.selfAngle)));//Rotating coordinate system.
	pathstr.goal.y= (tempx*sin(deg2rad(loc.selfAngle))) + (tempy* cos(deg2rad(loc.selfAngle)));
	printf("Sent To Path:goal coords x:%lf  y:%lf\n",pathstr.goal.x,pathstr.goal.y);

	//printf("goal coords y:%lf\n",pathstr.goal.x);
	pathstr.ball.x=fd.ball.r*cos(deg2rad(fd.ball.theta));
	pathstr.ball.y=fd.ball.r*sin(deg2rad(fd.ball.theta));
	printf("Relative ball:R=%f  Theta=%f\n",fd.ball.r,fd.ball.theta);
	printf("Sent To Path:ball coords x:%lf  y:%lf\n",pathstr.ball.x,pathstr.ball.y);
}

int main()
{
	//bootup
	//initialise
	int kickofftimecount=0;

	#ifndef TESTINGBOT
		//bootup_files_walk();
		//initialize_acyut();
	#endif

	#ifdef TESTINGBOT
		testbot_bootup();
	#endif
	//create threads

	#ifdef GAMECONTROLLERACTIVE
	GameControllerAcyut GC;
	pthread_t thread_id_gc;
    pthread_create (&thread_id_gc, NULL, readGameController, NULL);//function defined in its appropriate header file
	RoboCupGameControlData localgcdata;
	#endif

	pthread_t thread_id_walkthread;
	pthread_t thread_id_wifi;
	#ifdef 	SWITCHACTIVE
    void *statusgpio;
    pthread_t thread_id_gpio;
    pthread_create(&thread_id_gpio,NULL,switchupdate,NULL);//function defined in its appropriate header file
	
	led(3,1);
	led(4,0);


	while(1)//Master while
	#endif
	{
		#ifdef 	SWITCHACTIVE
		check_switch();//if on, does nothing, if off, exits program
		#endif
		#ifdef WIFITRANSMISSION
		pthread_create(&thread_id_wifi,NULL,main_wifitransmission,NULL);
		#endif
		pthread_create (&thread_id_walkthread, NULL, walk_thread, NULL);
		printf("[behaviour]  WALKTHREAD CREATED\n");
		hdmtr.bootup_files();
		capture.init();
		globalflags.reset();
		int nolocframecount=0;
		int landmarkstate1tries=0;
		
		//INITIALISE---------------------------------------
		// pthread_mutex_lock(&mutex_bodycommand);
		// bodycommand=INIT;
		// pthread_mutex_unlock(&mutex_bodycommand);
		// pthread_mutex_lock(&mutex_walkflag);
		// Flag_moving=1;
		// pthread_mutex_unlock(&mutex_walkflag);
		sleep(2);
		//INITIALISE--------------------------------------
		prevballr=0;

		while(1)//main Behaviour starts
		{
			printf("\n~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~\n");

			loc.printPosition();
			hdmtr.update();
			capture.getImage();
			cvShowImage("Image",capture.rgbimg);
			cvShowImage("Localization", loc.dispImage);
			cvWaitKey(5);
			#ifdef SWITCHACTIVE
			check_switch();
			#endif
			//CHECK IMU DATA HERE AND IF FALLEN SET FALLEN FLAG TO NO/FRONT/BACK
			// char temp = cvWaitKey(5)&0xff;
			// switch(temp)
			// {
			// 	case 'w': 

			// 	moveAcYut(100.0, 0.0, 0, WALK);
			// 	break;
				
			// 	case 'a':
			// 	moveAcYut(0.0, -18.0, 0, WALK);
			// 	break;
				
			// 	case 'd':
			// 	moveAcYut(0.0, 18.0, 0, WALK);
			// 	break;

			// 	case 's':
			// 	moveAcYut(0.0, 0.0, 0, WALK);
			// 	break;

			// 	case 'q':
			// 	moveAcYut(0,0,20,ENCIRCLE);
			// 	break;

			// 	case 'e':
			// 	moveAcYut(0,0,-20,ENCIRCLE);
			// 	break;
			// }
			//continue;
			if(GOALKEEPER==0)
			{
				if(globalflags.fallen==FRONT)
				{
					//FRONT GETUP
				}
				else if(globalflags.fallen==BACK)
				{
					//BACK GETUP
				}
			}
			else if(GOALKEEPER==1)
			{
				
			}

			#ifdef GAMECONTROLLERACTIVE
			pthread_mutex_lock(&mutex_GCData);
			localgcdata=GCData;
			pthread_mutex_unlock(&mutex_GCData);

			GC.update(localgcdata);
			int teamindex;
			teamindex=GC.getTeamIndex();

			//mygoal=GC.updategoal(teamindex);
			switch(localgcdata.state)//checking gamecontroller
			{
				case STATE_READY:printf("[behaviour] currentGCstate=READY");

								continue;

				case STATE_PLAYING:break;

				default:printf("[behaviour]  Gamecontroller: No PLAY signal yet\n");
						stopWalk();
						#ifndef TESTINGBOT
							//INITIALISE---------------------------------------
							pthread_mutex_lock(&mutex_bodycommand);
							bodycommand=INIT;
							pthread_mutex_unlock(&mutex_bodycommand);
							pthread_mutex_lock(&mutex_walkflag);
							Flag_moving=1;
							pthread_mutex_unlock(&mutex_walkflag);
							sleep(2);
							//INITIALISE--------------------------------------
						#endif
						continue;
			}
			if(GOALKEEPER==0)
			{
				if(GC.isMyKickOff())
				{
					printf("its my kickoff!!! :D \n\n%d\n\n",localgcdata.teams[localgcdata.kickOffTeam].teamNumber);
					//continue normally if not then dont walk for some time
				}
				else
				{
					printf("not my kickoff :/ \n%d\n\n",localgcdata.teams[localgcdata.kickOffTeam].teamNumber);
					//wait
				}

				if(localgcdata.teams[localgcdata.kickOffTeam].teamNumber!=myteamnumber)
				{
					kickofftimecount++;
					if(kickofftimecount<NO_OF_FRAMES_TO_WAIT_B4_KICKOFF)
					{
						printf("[behaviour] WALK stopped because of opponents KICKOFF\n\n");
						stopWalk();
					}
				}
			}
			#endif

			updateWalkFlag();
			
			//will read mm values only if updated
			pthread_mutex_lock(&mutex_walkstr);
			// localmm=walkstr.mm;
			
			printf("Motion model updated=%d===----------------------------\n",walkstr.mm.updated);

			fd.getLandmarks(capture, hdmtr, walkstr.mm);
			loc.doLocalize(fd,walkstr.mm);
			conf = loc.confidence();
			globalflags.LOCALISED=conf;
			ballnotinframe=!(fd.ballFound());

			walkstr.mm.updated=0;
			pthread_mutex_unlock(&mutex_walkstr);

			printf("[behaviour] confidence recieved = %lf\n",conf);

			// continue;
			//WEiGHTAGE FUNcTION ----------------------------------------------------------------------------------------------------------------
			updatewt(fd.ballRatio);
			
			if(globalflags.LocalizationState==0)
			{
				if(conf<TH)
				{
					camcont.search(hdmtr);
					printf("[behaviour] Looking for LANDMARKS***** State=0\n\n");
					nolocframecount++;
					if(nolocframecount>20000)//if framecount exceeds 200 then turn a little and search for more land marks
					{
						moveAcYut(0,45,0,WALK);
						nolocframecount=0;
					}
					globalflags.p=DONTGENERATE;
					globalflags.b=NOTFOUND;

				}
				else
					globalflags.LocalizationState=1;
			}
			if(globalflags.LocalizationState==1)
			{
				if(conf<TL)
				{
					globalflags.LocalizationState=0;
					continue;	//path might be made if continue not there
				}
				else
				{
					printf("[behaviour] BALL FINDING*****\n\n");
					globalflags.p=DONTGENERATE;
					// if(bwt>lmwt)
					// {
						//printf("[behaviour] called findBall\n\n");
						int ballreturn=camcont.findBall(fd,hdmtr);
						switch(ballreturn)
						{
							case BALLFOUND:
											ballnotinframe=0;
											globalflags.b=FOUND;
											// if(fd.ball.r<BALLDIST_THRESHOLD)
											// {
											// 	globalflags.b=FOUNDNEAR;
											// 	printf("FOUNDNEAR %lf\n",fd.ball.r);
											// }
											// else
											// {
											// 	globalflags.b=FOUNDFAR;
											// 	printf("FOUNDFAR %lf\n",fd.ball.r);
											// }
											break;
							case TURNRIGHT:
											ballnotinframe=1;
											globalflags.b=NOTFOUND;
											//SEND TURN COMMAND
											// moveAcYut(0,30,0,WALK);
											break;
							case TURNLEFT:
											ballnotinframe=1;
											globalflags.b=NOTFOUND;
											//SEND TURN COMMAND
											moveAcYut(0,-30,0,WALK);
											break;
							case BALLFINDING:
											ballnotinframe=1;
											globalflags.b=NOTFOUND;
											continue;
							default:continue;

						}
					// }
					// else
					// {
					// 	if(conf<TH)
					// 	{
					// 		camcont.search(hdmtr);
					// 		printf("[behaviour] Looking for LANDMARKS***** State=1\n\n");
					// 		landmarkstate1tries++;
					// 	}
					// 	else if(landmarkstate1tries>200)
					// 	{
					// 		landmarkstate1tries=0;
					// 		globalflags.LocalizationState=0;

					// 	}
					// }
				}
			}


			if(GOALKEEPER==0)
			{
				if(globalflags.b==FOUND)
				{
					globalflags.p=GENERATE;
				}

				#ifdef TESTINGBOT
				if(Flag_moving_mainlocal==0)//not moving
				#endif
				{

					printf("[behaviour] Not Moving...\n");
					pthread_mutex_lock(&mutex_walkstr);
					localmm=walkstr.mm;
					pthread_mutex_unlock(&mutex_walkstr);

					if(globalflags.p==GENERATE)
					{
						//callpath
						printf("[behaviour] CALLING PATH_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-\n\n");//CALLING PATH!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
						convertPathData();
						PathReturns pathreturnval=pathobj.path_return(pathstr);
						switch(pathreturnval)
						{
							case DOWALK:
										printf("[behaviour]-------Path Returns DOWALK--------\n");
										moveAcYut(pathobj.next.r,pathobj.next.theta,0,WALK);//REMEMBER Pathobj.next.theta has the opposite convention  hence the MINUS sign
										//moveAcYut(path.r,path.theta,0,WALK);
										// moveAcYut(fd.ball.r,fd.ball.theta,0,WALK);
										//cvWaitKey();
										break;
							case DOENCIRCLE:
										printf("[behaviour]-------Path Returns DOENCIRCLE--------\n");
										moveAcYut(0,0,pathobj.NextEncircleAngle,ENCIRCLE);
										//cvWaitKey();
										break;
							case DOKICK:
										printf("[behaviour]-------Path Returns DOKICK--------\n");
										moveAcYut(0,0,0,KICK);
										//ksjdklfdskljsdkfskldfjsldjflkdsjfkdsjfskjfldssystem("sh mario.sh");
										//cvWaitKey();
										//globalflags.reset();
										//This will pause the but when it is ready to kick. to change this, comment cvwaitkey and uncomment globalflags.reset
										//KICKING
										break;
							case DOORIENT:
										printf("[behaviour]-------Path Returns DOORIENT--------\n");
										
										moveAcYut(0,0,0,ORIENT,pathobj.OrientArray);
										break;
							case NOPATH:
										printf("[behaviour]-------Path Returns NOPATH--------\n");
										// moveAcYut(fd.ball.r,fd.ball.theta,0,WALK);//REMEMBER Pathobj.next.theta has the opposite convention  hence the MINUS sign
										//moveAcYut(path.r,path.theta,0,WALK);
										// moveAcYut(fd.ball.r,fd.ball.theta,0,WALK);
										//cvWaitKey();
										break;
							default: printf("What is happening?\n");

						}
						//printf("Preparing to walk %d\n",pathreturnval);
					}
					else
					{
						//do something
						pthread_mutex_lock(&mutex_walkstr);
						walkstr.instr = 's';
						pthread_mutex_unlock(&mutex_walkstr);
						//continue;
					}
				}
				//#ifdef WIFITRANSMISSION


				usleep(10000);
				printf("Called convert wifidata\n");

				#ifdef WIFITRANSMISSION
				convertTransmitData();
				#endif
			}
			if(GOALKEEPER==1)
			{
				//ball is found and localised
				if(globalflags.b==FOUND)
				{
					float balldist=fd.ball.r*cos(deg2rad(fd.ball.theta));
					if(prevballr==0)
					{
						prevballr=balldist;
						continue;
					}
					velocity=prevballr-balldist;
					prevballr=balldist;
					printf("[behaviour] Velocity(cm/frame) = %lf\nBall X=%f theta=%f\nprevballr=%f\n",velocity,balldist,fd.ball.theta,prevballr);
					if(velocity<-DIDNOTMOVEVELOCITY)
					{
						//ball is going away, good :D
					}
					if(balldist>GKFALLDISTTHRESHOLD)
					{
						if((balldist/velocity)<GKMINFRAMES2GO)//if no of frames left for ball to come close is less
						{
							initiateDive();
						}

					}
					if(balldist<GKFALLDISTTHRESHOLD)
					{
						if(velocity<DIDNOTMOVEVELOCITY && velocity>-DIDNOTMOVEVELOCITY)
						{
							//ball is stationary, go and kick it!!
							//maybe add path?
							moveAcYut(fd.ball.r,fd.ball.theta,0,WALK);
						}
						else
						{
							initiateDive();
						}
					}
					
				}
			}




			//
		}
	}

	//close threads
		void* status;
	#ifdef GAMECONTROLLERACTIVE
	void *status_gc;
	pthread_join (thread_id_gc, &status_gc);
	#endif
	#ifdef SWITCHACTIVE
	void *status_gpio;
	pthread_join (thread_id_gpio, &status_gpio);
	#endif
	#ifdef WIFITRANSMISSION
	void *status_wifi;
	pthread_join (thread_id_wifi, &status_wifi);
	#endif
	pthread_join(thread_id_walkthread,&status);

	return 0;

}
