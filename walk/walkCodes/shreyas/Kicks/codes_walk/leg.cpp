#include "leg.h"

Leg::Leg(int leg, Communication* comm, int ids[], int offsets[], int driveMode[], int zeros[])
{
	this->comm = comm;
	for (int i = 0; i < LEG_MOTOR_COUNT; ++i)
	{
		printf("ID\t%2d\tOffset\t%5d\tDriveMode\t%1d\tZero\t%4d\t\n",ids[i],offsets[i],driveMode[i],zeros[i]);
		m[i] = new Motor(MX106, ids[i], this->comm, offsets[i], driveMode[i], zeros[i]);
	}
	for (int i = 0; i <LEG_SLAVE_COUNT; ++i)
	{
		m[legSlaveMotors[i]]->slaveModeSoft();
	}
	x = legHeight;
	y = 0;
	z = 0;
	theta = 0;
	this->leg = leg;
	//this->mass[0] = mass[0];	
}

Leg::~Leg()
{
	//printf("Leg does not exist\n");

}

void Leg::setIDList(int* idList, int n)
{
	for (int i = 0; i < n && i < LEG_MOTOR_COUNT; ++i)
	{
		m[i]->setIDSoft(idList[i]);
	}
}

bool* Leg::pingLeg(bool pingStatus[])
{
	
	for (int i = 0; i < LEG_MOTOR_COUNT; ++i)
	pingStatus[i] = ~m[i]->ping();
	
	return pingStatus;
	
}

int Leg::pingLeg()
{
	bool pingStatus[LEG_MOTOR_COUNT];
	pingLeg(pingStatus);
	
	for(int i=0; i<LEG_MOTOR_COUNT;i++)
	{
		if(pingStatus[i] == false)
		return EXIT_FAILURE;
	}
	return EXIT_SUCCESS;
}

int Leg::pingMotor(int id)
{
	return m[id]->ping();
}	

int Leg::setSpeed(int speed)
{
	for(int i=0; i< LEG_MOTOR_COUNT; i++)
	{
		m[i]->setMovingSpeedSync(speed);
	}

}

int Leg::setMotorSpeed(int id, int speed)
{
	m[id]->setMovingSpeed(speed);
}


int Leg::getLoad()
{
	for (int i = 0; i < LEG_MOTOR_COUNT; ++i)
	{
		m[i]->getPresentLoad();
	}
}


int Leg::motorLedOn(int id)
{
	m[id]->ledOn();
}

int Leg::getMotorLoad(int motor)
{
	m[motor]->getPresentLoad();
}

int Leg::setMotorGainP(int p, int motorSelect)
{
	for (int motor = 0; motor < LEG_MOTOR_COUNT; motor++)
	{
		if((motorSelect&(int)pow(2,motor)) >> motor)
			if(m[motor]->setGainPSync(p))
				return EXIT_FAILURE;
	}
	
	return EXIT_SUCCESS;
}

double*  Leg::getSensedCurrentAmps()
{
	double *current=new double[LEG_MOTOR_COUNT];
	for(int motor=0;motor< LEG_MOTOR_COUNT; motor++)
	{
		current[motor]=m[motor]->getSensedCurrent();
	}
	return current;
}


int Leg::runIK(double x, double y, double z, double phi)
{
	double theta[7];
	int pos[7];
	this->legCoods[X] = this->x = x;
	this->legCoods[Y] = this->y = y;
	this->legCoods[Z] = this->z = z;
	this->legCoods[4] = this->theta = phi;
	
	double realX = x;
	double realY = y*cos(deg2rad(phi)) + z*sin(deg2rad(phi));
	double realZ = -y*sin(deg2rad(phi)) + z*cos(deg2rad(phi));
	
	//printf("RealX\t%lf\tRealY\t%lf\tRealZ\t%lf\n",realX,realY,realZ);
	theta[HIP_YAW] = phi;
	pos[6] = m[HIP_YAW]->ang2Pos(phi);

	if(leg==RIGHT)
		pos[6]=m[HIP_YAW]->getCenPos()+pos[6];
	else if(leg==LEFT)
		pos[6]=m[HIP_YAW]->getCenPos()-pos[6];
	
	//printf("%d",(x*x+y*y+z*z)<pow(LengthLink2,2));
	if((x*x+y*y+z*z)>=pow(LengthLink1+LengthLink2+LengthLink3,2) || (x*x+y*y+z*z)<pow(LengthLink2,2))
	{
		printf("Invalid position: x:%lf y:%lf z:%lf\n", x, y, z);
		return EXIT_FAILURE;
	}
	
	double projectedLegLength = sqrt(pow(realX,2)+pow(realZ,2)) - LengthLink2;
	theta[HIP_ROLL] = atan(realZ/realX);
	theta[HIP_PITCH] = asin(1) - asin((pow(projectedLegLength,2)+pow(LengthLink3,2)+realY*realY-pow(LengthLink1,2))/(2*LengthLink3*sqrt(pow(realY,2)+pow(projectedLegLength,2)))) + asin(realY/sqrt(pow(realY,2)+pow(projectedLegLength,2)));
	theta[KNEE2] = theta[HIP_PITCH];
	theta[KNEE1] = asin((realY-LengthLink3*sin(theta[HIP_PITCH]))/LengthLink1);
	theta[FOOT_PITCH] = theta[KNEE1];
	theta[FOOT_ROLL] = theta[HIP_ROLL];
	
	for(int i=0;i<LEG_MOTOR_COUNT-1;++i)
	{
		theta[i] = rad2deg(theta[i]);
		pos[i] = m[i]->ang2Pos(theta[i]);
		//printf("Theta[%d]\t%lf\tPos\t%d\n",i,theta[i],pos[i]);
	}
	
	pos[HIP_ROLL] = m[HIP_ROLL]->getCenPos() - pos[HIP_ROLL];
	pos[HIP_PITCH]= m[HIP_PITCH]->getCenPos() - pos[HIP_PITCH];
	pos[KNEE2] = m[KNEE2]->getCenPos() + pos[KNEE2];
	pos[KNEE1] = m[KNEE1]->getCenPos() - pos[KNEE1]; 
	pos[FOOT_PITCH] = m[FOOT_PITCH]->getCenPos()+ pos[FOOT_PITCH];
	pos[FOOT_ROLL] = m[FOOT_ROLL]->getCenPos() + pos[FOOT_ROLL];
	
		
	for (int i = 0; i < 7; ++i)
	{
		//printf("Theta[%d]\t%lf\tPos\t%d\n",i,theta[i],pos[i]);
		m[i]->setGoalPositionSoft(pos[i]);
		#ifdef DEBUG
		//printf("MotorID\t%2d\tPos\t%4d\tAngle\t%5d\n",m[i]->getIDSoft(),m[i]->getGoalPositionSoft(),pos[i]);
		#endif	
	}
}

int Leg::syncLeg()
{
	
	if(setGoalPositionSync() == EXIT_FAILURE || comm->syncFlush() == EXIT_FAILURE)
	return EXIT_FAILURE;
	
	for(int i=0;i<LEG_MOTOR_COUNT;i++)
		m[i]->goalPositionSynced();
	
	return EXIT_SUCCESS;
}

int Leg::setGoalPositionSync()
{
	for(int i=0;i<LEG_MOTOR_COUNT;i++)
	{
		if(m[i]->getGoalPositionSync() == false && m[i]->isSlave() == false)
		{
			if(m[i]->setGoalPositionSync()==EXIT_FAILURE)
			return EXIT_FAILURE;			
		}
	}
	
	return EXIT_SUCCESS;
}

const double (&(Leg::getFeetCoods()))[FEET_CORNERS][AXES]
{
	return feetCoods;
}
	
const double (&(Leg::runFK(Motor* mot[])))[LEG_MOTOR_COUNT][AXES]
{
	double theta[LEG_MOTOR_COUNT];
	
	for(int i=0;i<LEG_MOTOR_COUNT;i++)
	{
		theta[i] = deg2rad(((double)(mot[i]->getGoalPositionSoft() - mot[i]->getZeroPos())/(double)mot[i]->getMaxPos() * 360));
	}
	
	if(leg == LEFT)
	theta[HIP_YAW] *=-1; 
	
	theta[HIP_PITCH] *=-1;
	theta[KNEE2] *= -1;
	theta[HIP_ROLL] *=-1;
	theta[KNEE1] *= -1;
	theta[FOOT_PITCH] *=-1;
	
	#ifdef DEBUG
	for(int i=0;i<LEG_MOTOR_COUNT;i++)
	printf("MotorID\t%2d\tPos\t%4d\tAngle\t%3.5lf\n",mot[i]->getIDSoft(),mot[i]->getGoalPositionSoft(),rad2deg(theta[i]));
	#endif
	
	// Intersection of HIP_YAW, HIP_ROLL, HIP_PITCH axes is assumed to be origin
	jointCoods[HIP_YAW][X] = 0.0;
	jointCoods[HIP_YAW][Y] = 0.0;
	jointCoods[HIP_YAW][Z] = 0.0;
	
	jointCoods[HIP_ROLL][X] = 0.0;
	jointCoods[HIP_ROLL][Y] = 0.0;
	jointCoods[HIP_ROLL][Z] = 0.0;
	
	jointCoods[HIP_PITCH][X] = 0.0;
	jointCoods[HIP_PITCH][Y] = 0.0;
	jointCoods[HIP_PITCH][Z] = 0.0;
	
	jointCoods[KNEE2][X] = LengthLink1*cos(theta[HIP_PITCH])*cos(theta[HIP_ROLL]);
	jointCoods[KNEE2][Y] = LengthLink1*sin(theta[HIP_PITCH]);
	jointCoods[KNEE2][Z] = LengthLink1*cos(theta[HIP_PITCH])*sin(theta[HIP_ROLL]);
	
	jointCoods[KNEE1][X] = jointCoods[KNEE2][X] + LengthLink2*cos(theta[HIP_ROLL]);
	jointCoods[KNEE1][Y] = jointCoods[KNEE2][Y] + 0;
	jointCoods[KNEE1][Z] = jointCoods[KNEE2][Z] + LengthLink2*sin(theta[HIP_ROLL]);;
	
	jointCoods[FOOT_ROLL][X] = jointCoods[FOOT_PITCH][X] = jointCoods[KNEE1][X] + LengthLink3 * cos(theta[KNEE1]) * cos(theta[HIP_ROLL]);
	jointCoods[FOOT_ROLL][Y] = jointCoods[FOOT_PITCH][Y] = jointCoods[KNEE1][Y] + LengthLink3*sin(theta[KNEE1]);
	jointCoods[FOOT_ROLL][Z] = jointCoods[FOOT_PITCH][Z] = jointCoods[KNEE1][Z] + LengthLink3 * cos(theta[KNEE1]) * sin(theta[HIP_ROLL]);
	
	double temp[AXES];
	for(int i=0;i<FEET_CORNERS;i++)
	{
		temp[X]=jointCoods[FOOT_ROLL][X] + feetCornerOffset[leg][i][X]*cos(theta[FOOT_ROLL]) - feetCornerOffset[leg][i][Z]*sin(theta[FOOT_ROLL]);
		temp[Y]=jointCoods[FOOT_ROLL][Y] + feetCornerOffset[leg][i][Y];
		temp[Z]=jointCoods[FOOT_ROLL][Z] + feetCornerOffset[leg][i][X]*sin(theta[FOOT_ROLL]) + feetCornerOffset[leg][i][Z]*cos(theta[FOOT_ROLL]);
		
		feetCoods[i][X] = temp[X];
		feetCoods[i][Y] = temp[Y]*cos(theta[HIP_YAW]) - temp[Z]*sin(theta[HIP_YAW]);
		feetCoods[i][Z] = temp[Y]*sin(theta[HIP_YAW]) + temp[Z]*cos(theta[HIP_YAW]);
		//printf("%4.4lf\t%4.4lf\t%4.4lf\n",feetCoods[i][X],feetCoods[i][Y],feetCoods[i][Z]);	
	}
	
	for(int i=0;i<LEG_MOTOR_COUNT;i++)
	{
		for(int j=0;j<AXES;j++)
		temp[j] = jointCoods[i][j];
		jointCoods[i][Y] = temp[Y]*cos(theta[HIP_YAW]) - temp[Z]*sin(theta[HIP_YAW]);
		jointCoods[i][Z] = temp[Y]*sin(theta[HIP_YAW]) + temp[Z]*cos(theta[HIP_YAW]); 
	}
	
	#ifdef DEBUG
	for(int i = 0; i <LEG_MOTOR_COUNT;i++)
		printf("%d\tX%lf\tY%lf\tZ%lf\n",i,jointCoods[i][X],jointCoods[i][Y],jointCoods[i][Z]);
	#endif
	//printf("%d\tX%lf\tY%lf\tZ%lf\n",0,jointCoods[0][X],jointCoods[0][Y],jointCoods[0][Z]);
	
	return jointCoods;
	
}

const double (&(Leg::runFK())) [LEG_MOTOR_COUNT][AXES]
{
	return runFK(this->m);
}

const double (&Leg::getLegCOM()) [AXES]
{	
	runFK();
	
	COM[X] =0;
	COM[Y] =0;
	COM[Z] =0;
	
	float massCoods[5][AXES+1];
	float mass[5];
	float totMass =0;
	
	mass[0] = 0.509;   // Kilograms
	mass[1] = 0.176;
	mass[2] = 0.415;
	mass[3] = 0.176;
	mass[4] = 0.825;	
	
	massCoods[0][X] = jointCoods[FOOT_ROLL][X] + 6.251;
	massCoods[0][Y] = jointCoods[FOOT_ROLL][Y] + 18.799;
	massCoods[0][Z] = jointCoods[FOOT_ROLL][Z] + 1.969;
	
	for(int i=1;i<=3;i++)
		for(int j=0;j<AXES;j++)
		massCoods[i][j] = (jointCoods[i][j] + jointCoods[i+1][j])/2;
		
	massCoods[4][X] = jointCoods[HIP_ROLL][X] - 17.871;
	massCoods[4][Y] = jointCoods[HIP_ROLL][Y] - (40.20/2 - 18.14);
	massCoods[4][Z] = jointCoods[HIP_ROLL][Z];
	
	for(int i=0;i<5;i++)
	{
		for(int j=0;j<AXES;j++)
			COM[j] += massCoods[i][j]*mass[i];
			
		totMass +=mass[i];
	}

	for(int j=0;j<3;j++)
	{
		COM[j]/=totMass;
		//printf("LegCOM[%d]\t%lf\n",j,COM[j]);
	}
	
	return COM;
}

const float (&(Leg::getLegCoods())) [AXES+1]
{
	return legCoods;
}  
	
