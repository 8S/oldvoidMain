#include "motor.h"


Motor::Motor(MotorType motorType, int id, Communication* comm, int offsetValue, int dMode, int zeroPos)
{
	this->comm = comm;
	this->motorType = motorType;
	if(motorType == EX106)
	{
		motorControlType = COMPLIANCE;
		motorControl.compliance.slope = 32;
		motorControl.compliance.margin = 0;
		motorControl.compliance.punch = 32;
		cwAngleLimit = 0;
		ccwAngleLimit = 4095;
		minPos = 0;
		maxPos = 4095;
		cenPos = 2048;
		angleRange = 270;
	}
	else if(motorType == RX64 || motorType == RX28)
	{
		motorControlType = COMPLIANCE;
		motorControl.compliance.slope = 32;
		motorControl.compliance.margin = 0;
		motorControl.compliance.punch = 32;
		cwAngleLimit = 0;
		ccwAngleLimit = 1023;
		minPos = 0;
		maxPos = 1023;
		cenPos = 512;
		angleRange = 270;
	}
	else if(motorType == MX64 || motorType == MX106)
	{
		motorControlType = PID;
		motorControl.pid.gainP = 32;
		motorControl.pid.gainI = 0;
		motorControl.pid.gainD = 0;
		motorControl.pid.punch = 0;
		cwAngleLimit = 0;
		ccwAngleLimit = 4095;
		minPos = 0;
		maxPos = 4095;
		cenPos = 2048;
		angleRange = 360;
	}
	
	
	motorID = id;
	baudrate = comm->getBaudrate();
	driveMode = dMode;
	maxTorque = 1023;
	torqueOn  = false;
	led = false;
	goalPosition = ccwAngleLimit/2;
	syncedPos = false;
	movSpeed = 0;
	syncedSpeed = false;
	torqueLimit = 1023;
	errorNo = 0;
	slave = false;
	offset = offsetValue;
	this->zeroPos = zeroPos;
	//this->networkedMotor = networkedMotor;
	
	return;
}

int Motor::motorWrite(int location, int dataLength, byte data[], int id = DEFAULT_ID)
{
	byte data_D[dataLength+1];
	data_D[0] = location;
	for(int i =0;i<dataLength;i++)
	data_D[1+i]=data[i];
	return comm->motorSend(0x03,dataLength+1,data_D,id);
}

int Motor::motorRead(int location, int nBytes, byte data[], int id = DEFAULT_ID)
{
	int i =0;
	byte data_D[] = {(byte)location, (byte)nBytes};
	comm->motorSend(0x02,0x02,data_D,id);
	return comm->motorReceive(data);	
}

int Motor::setID(int id)
{
	byte data[] = {id};
	if(motorWrite(0x03, 0x01, data, BROADCAST_ID)==EXIT_SUCCESS)
	{
		motorID = id;
		return EXIT_SUCCESS;
	}
	else
		return EXIT_FAILURE;

}

int Motor::setID()
{
	return setID(motorID);

}

int Motor::getID()
{
	byte data[1];
	motorRead(0x03, 1, data);
	//cout<<"\n"<<data[0]<<"\n";
	return data[0];
}

int Motor::getIDSoft()
{
	return motorID;
}

int Motor::setBaud(int baud)
{
	byte data[] = {baud};
	if(motorWrite(0x04,0x01,data))
	{
		return EXIT_FAILURE;
	}
	else
	{
		baudrate = baud;
		return EXIT_SUCCESS;
	}
}

int Motor::getBaud()
{
	byte data[1];
	motorRead(0x04, 0x01, data, 0xfe);
	//cout<<"\n"<<data[0]<<"\n";
	return data[0];
}

int Motor::checkPositionHard(int pos)
{
	if(pos < minPos || pos > maxPos)
	{
		//cout<<"Out of bound values to Motor "<< motorID <<"\n";
		return EXIT_FAILURE;
	}
	return EXIT_SUCCESS;
}

int Motor::checkGoalPosition(int goalPos)
{
	int max = cwAngleLimit, min;
	if(max < ccwAngleLimit)
	{
		max = ccwAngleLimit;
		min = cwAngleLimit;
	}
	else
		min = ccwAngleLimit;
	if(goalPos < min || goalPos > max)
	{
		printf("Out of bound values to Motor %d \n",motorID);
		return EXIT_FAILURE;
	}
	return EXIT_SUCCESS;
}

int Motor::setGoalPositionSync(int golPos)
{
	if(checkGoalPosition(golPos+offset)==EXIT_FAILURE || slave == true)
		return EXIT_FAILURE;
	
	byte goal[2];
	goal[0] = (golPos + offset)%256;
	goal[1] = (golPos + offset)/256;
	if(comm->addSyncWrite(0x1e, 2, goal, motorID))
		return EXIT_FAILURE;
	else
		return EXIT_SUCCESS;
}

int Motor::setGoalPositionSync()
{
	return setGoalPositionSync(goalPosition);	
}

int Motor::setMovingSpeedSync(int movSpeed)
{
	if(slave == true)
		return EXIT_FAILURE;
	byte speed[2];
	speed[0] = movSpeed%256;
	speed[1] = movSpeed/256;
	if(comm->addSyncWrite(0x20, 0x02, speed, motorID))
		return EXIT_FAILURE;
	else
	{
		return EXIT_SUCCESS;
	}
}

int Motor::setGainPSync(int p)
{
	if(motorType == MX64 || motorType == MX106)
	{
		if(slave==true)
		return EXIT_FAILURE;
		
		if(comm->addSyncWrite(0x1c, 0x01, (byte*)&p, motorID))
			return EXIT_FAILURE;
		else 
			return EXIT_SUCCESS;
	}
	else 
		return EXIT_FAILURE;
}

int Motor::setGoalPosition(int golPos)
{
	if(checkGoalPosition(golPos+offset)==EXIT_FAILURE)
		return EXIT_FAILURE;
	byte goal[2];
	goal[0] = (golPos + offset)%256;
	goal[1] = (golPos + offset)/256;
	if(motorWrite(0x1e, 0x02, goal, motorID))
		return EXIT_FAILURE;
	else
	{
		goalPosition = golPos;
		torqueOn = true;
		syncedPos = true;
		return EXIT_SUCCESS;
	}
}

int Motor::setGoalPosition()
{
	return setGoalPosition(goalPosition);
}

int Motor::setGoalPositionSoft(int goalPos)
{
	if(checkGoalPosition(goalPos+offset)==EXIT_FAILURE)
		return EXIT_FAILURE;
	else 
	{
		goalPosition = goalPos;
		syncedPos = false;
		return EXIT_SUCCESS;
	}
}

int Motor::getGoalPosition()
{
	byte data[2];
	motorRead(0x1e, 0x02, data, motorID);
	int pos = data[0] + (data[1]<<8) - offset;
	//cout<<"\n"<<pos<<"\n";
	return pos;
}

int Motor::getGoalPositionSoft()
{
	return goalPosition;
}

bool Motor::getGoalPositionSync()
{
	return syncedPos;
}

int Motor::goalPositionSynced()
{
	syncedPos = true;
	return EXIT_SUCCESS;
}


int Motor::setDriveMode(int dMode)
{
	byte data[1];
	data[0] = (slave<<1)|dMode;
	if(motorWrite(0x0a, 0x01, data, motorID))
		return EXIT_FAILURE;
	else
	{	
		driveMode = dMode;
		return EXIT_SUCCESS;
	}	
}

int Motor::getDriveMode()
{
	byte data[1];
	motorRead(0x1a, 0x01, data, motorID);
	//cout<<"\n"<<data[0]<<"\n";
	driveMode = data[0]&0x01;
	return driveMode;
}

int Motor::setMovingSpeed(int movSpeed)
{
	byte speed[2];
	speed[0] = movSpeed%256;
	speed[1] = movSpeed/256;
	if(motorWrite(0x20, 0x02, speed, motorID))
		return EXIT_FAILURE;
	else
	{
		this->movSpeed = movSpeed;
		syncedSpeed = true;
		torqueOn = true;
		return EXIT_SUCCESS;
	}
}

int Motor::setMovingSpeed()
{
	return setMovingSpeed(movSpeed);
}

int Motor::setMovingSpeedSoft(int movSpeed)
{
	if(movSpeed >= 0 && movSpeed <= 1023)
	{
		this->movSpeed = movSpeed;
		syncedSpeed = false;
		return EXIT_SUCCESS;	
	}
	else 
		return EXIT_FAILURE;
}	

int Motor::getMovingSpeed()
{
	byte data[2];
	motorRead(0x20, 0x02, data, motorID);
	int speed = data[0] + (data[1]<<8);
	//cout<<"\n"<<speed<<"\n";
	return speed;
}

int Motor::getMovingSpeedSync()
{
	return syncedSpeed;
}

int Motor::setCWAngleLimitHard(int angLimit)
{
	if(checkPositionHard(angLimit) == EXIT_FAILURE)
	return EXIT_FAILURE;
	
	byte lim[2];
	lim[0] = angLimit%256;
	lim[1] = angLimit/256;
	if(motorWrite(0x06, 0x02, lim, motorID))
		return EXIT_FAILURE;
	else
	{
		if(cwAngleLimit < angLimit)
		cwAngleLimit = angLimit;
		return EXIT_SUCCESS;
	}	
}

int Motor::setCWAngleLimit(int angLimit)
{
	if(setCWAngleLimitHard(angLimit) == EXIT_SUCCESS)
	{
		cwAngleLimit = angLimit;
		return EXIT_SUCCESS;
	}
	else 
	return EXIT_FAILURE;
}

int Motor::setCWAngleLimit()
{
	return setCWAngleLimit(cwAngleLimit);
}

int Motor::setCWAngleLimitSoft(int angLimit)
{
	if(checkPositionHard(angLimit) == EXIT_SUCCESS)
	{
		cwAngleLimit = angLimit;
		return EXIT_SUCCESS;		
	}
	else
	return EXIT_FAILURE;
}

int Motor::setCCWAngleLimitHard(int angLimit)
{
	if(checkPositionHard(angLimit) == EXIT_FAILURE)
	return EXIT_FAILURE;

	byte lim[2];
	lim[0] = angLimit%256;
	lim[1] = angLimit/256;
	if(motorWrite(0x08, 0x02, lim, motorID))
		return EXIT_FAILURE;
	else
	{
		if(ccwAngleLimit > angLimit)
		ccwAngleLimit = angLimit;
		return EXIT_SUCCESS;
	}
}

int Motor::setCCWAngleLimit(int angLimit)
{
	if(setCCWAngleLimitHard(angLimit)==EXIT_SUCCESS)
	{
		ccwAngleLimit = angLimit;
		return EXIT_SUCCESS;
	}
	else
	return EXIT_FAILURE;
}

int Motor::setCCWAngleLimit()
{
	return setCCWAngleLimit(ccwAngleLimit);
}

int Motor::setCCWAngleLimitSoft(int angLimit)
{
	if(checkPositionHard(angLimit) == EXIT_SUCCESS)
	{
		ccwAngleLimit = angLimit;
		return EXIT_SUCCESS;
	}	
	else
	return EXIT_FAILURE;
}

int Motor::getCWAngleLimit()
{
	return cwAngleLimit;
}

int Motor::getCCWAngleLimit()
{
	return ccwAngleLimit;
}


int Motor::getCWAngleLimitHard()
{
	byte data[2];
	motorRead(0x06, 0x02, data, motorID);
	int angleLimit = data[0] + (data[1]<<8);
	//cout<<"\n"<<angleLimit<<"\n";
	return angleLimit;
}


int Motor::getCCWAngleLimitHard()
{
	byte data[2];
	motorRead(0x08, 0x02, data, motorID);
	int angleLimit = data[0] + (data[1]<<8);
	//cout<<"\n"<<angleLimit<<"\n";
	return angleLimit;
}

int Motor::torqueEnable()
{
	byte data[] = {0x01};
	if(motorWrite(0x18, 0x01, data, motorID))
		return EXIT_FAILURE;
	else
	{
		torqueOn = true;
		return EXIT_SUCCESS;
	}
}

int Motor::torqueDisable()
{
	byte data[] = {0x00};
	if(motorWrite(0x18, 0x01, data, motorID))
		return EXIT_FAILURE;
	else
	{
		torqueOn = false;
		return EXIT_SUCCESS;
	}
}

int Motor::getTorqueStatus()
{
	byte data[1];
	motorRead(0x18, 0x01, data, motorID);
	torqueOn = data[0];
	//cout<<"\n"<<data[0]<<"\n";
	return data[0];
}

int Motor::setComplianceMargin(int margin)
{
	if(motorType == EX106 || motorType == RX28 || motorType == RX64)
	{
		if(motorWrite(0x1a, 0x01, (byte*)&margin, motorID) && motorWrite(0x1b, 0x01, (byte*)&margin, motorID))
			return EXIT_FAILURE;
		else
		{
			motorControl.compliance.margin = margin;
			return EXIT_SUCCESS;
		}
	}
	else 
		return EXIT_FAILURE;		
}

int Motor::setComplianceSlope(int slope)
{
	if(motorType == EX106 || motorType == RX28 || motorType == RX64)
	{
		if(motorWrite(0x1c, 0x01, (byte*)&slope, motorID) && motorWrite(0x1d, 0x01, (byte*)&slope, motorID))
			return EXIT_FAILURE;
		else
		{
			motorControl.compliance.slope = slope;
			return EXIT_SUCCESS;
		}
	}
	else
		return EXIT_FAILURE;
}

int Motor::setCompliancePunch(int punch)
{
	byte pun[2];
	pun[1] = punch%256;
	pun[2] = punch/256;
	//if(motorType == EX106 || motorType == RX28)
	if(motorWrite(0x30, 0x02, pun, motorID))
		return EXIT_FAILURE;
	else
	{
		if(motorType == EX106 || motorType == RX64 || motorType == RX28)
		motorControl.compliance.punch = punch;
		else
		motorControl.pid.punch = punch;
		return EXIT_SUCCESS;
	}
	
}

int Motor::getComplianceMargin()
{
	if(motorType == EX106 || motorType == RX64 || motorType == RX28)
	{
		byte data[1];
		motorRead(0x1a, 0x01, data, motorID);
		//cout<<"\n"<<data[0]<<"\n";
		motorControl.compliance.margin = data[0];
		return data[0];
	}
	
	return EXIT_FAILURE;
}

int Motor::getComplianceSlope()
{
	if(motorType == EX106 || motorType == RX64 || motorType == RX28)
	{
		byte data[1];
		motorRead(0x1c, 0x01, data, motorID);
		//cout<<"\n"<<data[0]<<"\n";
		motorControl.compliance.slope = data[0];
		return data[0];
	}
	
	return EXIT_FAILURE;
}

int Motor::getCompliancePunch()
{
	byte data[1];
	motorRead(0x30, 0x01, data, motorID);
	if(motorType == EX106 || motorType == RX64 || motorType == RX28)
	motorControl.compliance.punch = data[0];
	else if(motorType == MX106 || motorType == MX64)
	motorControl.pid.punch = data[0];
	else
	return EXIT_FAILURE;
	
	//cout<<"\n"<<data[0]<<"\n";
	return data[0];
}

int Motor::setGainP(int p)
{
	if(motorType == MX64 || motorType == MX106)
	{
		if(motorWrite(0x1c, 0x01, (byte*)&p, motorID))
			return EXIT_FAILURE;
		else
		{
			motorControl.pid.gainP = p;
			return EXIT_SUCCESS;
		}
	}
	else
	return EXIT_FAILURE;		
}

int Motor::setGainI(int i)
{
	if(motorType == MX64 || motorType == MX106)
	{
		if(motorWrite(0x1b, 0x01, (byte*)&i, motorID))
			return EXIT_FAILURE;
		else
		{
			motorControl.pid.gainI = i;
			return EXIT_SUCCESS;
		}
	}
	else
	return EXIT_FAILURE;	
}

int Motor::setGainD(int d)
{
	if(motorType == MX64 || motorType == MX106)
	{
		if(motorWrite(0x1a, 0x01, (byte*)&d, motorID))
			return EXIT_FAILURE;
		else
		{
			motorControl.pid.gainD = d;
			return EXIT_SUCCESS;
		}
	}
	else
	return EXIT_FAILURE;	
}

int Motor::getGainP()
{
	if(motorType == MX106 || motorType == MX64)
	{
		byte data[1];
		motorRead(0x1c, 0x01, data, motorID);
		//cout<<"\n"<<data[0]<<"\n";
		motorControl.pid.gainP = data[0];
		return data[0];
	}
	else return EXIT_FAILURE;
}

int Motor::getGainI()
{
	if(motorType == MX106 || motorType == MX64)
	{
		byte data[1];
		motorRead(0x1b, 0x01, data, motorID);
		//cout<<"\n"<<data[0]<<"\n";
		motorControl.pid.gainI = data[0];
		return data[0];
	}
	else 
	return EXIT_FAILURE;
}

int Motor::getGainD()
{
	if(motorType == MX106 || motorType == MX64)
	{
		byte data[1];
		motorRead(0x1a, 0x01, data, motorID);
		//cout<<"\n"<<data[0]<<"\n";
		motorControl.pid.gainD = data[0];
		return data[0];
	}
	else return EXIT_FAILURE;
	
}


int Motor::ledOn()
{
	byte data[] = {0x01};
	if(motorWrite(0x19, 0x01, data, motorID))
		return EXIT_FAILURE;
	else
	{
		led = true;
		return EXIT_SUCCESS;
	}
}

int Motor::ledOff()
{
	byte data[] = {0x00};
	if(motorWrite(0x19, 0x01, data, motorID))
		return EXIT_FAILURE;
	else
	{
		led = false;
		return EXIT_SUCCESS;
	}	
}

int Motor::ledStatus()
{
	byte data[1];
	motorRead(0x19, 0x01, data, motorID);
	//cout<<"\n"<<data[0]<<"\n";
	led = data[0];
	return data[0];
}

int Motor::getPresentPosition()
{
	byte data[2];
	motorRead(0x24, 0x02, data, motorID);
	int pos = data[0] + (data[1]<<8) -offset;
	//cout<<"\n"<<pos<<"\n";
	return pos;
}

int Motor::getPresentSpeed()
{
	byte data[2];
	motorRead(0x26, 0x02, data, motorID);
	int speed = data[0] + (data[1]<<8);
	//cout<<"\n"<<speed<<"\n";
	return speed;
}

int Motor::getPresentLoad()
{
	byte data[2];
	motorRead(0x28, 0x02, data, motorID);
	int load = data[0] + ((data[1]&3)<<8);
	//cout<<"Load:"<<motorID<<" "<<load<<"\n";
	return load;
}

int Motor::ping()
{
	byte* data = NULL;
	comm->motorSend(0x01, 0x00, NULL, motorID);
	if(comm->motorReceive(data))
	{
		printf("Ping Failed %d\n",motorID);
		return EXIT_FAILURE;
	}
	else
	{
		printf("Ping Success %d\n",motorID);
		return EXIT_SUCCESS;
	}
}

int Motor::slaveModeSoft()
{
	slave = true;
	printf("Software Slave Mode set for ID %d\n",motorID);
	return EXIT_SUCCESS;
}

bool Motor::isSlave()
{
	return slave;
}
int Motor::masterModeSoft()
{
	slave = false;
	printf("Software Master Mode set for ID %d\n",motorID);
	return EXIT_SUCCESS;
}

int Motor::slaveMode()
{
	byte data[1] = { (true<<1) | driveMode};
	if(motorWrite(0x0a, 0x01,data,motorID))
	return EXIT_FAILURE;
	else
	{	
		slave = true;
		return EXIT_SUCCESS;
	}
	
}

int Motor::masterMode()
{
	byte data[1] = {(false<<1) | driveMode};
	if(motorWrite(0x0a, 0x01, data,motorID))
	return EXIT_FAILURE;
	else
	{
		slave = false;
		return EXIT_SUCCESS;
	}
}

int Motor::setOffset(int offsetValue)
{
	offset = offsetValue;
	return EXIT_SUCCESS;
}

double Motor::getSensedCurrent()
{
	byte data[2];
	motorRead(0x44, 0x02, data, motorID);
	int CURRENT = data[0] + (data[1]<<8);
	double sensedCurrentAmp = 0.045* (CURRENT-2048);
	//cout<<"\n"<<sensedCurrent<<"\n";
	return sensedCurrentAmp;
}


int Motor::getMaxPos()
{
	return maxPos;
}

int Motor::getMinPos()
{
	return minPos;
}

int Motor::getCenPos()
{
	return cenPos;
}

int Motor::getZeroPos()
{	
	return zeroPos;
}

int Motor::setZeroPos(int zeroPos)
{	
	if(checkGoalPosition(zeroPos))
	return EXIT_FAILURE;
	else 
	{
		this->zeroPos = zeroPos;
		return EXIT_SUCCESS;
	}
}

int Motor::ang2Pos(float ang)
{
	return (int)((float)ang/(float)angleRange*((float)maxPos - (float)minPos));
}

float Motor::pos2Ang(int pos)
{
	return (float)pos*angleRange/((float)maxPos-(float)minPos);
}
