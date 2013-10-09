#include "walk.h"
int k=1;


Walk::Walk(AcYut* bot)
{
	this->bot = bot;
	legYin=-1.773347;
	supLegYin=1.773347;
	veloYin=10;
	veloYfi=10;
	legZin=25.155276;
	supLegZin=25.155276;
	veloZin=-170;
	veloZfi=170;
	zMax=70;
	lift=40;
	legRotin=0;
	legRotfi=0;
	supLegRotin=0;
	supLegRotfi=0;
	
	start();
};

int Walk::kick()
{
	



}

int Walk::start()
{
	lift=5;
	dribble();
	lift=10;
	dribble();
	lift=20;
	dribble();
	lift=30;
	dribble();
}

float Walk::accelerate()
{
	veloYfi=veloYfi*1.72;
	legRotfi = 0;
	supLegRotfi =0;
}

float Walk::decelerate()
{
	veloYfi=veloYfi*0.35;
	legRotfi=0;
	supLegRotfi=0;
}

float Walk::velocity()
{
	return veloYfi;
}

float Walk::turnright(float theta)
{
	if(theta==0)
	{
		legRotfi=0;
		supLegRotfi=0;
		dribble();
		return 0;
	}
	if(theta>15)
		legRotfi=(1-(int)leg)*15;
	else
		legRotfi=(1-(int)leg)*theta;
	supLegRotfi=0;
	dribble();	
	theta=theta-legRotfi;
	turnright(theta);	
}

float Walk::turnleft(float theta)
{
	if(theta==0)
	{
		legRotfi=0;
		supLegRotfi=0;
		dribble();
		return 0;
	}
	if(theta>15)
		legRotfi=((int)leg)*15;
	else
		legRotfi=((int)leg)*theta;
	supLegRotfi=0;
	dribble();	
	theta=theta-legRotfi;
	turnleft(theta);	
}

int Walk::dribble()
{
	double av=13;
	double corr;
	leg=(LEG)(1-(int)leg);	
	printf("%d\t",leg);
	
	printf("VYin\t%lf\tVyfi\t%lf\n",veloYin,veloYfi);
	int fps = 60;
	int sleep = 1000000.0/(double)fps;
	double timeInc =1.0/(double)fps;
	double feetSeperation = 0;
	double hipLength = 130;
	
	double Tc = sqrt(600.00/9810.0);
	legZin += hipLength/2;
	supLegZin += hipLength/2;
	
	///// desired Values 
	
	double D_dsp1Time = 0.05;
	double D_dsp2Time = 0.05;
	
	// ////printf("Tc\t\t%lf\n",Tc);
	// ////printf("legZin\t\t%lf\n",legZin);
	// ////printf("supLegZin\t%lf\n",supLegZin);
	
	//// Z Control 
	
	double sspZAmp   = zMax; 
	double sspZPhs   = asinh(veloZin*Tc/sspZAmp);
	double sspZTime  = Tc * (asinh(veloZfi*Tc/sspZAmp) - sspZPhs);
	double sspZSupin = sspZAmp * cosh(sspZPhs);
	double sspZSupfi = sspZAmp * cosh(sspZTime/Tc + sspZPhs);
	
	double sspTime   = sspZTime;
	double dsp1Time  = (supLegZin - sspZSupin)/(-veloZin);		
	double dsp2Time  = D_dsp2Time;
	double stepTime  = sspTime + dsp1Time + dsp2Time;
	
	double sspZin    = legZin + (-veloZin)*dsp1Time;
	double P_sspZAmp = zMax;
	double P_sspZPhs = asinh(-veloZfi*Tc/P_sspZAmp);
	double P_sspZin  = P_sspZAmp * cosh(P_sspZPhs);
	double P_legZin  = P_sspZin - (-veloZfi)* D_dsp2Time;
	double P_veloZfi = veloZfi + (veloZfi + veloZin)/2;
	double P_sspZTime= Tc * (asinh(P_veloZfi*Tc/P_sspZAmp) - P_sspZPhs);
	
	
	double sspZfi    = P_legZin + veloZfi*D_dsp1Time; 
	double legZfi    = sspZfi - veloZfi*dsp2Time;
	double supLegZfi = sspZSupfi + veloZfi*dsp2Time;
	
	//TODO add case in which dsp1Time becomes negative 
	//TODO add case where dsp2Time needs to be modified (if required)
	//TODO calculation of sspZfi for next step based on predicted change in velocity for the next step 
	
	////printf("zMax\t\t%lf\n",zMax);
	////printf("sspZPhs\t\t%lf\n",sspZPhs);
	////printf("sspZTime\t%lf\n",sspZTime);
	////printf("sspZin\t\t%lf\n",sspZin);
	////printf("sspZSupin\t%lf\n",sspZSupin);
	////printf("sspZfi\t\t%lf\n",sspZfi);
	////printf("sspZSupfi\t%lf\n",sspZSupfi);
	////printf("dsp1Time\t%lf\n",dsp1Time);
	////printf("stepTime\t%lf\n",stepTime);
	////printf("LegZfi\t\t%lf\n",legZfi);
	////printf("supLegZfi\t%lf\n",supLegZfi);
	
	// Y Control 
	
	legYin    = -legYin;
	supLegYin = -supLegYin;
	
	double sspYin    = legYin + veloYin * dsp1Time;
	double sspYSupin = supLegYin + veloYin * dsp1Time;
	double sspYAmp   = veloYin * sqrt(pow(Tc,2) - pow(sspYSupin/veloYin,2));
	double sspYPhs   = asinh((sspYSupin/veloYin)/sqrt(pow(Tc,2) - pow(sspYSupin/veloYin,2)));
	double veloYfi_d   = sspYAmp/Tc * cosh (sspTime/Tc + sspYPhs);
	//////printf("**************%lf******************\n",sspYAmp/Tc * cosh (sspTime/Tc + sspYPhs));
	double P_sspYAmp = sgn(veloYin) * Tc * sqrt((2*cosh(P_sspZTime/Tc)*veloYfi_d*veloYfi - pow(veloYfi_d,2) - pow(veloYfi,2))/(pow(sinh(P_sspZTime/Tc),2)));
	double P_sspYPhs = -acosh(veloYfi_d*Tc/P_sspYAmp);
	double P_sspYin  = P_sspYAmp * sinh(P_sspYPhs);
	double P_legYin  = P_sspYin - veloYfi_d*D_dsp1Time;

	double sspYfi    = P_legYin - veloYfi_d*dsp2Time;
	double sspYSupfi = sspYAmp * sinh(sspTime/Tc + sspYPhs);
	
	double legYfi    = sspYfi + veloYfi_d*dsp2Time;
	double supLegYfi = sspYSupfi + veloYfi_d*dsp2Time;
	
	////printf("\n\n\n");
	////printf("legYin\t\t%lf\n",legYin);
	////printf("supLegYin\t%lf\n",supLegYin);
	////printf("sspYin\t\t%lf\n",sspYin);
	////printf("sspYSupin\t%lf\n",sspYSupin);
	////printf("sspYAmp\t\t%lf\n",sspYAmp);
	////printf("sspYPhs\t\t%lf\n",sspYPhs);
	////printf("sspYfi\t\t%lf\n",sspYfi);
	////printf("sspYSupfi\t%lf\n",sspYSupfi);
	////printf("legYfi\t\t%lf\n",legYfi);
	////printf("suplegYfi\t%lf\n",supLegYfi);
	////printf("veloYfi_d\t%lf\n",veloYfi_d);
	
	////printf("P_sspYAmp\t%lf\n",P_sspYAmp);
	////printf("P_sspYPhs\t%lf\n",P_sspYPhs);
	////printf("P_sspZTime\t%lf\n",P_sspZTime);
	////printf("P_sspYin\t\t%lf\n",P_sspYin);
	////printf("P_legYin\t\t%lf\n",P_legYin);
	// Y (cubic)
	
	double a = ((-veloYfi-veloYin)-2*(-sspYfi+sspYin)/sspTime)/pow(sspTime,2);
	double b = ((-sspYfi+sspYin)/sspTime+veloYin-a*pow(sspTime,2))/sspTime;
	double c = -veloYin;
	double d = -sspYin;
	
	double height = 390;
	//double lift   = 30;
	double xfreq  = 2*3.14;
	double displacement = 1;
	double startX = dsp1Time-1.0/60.0;
	double stopX  = stepTime-dsp2Time + 1.0/60.0;
	double xPhase =-3.14/2;
	
	double dampLift = 0;
	double dampFreq = 2*3.14;
	double dampStart = dsp1Time + sspTime - 4.0/60.0;
	double dampEnd   = stepTime;
	double dampPhase = -3.14/2;
	double dampDisplacement = 1;
	
	double fraction;
	
	double x,xr,y,yr,z,zr,phi,phiR,s,sr;
	double yReal,yrReal,zReal,zrReal;
	double walkTime =0.0;
	int walk_i = 0;
	int target_i = 0;
	double *fval;
	//compliance(leg,10,10);
	
	
	int state = DSP;
//	////printf("*************************************** STEP **********************************************\n");	
	for(walkTime = 0.0/fps; walkTime<=stepTime; walkTime +=timeInc)
	{
		if(walkTime < dsp1Time)
		{
			x  = height;
			xr = height;
			y  = -legYin - veloYin*walkTime;
			yr = -supLegYin - veloYin*walkTime;
			z  = legZin - veloZin*walkTime - hipLength/2;
			zr = supLegZin + veloZin*walkTime - hipLength/2;
			phi= legRotin; 
			phiR= supLegRotin;
	//		////printf("DSP1\t");
			state = DSP;
		}
		else if (walkTime >= dsp1Time && walkTime <= dsp1Time + sspTime)
		{
			fraction=2*(walkTime-startX)/(stopX-startX);
			//x  = height - lift * (sin(xfreq*((walkTime-startX)/(stopX-startX))+xPhase) + displacement)/(1+displacement);
			x = height- lift*fraction*(2-fraction);
			xr = height;
			y=a*pow(walkTime-dsp1Time,3)+b*pow(walkTime-dsp1Time,2)+c*(walkTime-dsp1Time)+d;
			//y  = linear(-sspYin,-sspYfi, ((walkTime-dsp1Time)/sspTime);
			yr = -sspYAmp * sinh((walkTime-dsp1Time)/Tc +sspYPhs);
			z  = scurve(sspZin,sspZfi, walkTime-dsp1Time,sspTime) - hipLength/2;
			zr =  sspZAmp * cosh((walkTime-dsp1Time)/Tc + sspZPhs) -hipLength/2;
			phi= scurve(legRotin,legRotfi,walkTime-dsp1Time,sspTime);
			phiR=scurve(supLegRotin,supLegRotfi,walkTime-dsp1Time,sspTime);
			state = SSP;
	//		////printf("SSP0\t");
		}
		else if (walkTime > dsp1Time+sspTime )//&& walkTime<=stepTime
		{
			state = DSP;
			x  = height;// - dampLift* (sin(dampFreq*((walkTime-dampStart)/(dampEnd-dampStart))+dampPhase)+dampDisplacement)/(1+dampDisplacement);
			xr = height;
			y  = -sspYfi - veloYfi_d*(walkTime-dsp1Time-sspTime);
			yr = -sspYSupfi - veloYfi_d*(walkTime-dsp1Time-sspTime);
			z  = sspZfi - veloZfi*(walkTime-dsp1Time-sspTime) -hipLength/2;
			zr = sspZSupfi + veloZfi*(walkTime-dsp1Time-sspTime) - hipLength/2;
			phi= legRotfi;
			phiR= supLegRotfi;
	//		////printf("DSP2\t");
		}
		else 
		{
	//		////printf("************* SOMETHING WRONG*******************\n");
		}
		///////printf("X\t%3.1lf\tXR\t%3.1lf\tY\t%lf\tYR\t%lf\tZ\t%lf\tZR\t%lfP\t%lf\tPR\t%lf\n",x,xr,y,yr,z,zr,phi,phiR);
		////printf("Y\t%lf\tYR\t%lf\tZ\t%lf\tZR\t%lf\tP\t%lf\tPR\t%lf\n",y,yr,z+s,zr+sr,phi,phiR);
		///////printf("Z\t%lf\tZR\t%lf\n",z,zr);
		printf("W phi\t%lf\tphiR\t%lf\tZ\t%lf\tZR\t%lf\tY\t%lf\tYR\t%lf\n",phi,phiR,z,zr,y-corr,yr-corr);
		const double (&COM)[AXES] = bot->getRotCOM();
		//bot->printRotCOM(); 
		corr=1*(COM[1]-13);
		bot->storeCOM2(k);
		k=k+1;	
		bot->leg[leg]->runIK(x,y-corr,z+feetSeperation,phi);
		bot->leg[1-leg]->runIK(xr,yr-corr,zr+feetSeperation,phiR);
		//printf("%d %d\n\n\n",leg,1-leg);
		bot->updateBot();
		//printf("Sent Values\n");
		//leg->getMotorLoad(FOOT_ROLL);
		//revLeg->getMotorLoad(FOOT_ROLL);

		usleep(sleep);
		//getchar();
		
	}
	supLegYin  = -legYfi;
	legYin = -supLegYfi;
	veloYin=veloYfi_d;
	supLegZin = supLegZfi - hipLength/2;
	legZin = legZfi - hipLength/2;
	veloZin = -veloZfi;
	veloZfi= veloZfi;	
	supLegRotin  = legRotfi;
	legRotin = supLegRotfi;
	return EXIT_SUCCESS;//endState;
}

int Walk::dribble(double dy, double dx, double t1, double t2)
{
	leg=(LEG)(1-(int)leg);	
	printf("%d\t",leg);
	
	printf("X:%lf Y:%lf\n\n",dx,dy);
	
	printf("VYin\t%lf\tVyfi\t%lf\n",veloYin,veloYfi);
	int fps = 60;
	int sleep = 1000000.0/(double)fps;
	double timeInc =1.0/(double)fps;
	double feetSeperation = 0;
	double hipLength = 130;
	
	double Tc = sqrt(600.00/9810.0);
	legZin += hipLength/2;
	supLegZin += hipLength/2;
	
	
	legRotfi=t1;
	supLegRotfi=t2;
	///// desired Values 
	
	double D_dsp1Time = 0.05+dx/(2.0*veloZfi);
	double D_dsp2Time = 0.05+dx/(2.0*veloZfi);
	
	// ////printf("Tc\t\t%lf\n",Tc);
	// ////printf("legZin\t\t%lf\n",legZin);
	// ////printf("supLegZin\t%lf\n",supLegZin);
	
	//// Z Control 
	
	double sspZAmp   = zMax; 
	double sspZPhs   = asinh(veloZin*Tc/sspZAmp);
	double sspZTime  = Tc * (asinh(veloZfi*Tc/sspZAmp) - sspZPhs);
	double sspZSupin = sspZAmp * cosh(sspZPhs);
	double sspZSupfi = sspZAmp * cosh(sspZTime/Tc + sspZPhs);
	
	double sspTime   = sspZTime;
	double dsp1Time  = (supLegZin - sspZSupin)/(-veloZin);		
	double dsp2Time  = D_dsp2Time;
	double stepTime  = sspTime + dsp1Time + dsp2Time;
	
	double sspZin    = legZin + (-veloZin)*dsp1Time;
	double P_sspZAmp = zMax;
	double P_sspZPhs = asinh(-veloZfi*Tc/P_sspZAmp);
	double P_sspZin  = P_sspZAmp * cosh(P_sspZPhs);
	double P_legZin  = P_sspZin - (-veloZfi)* D_dsp2Time;
	double P_veloZfi = veloZfi + (veloZfi + veloZin)/2;
	double P_sspZTime= Tc * (asinh(P_veloZfi*Tc/P_sspZAmp) - P_sspZPhs);
	
	
	double sspZfi    = P_legZin + veloZfi*D_dsp1Time; 
	double legZfi    = sspZfi - veloZfi*dsp2Time;
	double supLegZfi = sspZSupfi + veloZfi*dsp2Time;
	
	//TODO add case in which dsp1Time becomes negative 
	//TODO add case where dsp2Time needs to be modified (if required)
	//TODO calculation of sspZfi for next step based on predicted change in velocity for the next step 
	
	////printf("zMax\t\t%lf\n",zMax);
	////printf("sspZPhs\t\t%lf\n",sspZPhs);
	////printf("sspZTime\t%lf\n",sspZTime);
	////printf("sspZin\t\t%lf\n",sspZin);
	////printf("sspZSupin\t%lf\n",sspZSupin);
	////printf("sspZfi\t\t%lf\n",sspZfi);
	////printf("sspZSupfi\t%lf\n",sspZSupfi);
	////printf("dsp1Time\t%lf\n",dsp1Time);
	////printf("stepTime\t%lf\n",stepTime);
	////printf("LegZfi\t\t%lf\n",legZfi);
	////printf("supLegZfi\t%lf\n",supLegZfi);
	
	// Y Control 
	
	legYin    = -legYin;
	supLegYin = -supLegYin;
	
	double sspYin    = legYin + veloYin * dsp1Time;
	double sspYSupin = supLegYin + veloYin * dsp1Time;
	double sspYAmp   = veloYin * sqrt(pow(Tc,2) - pow(sspYSupin/veloYin,2));
	double sspYPhs   = asinh((sspYSupin/veloYin)/sqrt(pow(Tc,2) - pow(sspYSupin/veloYin,2)));
	double veloYfi_d   = sspYAmp/Tc * cosh (sspTime/Tc + sspYPhs);
	//////printf("**************%lf******************\n",sspYAmp/Tc * cosh (sspTime/Tc + sspYPhs));
	//double P_sspYAmp = sgn(veloYin) * Tc * sqrt((2*cosh(P_sspZTime/Tc)*veloYfi_d*veloYfi - pow(veloYfi_d,2) - pow(veloYfi,2))/(pow(sinh(P_sspZTime/Tc),2)));
	//double P_sspYPhs = -acosh(veloYfi_d*Tc/P_sspYAmp);
	//double P_sspYin  = P_sspYAmp * sinh(P_sspYPhs);
	double P_sspYin  = (-sinh(P_sspZTime/Tc)*veloYfi_d*Tc + dy)/(cosh(P_sspZTime/Tc)-1);
	double P_sspYAmp = sgn(veloYfi_d)*sqrt(-pow(P_sspYin,2)+pow(veloYfi_d*Tc,2));
	double P_sspYPhs = asinh(P_sspYin/P_sspYAmp);
	double P_legYin  = P_sspYin - veloYfi_d*D_dsp1Time;
	double P_veloYfi= (P_sspYAmp/Tc)*cosh((P_sspZTime/Tc)+P_sspYPhs);
	double P_sspYfi = P_sspYAmp*sinh((P_sspZTime/Tc)+P_sspYPhs);

	double sspYfi    = P_legYin - veloYfi_d*dsp2Time;
	double sspYSupfi = sspYAmp * sinh(sspTime/Tc + sspYPhs);
	
	double legYfi    = sspYfi + veloYfi_d*dsp2Time;
	double supLegYfi = sspYSupfi + veloYfi_d*dsp2Time;
	
	printf("\n\n\n");
	printf("legYin\t\t%lf\n",legYin);
	printf("supLegYin\t%lf\n",supLegYin);
	printf("sspYin\t\t%lf\n",sspYin);
	printf("sspYSupin\t%lf\n",sspYSupin);
	printf("sspYAmp\t\t%lf\n",sspYAmp);
	printf("sspYPhs\t\t%lf\n",sspYPhs);
	printf("sspYfi\t\t%lf\n",sspYfi);
	printf("sspYSupfi\t%lf\n",sspYSupfi);
	printf("legYfi\t\t%lf\n",legYfi);
	printf("suplegYfi\t%lf\n",supLegYfi);
	printf("veloYfi_d\t%lf\n",veloYfi_d);
	
	printf("P_sspYAmp\t%lf\n",P_sspYAmp);
	printf("P_sspYPhs\t%lf\n",P_sspYPhs);
	printf("P_sspZTime\t%lf\n",P_sspZTime);
	printf("P_sspYin\t\t%lf\n",P_sspYin);
	printf("P_legYin\t\t%lf\n",P_legYin);
	printf("P_veloYfi\t\t%lf\n",P_veloYfi);
	printf("P_sspYfi\t\t%lf\n",P_sspYfi);
	// Y (cubic)
	
	double a = ((-veloYfi-veloYin)-2*(-sspYfi+sspYin)/sspTime)/pow(sspTime,2);
	double b = ((-sspYfi+sspYin)/sspTime+veloYin-a*pow(sspTime,2))/sspTime;
	double c = -veloYin;
	double d = -sspYin;
	
	double height = 390;
	//double lift   = 30;
	double xfreq  = 2*3.14;
	double displacement = 1;
	double startX = dsp1Time-1.0/60.0;
	double stopX  = stepTime-dsp2Time + 1.0/60.0;
	double xPhase =-3.14/2;
	
	double dampLift = 0;
	double dampFreq = 2*3.14;
	double dampStart = dsp1Time + sspTime - 4.0/60.0;
	double dampEnd   = stepTime;
	double dampPhase = -3.14/2;
	double dampDisplacement = 1;
	
	double fraction;
	
	double x,xr,y,yr,z,zr,phi,phiR,s,sr;
	double yReal,yrReal,zReal,zrReal;
	double walkTime =0.0;
	int walk_i = 0;
	int target_i = 0;
	double *fval;
	//compliance(leg,10,10);
	
	
	int state = DSP;
//	////printf("*************************************** STEP **********************************************\n");	
	for(walkTime = 0.0/fps; walkTime<=stepTime; walkTime +=timeInc)
	{
		if(walkTime < dsp1Time)
		{
			x  = height;
			xr = height;
			y  = -legYin - veloYin*walkTime;
			yr = -supLegYin - veloYin*walkTime;
			z  = legZin - veloZin*walkTime - hipLength/2;
			zr = supLegZin + veloZin*walkTime - hipLength/2;
			phi= legRotin; 
			phiR= supLegRotin;
	//		////printf("DSP1\t");
			state = DSP;
		}
		else if (walkTime >= dsp1Time && walkTime <= dsp1Time + sspTime)
		{
			fraction=2*(walkTime-startX)/(stopX-startX);
			//x  = height - lift * (sin(xfreq*((walkTime-startX)/(stopX-startX))+xPhase) + displacement)/(1+displacement);
			x = height- lift*fraction*(2-fraction);
			xr = height;
			y=a*pow(walkTime-dsp1Time,3)+b*pow(walkTime-dsp1Time,2)+c*(walkTime-dsp1Time)+d;
			//y  = linear(-sspYin,-sspYfi, ((walkTime-dsp1Time)/sspTime);
			yr = -sspYAmp * sinh((walkTime-dsp1Time)/Tc +sspYPhs);
			z  = scurve(sspZin,sspZfi, walkTime-dsp1Time,sspTime) - hipLength/2;
			zr =  sspZAmp * cosh((walkTime-dsp1Time)/Tc + sspZPhs) -hipLength/2;
			phi= scurve(legRotin,legRotfi,walkTime-dsp1Time,sspTime);
			phiR=scurve(supLegRotin,supLegRotfi,walkTime-dsp1Time,sspTime);
			state = SSP;
	//		////printf("SSP0\t");
		}
		else if (walkTime > dsp1Time+sspTime )//&& walkTime<=stepTime
		{
			state = DSP;
			x  = height;// - dampLift* (sin(dampFreq*((walkTime-dampStart)/(dampEnd-dampStart))+dampPhase)+dampDisplacement)/(1+dampDisplacement);
			xr = height;
			y  = -sspYfi - veloYfi_d*(walkTime-dsp1Time-sspTime);
			yr = -sspYSupfi - veloYfi_d*(walkTime-dsp1Time-sspTime);
			z  = sspZfi - veloZfi*(walkTime-dsp1Time-sspTime) -hipLength/2;
			zr = sspZSupfi + veloZfi*(walkTime-dsp1Time-sspTime) - hipLength/2;
			phi= legRotfi;
			phiR= supLegRotfi;
	//		////printf("DSP2\t");
		}
		else 
		{
	//		////printf("************* SOMETHING WRONG*******************\n");
		}
		///////printf("X\t%3.1lf\tXR\t%3.1lf\tY\t%lf\tYR\t%lf\tZ\t%lf\tZR\t%lfP\t%lf\tPR\t%lf\n",x,xr,y,yr,z,zr,phi,phiR);
		////printf("Y\t%lf\tYR\t%lf\tZ\t%lf\tZR\t%lf\tP\t%lf\tPR\t%lf\n",y,yr,z+s,zr+sr,phi,phiR);
		///////printf("Z\t%lf\tZR\t%lf\n",z,zr);
		printf("phi\t%lf\tphiR\t%lf\tZ\t%lf\tZR\t%lf\tY\t%lf\tYR\t%lf\n",phi,phiR,z,zr,y,yr);
		
		bot->leg[leg]->runIK(x,y,z+feetSeperation,phi);
		bot->leg[1-leg]->runIK(xr,yr,zr+feetSeperation,phiR);
		//printf("%d %d\n\n\n",leg,1-leg);
		bot->updateBot();
		//printf("Sent Values\n");
		//leg->getMotorLoad(FOOT_ROLL);
		//revLeg->getMotorLoad(FOOT_ROLL);

		usleep(sleep);
		//getchar();
		
	}
	supLegYin  = -legYfi;
	legYin = -supLegYfi;
	veloYin=veloYfi_d;
	supLegZin = supLegZfi - hipLength/2;
	legZin = legZfi - hipLength/2;
	veloZin = -veloZfi;
	veloZfi= veloZfi;	
	supLegRotin  = legRotfi;
	legRotfi=0;
	legRotin = supLegRotfi;
	supLegRotfi=0;
	return EXIT_SUCCESS;//endState;
}
