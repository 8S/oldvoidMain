#include "commondefswalk.h"

int compareFloat(double num1, double num2, double precision)
{
	if(fabs(num1-num2)<precision)
	return true;
	else return false;	
}

int trun(double num, int precision)
{
	return round(pow(10,precision)*num)/pow(10,precision);
}


double scurve(double in,double fi,double t, double tot)
{
	double frac=(double)t/(double)tot;
	if(frac>1)
	frac=1;
	double retfrac=frac*frac*(3-2*frac);
	double mval=in + (fi-in)*retfrac;
	return mval;
}

double linear(double in,double fi,double t, double tot)
{
	if(t>=0&&t<=tot)
	return in*(1-t/tot)+fi*(t/tot);
	else if(t<0)
	return in;
	else
	return fi;
}

int sortDescending(double arr[], int length)
{
	for(int i=1;i<length;i++)
	{
		for(int j=0;j<length-i;j++)
		{
			if(arr[j]<arr[j+1])
			{
				double temp = arr[j+1];
				arr[j+1]=arr[j];
				arr[j] = temp;
			}
	
		}
		printArr(arr,length);
	}
}

int printArr(double arr[],int length)
{
	for(int i=0;i<length;i++)
	printf("%lf\t",arr[i]);
	printf("\n");
}

int isCCW(Point a, Point b, Point c)
{
	return (b.x-a.x)*(c.y-a.y) - (c.x-a.x)*(b.y-a.y);
}

int calcConvexHull(Point unSortedPoints[], int length)
{	
	if(length<=2)
	return EXIT_FAILURE;

	// Sorting according to Y and then X in ascending order	
	for(int i=1;i<length;i++)
	{
		for(int j=0;j<length-i;j++)
		{
			if(unSortedPoints[j].y>unSortedPoints[j+1].y)
			{
				Point temp = unSortedPoints[j];
				unSortedPoints[j]=unSortedPoints[j+1];
				unSortedPoints[j+1]=temp;
			}
			else if(unSortedPoints[j].y==unSortedPoints[j+1].y)
			{
				if(unSortedPoints[j].x>unSortedPoints[j+1].x)
				{
					Point temp = unSortedPoints[j];
					unSortedPoints[j]=unSortedPoints[j+1];
					unSortedPoints[j+1]=temp;
				}
			}
			
		}
	}
	
	// Jarvis March (Running counterclockwise)
	Point point[length+1];
	Point points[length+1];
	for(int i=0;i<length;i++)
	{
		point[i] = unSortedPoints[i];
		points[i] = unSortedPoints[i];
		//printf("%lf\t%lf\n",point[i].x,point[i].y);
	}
	point[length] = points[0];
	points[length] = points[0];
	int endPoint=1;
	double rotAng=0,oldRotAng=0;
	int i=0;
	while(points[endPoint]!= points[0] && i<length)
	{
		//printf("%d Current Point %3.3lf %3.3lf\n",i,points[i].x,points[i].y);
		Point temp;
		
		// Translating and rotating to make current side the X axis
		for(int j=i+1;j<length+1;j++)
		{
			temp.x = points[j].x-points[i].x;
			temp.y = points[j].y-points[i].y;
			point[j].y = trun(-temp.x*sin(rotAng) + temp.y*cos(rotAng),10);
			point[j].x = trun(temp.x*cos(rotAng) +temp.y*sin(rotAng),10);
			
			//printf("New Points %le\t%le\n",point[j].x,point[j].y);
			//printf("%d\n",point[j].x<0);
		}
		endPoint = i+1;	
		oldRotAng = rotAng;
		// Finding next point 
		for(int j=i+2;j<length+1;j++)
		{
			//printf("Candidate %d %lf %lf %lf %lf %lf\n",j,point[j].x,point[j].y,points[j].x,points[j].y,atan2(point[j].y,point[j].x));
			if(point[j].x==0 && point[j].y==0)
			{
				//printf("Continued\n");
				continue;
			}
			if(atan2(point[j].y,point[j].x)<atan2(point[endPoint].y,point[endPoint].x))
			{
				endPoint=j;
				rotAng = atan2(point[j].y,point[j].x) + oldRotAng;
			}
			else if(atan2(point[j].y,point[j].x)==atan2(point[endPoint].y,point[endPoint].x))
			{
				if(sqrt(pow(point[j].y,2)+pow(point[j].x,2))>sqrt(pow(point[endPoint].y,2)+pow(point[endPoint].x,2)))
				{
					endPoint = j;
					rotAng = atan2(point[j].y,point[j].x) + oldRotAng;
				}
			}
		}
		//printf("I %d\tEndPoint %d RotAng %lf\n",i,endPoint,rotAng-oldRotAng);
		temp = points[i+1];
		points[i+1] = points[endPoint];
		points[endPoint] = temp;
		endPoint=i+1;
		
		i++;
		//printf("endPoint %d the poitns s%3.3lf\t%3.3lf\n",i,point[i].x,point[i].y);		
		//printf("LOOP %d \n\n\n\n\n\n",points[endPoint]!= points[0]);
	}
	///  Jarvis March over
	
	//printf("%d polyGon points are:\n",i);
	for(int j=0;j<i;j++)
	{
		unSortedPoints[j] = points[j];
		//printf("Original %3.3lf\t%3.3lf\n",points[j].x,points[j].y);
	}
	
	return i;
}


const Point getPolyCentroid(const pointArray poly_)
{
	double a=0;
	Point cen;
	cen.x=0;
	cen.y=0;
	
	pointArray poly = poly_;
	poly.point[poly.length] = poly_.point[0];
	
	//printf("poly.l %d\n",poly.length);
	//for(int i=0;i<poly.length;i++)
	//printf("%lf\t%lf\n",poly.point[i].y,poly.point[i].x);
	
	for(int i=0;i<poly.length;i++)
	{
		a+= (poly.point[i].x*poly.point[i+1].y - poly.point[i+1].x*poly.point[i].y);
		cen.x += (poly.point[i].x+poly.point[i+1].x)*(poly.point[i].x*poly.point[i+1].y - poly.point[i+1].x*poly.point[i].y);
		cen.y += (poly.point[i].y+poly.point[i+1].y)*(poly.point[i].x*poly.point[i+1].y - poly.point[i+1].x*poly.point[i].y);
	
	}
	a *=0.5;
	cen.x = cen.x/(6*a);
	cen.y = cen.y/(6*a);
	//printf("A %lf\n",a);
	//printf("CenX %lf\n",cen.x);
	//printf("CenY %lf\n",cen.y);
	
	return cen;
}
