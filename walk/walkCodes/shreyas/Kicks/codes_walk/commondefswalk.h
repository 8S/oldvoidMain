#ifndef _COMMON_DEFS_
#define _COMMON_DEFS_

#include <inttypes.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/time.h>
#include <unistd.h>
#include <iostream>
#include "bot1.h"
//#include <ncurses.h>
//#include <cstdlib>

#define byte uint8_t
#define PI 3.14159265359
#define rad2deg(x) ((x)*180.0/(PI))
#define deg2rad(x) (((x)*(PI))/180.0)
#define sgn( a ) ( ((a) > 0) ? (1) : ( ((a) < 0) ? -1 : 0 ) )
#define AXES 3
//#define DEBUG

struct Point{
	float x;
	float y;
	
	int (operator!=)(const Point par)
	{
		if(x==par.x && y==par.y)
		return false;
		else
		return true;
	
	}
};
struct pointList{
	Point point;
	pointList *nxtPoint;
};

struct pointArray{
	Point point[FEET_CORNERS*2];
	int length;
};


int compareFloat(double num1, double num2, double precision = 0.00001);
int sortAscending(double arr[], int length);
int sortDescending(double arr[], int length);
int printArr(double arr[], int length);
int calcConvexHull(Point points[], int length);
const Point getPolyCentroid(const pointArray poly);
int isCCW(Point a, Point b, Point c);
double scurve(double in,double fi,double t, double tot);
double linear(double in,double fi,double t, double tot);

#endif

