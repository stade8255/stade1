
#include <cstdlib>
#include <iostream>
#include <fstream>
#include <cmath>
#include "plane.h"
#include <vector>
#include "QuaternionTrans.hpp"

#ifndef estimate_H
#define estimate_H
using namespace std;
using namespace Kalman;
#define Estimatebuffersize 12000

class estimate {

public:
    
	estimate(); // 建構子
	~estimate (); // 解構子

	int compute ( double cog, double *zmp, double cogaccel,int estimatecount, int direction ) ;
	int rcompute ( float anglex ,float angley, float anglez, float angularratex, float angularratey, float angularratez, float cogaccelx, float cogaccely, float cogaccelz, float &filtercogaccelx, float &filtercogaccely, float &filtercogaccelz, int estimatecount);
	
	double  Cogstatelateral[Estimatebuffersize];
	double  Dcogstatelateral[Estimatebuffersize];
	double  state3lateral[Estimatebuffersize];

	double  Cogstatesaggital[Estimatebuffersize];
	double  Dcogstatesaggital[Estimatebuffersize];
	double  state3saggital[Estimatebuffersize];

	double AngleX[Estimatebuffersize];
	double AngleY[Estimatebuffersize];
	double AngleZ[Estimatebuffersize];
	
	bool estimate_lock ;
	bool initflag;
	bool rinitflag;



};


#endif
