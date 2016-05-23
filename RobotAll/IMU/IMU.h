#include <stdio.h>
#include <windows.h>
#include <sys/timeb.h>
#include <time.h>
#include <fstream>
#include <iostream>



#define PI 3.1415926535828 
#define IMUbuffersize 10000
using namespace std;
extern "C"       //include c library �S��g�k
{
	#include "i3dmgx3_Errors.h"
	#include "i3dmgx3_Cont.h"
	#include "i3dmgx3_Utils.h"
	#include "i3dmgx3_Utils_CM.h"
	#include "i3dmgx3_readWriteDR.h"
	#include "i3dmgx3_Serial.h"
}

#ifndef _IMU_H_
#define _IMU_H_
#pragma once



class IMU {

public:
	
	IMU (); // �غc�l
	~IMU (); // �Ѻc�l


public:	

	
	int absangle ; //  flag ���﨤��
	int count   ;
	int datacount  ;    //�s�U��data��
	int calicount  ;   //cali������
	int adamssimcount ; // adams counts start from 1000
	double Ts ;
	float  ang1,ang2; 
	float microstraincalix , microstraincaliy  ;  //cali��  �P microstrain MIP ���
	bool IMU_plot;
	double AngularVel_bound ;




	float anglex[IMUbuffersize];        // vel �n��
	float angley[IMUbuffersize];
	float anglez[IMUbuffersize];
	
	float absanglex [IMUbuffersize] ;  //abs ���﨤��
	float absangley [IMUbuffersize] ;
	float absanglez [IMUbuffersize] ;
	
	float finalanglex[IMUbuffersize];   //�����Hroll��V
	float finalangley[IMUbuffersize];   //�����Hpitch��V
	float finalanglez[IMUbuffersize];
	

	float anglefilterx[IMUbuffersize];
	float anglefiltery[IMUbuffersize];
	float anglefilterz[IMUbuffersize];//20140921 dora add

	float filterx [IMUbuffersize];
	float filtery [IMUbuffersize];
	float filterz [IMUbuffersize];
	
	float filterabsx [IMUbuffersize];
	float filterabsy [IMUbuffersize];
	float filterabsz [IMUbuffersize];
	
	float finalanglex_test[IMUbuffersize];

	float AngularVelx [IMUbuffersize];	
	float AngularVely [IMUbuffersize];	
	float AngularVelz [IMUbuffersize];	
	
	float accelx[IMUbuffersize];
	float accely[IMUbuffersize];
	float accelz[IMUbuffersize];
	
	
	float magx[IMUbuffersize];
	float magy[IMUbuffersize];
	float magz[IMUbuffersize];

	float M11[IMUbuffersize];
	float M12[IMUbuffersize];
	float M13[IMUbuffersize];
	float M21[IMUbuffersize];
	float M22[IMUbuffersize];
	float M23[IMUbuffersize];
	float M31[IMUbuffersize];
	float M32[IMUbuffersize];
	float M33[IMUbuffersize];



	double waisttheta[IMUbuffersize];
	double waistthetadot[IMUbuffersize];


	//------------------------variables---------------------------//
	float  AngularVelbiasx , AngularVelbiasx1 , AngularVelbiasx2 ;
	float  AngularVelbiasy , AngularVelbiasy1 , AngularVelbiasy2 ;
	float  AngularVelbiasz , AngularVelbiasz1 , AngularVelbiasz2 ;// AngularVel cali�q


	float  anglebiasx ; 
	float  anglebiasy ;
	float  anglebiasz ; 

	float anglexbias  ;
	float angleybias  ; 
	float absxbias    ;
	float absybias    ;

	float abscalix    ;
	float abscaliy    ;
	float anglecalix  ;  
	float anglecaliy  ;  
	float anglecaliz  ;  

    ////// FOR IMU rotation matrix 
	float IMU_roll;
	float IMU_pitch;
	float IMU_yaw;


//------------------------------------------------ ����function-----------------------------------------------------------//
	 void ReadContinuousData(int portNum); 
	 void integation(float *AngularVelx ,float *anglex , float *AngularVely ,float *angley ,float *AngularVelz ,float *anglez );
	 void complementaryfilter(float* ang ,  float * abs , float * filterang , float *filterabs , float *filter , float bias  );
	 void adamssim(int gFlagSimulation,int adamssimcount);
	 bool IMU_Lock;
 //------------------------------------------------ ����function-----------------------------------------------------------//




}  ;

#endif

