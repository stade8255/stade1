/**************************************************************************************************
Copyright, 2010-2012, Robotics Lab., Dept. of M.E., National Taiwan University
File Name: neck.h

Author: che-hsuan chang
Version: 1
Date: 2013/03/19 
Functions:
	    setvalue()
Classes: 
        neck
Description:
     ���{���D�n�Φb�p�⵹����l���c�������F����m�y��
	 �U�禡�P�ܼƤ������иԨ��U��ŧi�P�w�q�B������

Note: None

//��lCOM57
//�y COM56

***************************************************************************************************/

#include "stdafx.h"
#include "serial_port.h"

#ifndef NECK_H
#define NECK_H
#include "neck.h"
#endif

//extern SerialPort *gpPortneck ; 


neck :: neck (void) {
      theta = 0    ;
      thetastart = 0 ; 
      thetafinal = 0  ;   
      neck_point = 0 ;   // thetastart  ��  thetafinal  ���t�I�� 
	  neck_omega = 0  ;  //��ʳt��  300~ 500  �@��t��300  
	  neck_motor = 0    ;//�W���F1  �U���F0
      extern SerialPort *gneck; 
      
}




void neck :: setvalue(int  thetastart  , int thetafinal , int neck_point  ,int  neck_motor , int neck_omega){
	/******************************************************************
	input:  (int  thetastart  , int thetafinal , int neck_point  ,int  neck_motor , int neck_omega)
	output: void

	Note:
	
	******************************************************************/
	
    extern SerialPort *gneck; 	
	
	for (int yy = 0 ;  yy<= neck_point ; yy++)
	{   
		
		theta  = thetastart + (((thetafinal - thetastart)/ neck_point) * yy)     ;
	    
	   
	char s[40];
	sprintf(s,"# %d P %d  S %d   \r  ", neck_motor , theta , neck_omega );
	
	
	printf("%d %d  \n "  , theta , neck_motor);
	
	unsigned char buffer[40]; 
	for( int i=0;i<strlen(s);i++)
		buffer[i]= s[i];
	gneck ->write(buffer,strlen(s));
	
		
}
}

neck ::~neck (void) {
}