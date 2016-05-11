/**************************************************************************************************
Copyright, 2010-2012, Robotics Lab., Dept. of M.E., National Taiwan University
File Name: neck.h

Author: che-hsuan chang
Version: 1.0
Date: 2013/03/19 
Functions:
	    setvalue()
Classes: 
        neck
Description:
     ���{���D�n�Φb�p�⵹����l���c�������F����m�y��
	 �U�禡�P�ܼƤ������иԨ��U��ŧi�P�w�q�B������

Note: None
***************************************************************************************************/
#include "stdafx.h"
#include <stdio.h>
#include <iostream>
using namespace std;

class  neck 
{
public  :
	neck(void) ;  // �غc�l
	~neck(void) ; //�Ѻc�l
	void setvalue(int thetastart , int thetafinal  , int neck_point , int neck_motor , int neck_omega);
	int  theta;   // ���ת��y���
    int  thetastart; // ��l����
    int  thetafinal; // �̲ר��� 
    int  neck_point;  // ��l���׻P�̲ר��׶������t�I��
	int  neck_motor  ;  // �����l���c�����Findex  �W���F1  �U���F0
	int  neck_omega ;   //��ʳt��  300~ 500  �@��t��300  
	};

