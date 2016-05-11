/**************************************************************************************************
Copyright, 2010-2012, Robotics Lab., Dept. of M.E., National Taiwan University
File Name: DataProcess.cpp

Author: Jiu-Lou Yan
Version: 1.0
Date: 2010/06/20

Functions:
	DataProcess() ~DataProcess() DPInit() TransENC()

Classes: DataProcess

Description:
     ���{���D�n��ƶǿ骫��
	 �ثe�u��encoder��X�\�� �D�n�ഫ�q���Plocal�ݪ�encoder��
	 �U�禡�P�ܼƤ������иԨ��U��ŧi�P�w�q�B������

Note: None
***************************************************************************************************/

#include "StdAfx.h"
#include "DataProcess.h"
#include "ConstIO.h"
#include <iostream>
#include "math.h"
using namespace std;
DataProcess::DataProcess(void)
{
	/******************************************************************
	input: void
	output: void

	Note:
	// Class constructor 
	******************************************************************/
}

DataProcess::~DataProcess(void)
{
	/******************************************************************
	input: void
	output: void

	Note:
	// Class destructor 
	******************************************************************/
}
void DataProcess::DPInit(void)
{
	/******************************************************************
	input: void
	output: void

	Note:
	// ��l���ܼ� State = [255 0 0 0 255]
	// �o�O�ΨӻPC32���q�Ϊ�5 byte �ܼ�
	******************************************************************/
		State[0]=0xff;
		State[1]=0x00;
		State[2]=0x00;
		State[3]=0x00;
		State[4]=0xff;
}

void DataProcess::TransENC(long double TarENC,unsigned int TxID)
{
	/******************************************************************
	input: TarENC �ؼжb��encoder�R�O  TxID �ؼжb��ID
	output: void

	Note:
	// �]�w��bencoder��m ����F��m��
	******************************************************************/

		TxData[0]=TxID;
		if (TarENC > 0)
		{
			// 32bit��ƳQ�4 bytes �~�ۮe��CAN-Bus�榡
			TxData[1] = (signed long)TarENC;
            TxData[2] = (signed long)TarENC>>8;
			TxData[3] = (signed long)TarENC>>16;
			TxData[4] = (signed long)TarENC>>24;
			TxData[5] = TxData[6] = TxData[7] = 0;	
			TxData[8] = 1;
		}
		else if (TarENC == 0)
		{
			TxData[1] = TxData[2] = TxData[3] = TxData[4] = TxData[5] = TxData[6] = TxData[7] = 0;
			TxData[8] = 1; 
		}
		else
		{
			// 32bit��ƳQ�4 bytes �~�ۮe��CAN-Bus�榡
			TxData[1] = (signed long)(-TarENC);
			TxData[2] = (signed long)(-TarENC)>>8;
			TxData[3] = (signed long)(-TarENC)>>16;
			TxData[4] = (signed long)(-TarENC)>>24;
			TxData[5] = TxData[6] = TxData[7] = 0;	
			TxData[8] = 1; 	

			TxData[4] = 255-TxData[4]; // �i�����O�t��
		}

}
