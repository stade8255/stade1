/**************************************************************************************************
Copyright, 2010-2012, Robotics Lab., Dept. of M.E., National Taiwan University
File Name: DataProcess.h

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

#pragma once

#include <iostream>
using namespace std;
class DataProcess
{
public:
	DataProcess(void); // �غc�l
public:
	~DataProcess(void); // �Ѻc�l

public:
	void DPInit(void); // ��l�ƪ���
	unsigned char State[5];
	unsigned char TxData[9];
	void TransENC(long double TarENC,unsigned int TXID); // �ǿ�encoder�� �H�εo�e��HID

};
