/**************************************************************************************************
Copyright, 2010-2012, Robotics Lab., Dept. of M.E., National Taiwan University
File Name: ConstIO.h

Author: Jiu-Lou Yan
Version: 1.0
Date: 2012/02/01

Functions:None

Classes: None

Description:
     �����Y�ɩw�q�F�Ҧ�CAN-Bus�q�T�����O�w�q�H�γq�T��w
	 �ԲӤ��e�P�w�q�аѷӤ������

Note: None
***************************************************************************************************/

// ID parameters
#define MID 0 // �Ӷb��ID 
#define CMD_SET 16 // �C�b

// �����аѷӤU���ª�
enum ID_Sets{ID_InitMech,ID_InitPara,ID_SetENC,ID_SetPID,ID_ModeIdle,ID_ModePID,ID_AllReturnENC,ReservedID1,ID_SetOneAxis,
	         ID_SetIdle,ID_SetCaliENC,ID_ReadEncoder,ReservedID2,ID_SetPWMLimit,ID_EndOfContTraj,ID_SetTrajPacket};

#define ID_SendReadENC		MID*CMD_SET+1
#define ID_ReadENCcount		MID*CMD_SET+9
#define ID_SetpotPID		MID*CMD_SET+11

//// �ª�
//// Master Output ID Command Definitions
//#define ID_InitMech		MID*CMD_SET+0 // ���c��l���k�s
//#define ID_InitPara		MID*CMD_SET+1 // ��l�ưѼ� �ثe���ݭn�ϥ�
//#define ID_SetENC		MID*CMD_SET+2 // �]�w��b�ؼ�ENCDOR��m
//#define ID_SetPID		MID*CMD_SET+3 // �]�w��bPID�Ѽ�
//#define ID_ModeIdle		MID*CMD_SET+4 // �]�w��b��IDLE MODE
//#define ID_ModePID		MID*CMD_SET+5 // �]�w��b��PID MODE
//#define ID_AllReturnENC MID*CMD_SET+6 // all motor nodes return their ENC counts
//#define ID_ReadEncoder  MID*CMD_SET+11 //Ū����bencoder
////////////////////////  MID*CMD_SET+7 7���w�g�Q�w�q���^�� //////////////////////
//#define ID_SetOneAxis	MID*CMD_SET+8 // �]�w��b�R�O -- ��C32���q�M��
//#define ID_SetIdle		MID*CMD_SET+9 // �]�wIDLE MODE -- ��C32���q�M��
//#define ID_SetCaliENC 	MID*CMD_SET+10 // �]�w���c�~�t���v��encoder�� 
//#define ID_SetPWMLimit		13 // �]PWM���W�U����
//#define ID_EndOfContTraj	14 // �i�DC32�s��y��w�g�ǧ�
//#define ID_SetTrajPacket	15 // �i�DC32�Y�N�}�l�s��ǰe�j�q�y��

// �����ЬݤU���ª�
enum SP_Sets{SP_IDLE,SP_PID,SP_INIT_MECH,SP_WATCH_NODES,SP_SetOneAxis,SP_SetIdle};
//// State Machine -- Primary State Definition
//#define SP_IDLE			0 // main state is idle
//#define SP_PID			1 // main state is pid
//#define SP_INIT_MECH	2 // main state is initial mechanism
//#define SP_WATCH_NODES  3 // the master node watches the slave nodes -- not used now
//#define SP_SetOneAxis	4 // set the position of one specified axis 
//#define SP_SetIdle		5 // set C32 IDLE

//#define PI 3.1415926  // �bGlobal Constant �]�w

