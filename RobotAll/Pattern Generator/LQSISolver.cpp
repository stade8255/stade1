/**************************************************************************************************
Copyright, 2010-2012, Robotics Lab., Dept. of M.E., National Taiwan University
File Name: LQSISolver.cpp

Author: Jiu-Lou Yan
Version: 1.0
Date: 2010/07/20

Functions:
     DiffEqn() DiffEqnShift() LowPassFilter() GetMTSAcceleration()
     MovingAvergeSmooth() LQSISolver()  ~LQSISolver() Initval()
	 BackRiccati() C2DmHumanoid() DummyControl()
	 ZMPFeedbackControl() tic()toc()toc2()

Classes: LQSISolver

Description:
     ���{���D�n�Φb�ѥX���˳��\�ҫ���inverted pendulum ���D
	 ��JZMP�H��COG���׭y�� �N�i�H�ѥX������VZMP�\�ʤ��y��
	 �U�禡�P�ܼƤ������иԨ��U��ŧi�P�w�q�B������

Note: None
***************************************************************************************************/

#include "stdafx.h"
#include <fstream>
#include "LQSISolver.h"

extern double MaxIterationTime; //Parameter for Computational Speed Calculation

LQSISolver::LQSISolver()
{
	/******************************************************************
	input: void
	output: void

	Note:
	// Class constructor  ��l�ƩҦ��ݭn�Ψ쪺�ܼƻP�x�}
	******************************************************************/
	for (int i = 0; i<LQSIBufferSize ; i++)
	{
		AdStk[i].InitPara(3,3);
		AIStk[i].InitPara(3,3);
		BdStk[i].InitPara(3,1);
		BinvMBT_Stk[i].InitPara(3,3);
		WkStk[i].InitPara(3,3);
		NkStk[i].InitPara(3,3);
		SsStk[i].InitPara(3,3);
		vsStkX[i].InitPara(3,1);
		vsStkY[i].InitPara(3,1);
		invDkStk[i].InitPara(3,3);
		XState[i].InitPara(3,1);
		YState[i].InitPara(3,1);
	}

	SsStk[int(LQSIBufferSize)].InitPara(3,3); 
	vsStkX[int(LQSIBufferSize)].InitPara(3,1);
	vsStkY[int(LQSIBufferSize)].InitPara(3,1);
	XState[int(LQSIBufferSize)].InitPara(3,1);
	YState[int(LQSIBufferSize)].InitPara(3,1);

	CompuTemp = new double[400]; // �x�}���k���ݭn���ƧQ�Ϊ�function�ݭn��ӼȦs�� ���M�|�мg�쥿�b�Ϊ����
	CompuTemp2 = new double[400];
	CompuTemp3 = new double[400];
	CompuTemp4 = new double[400];

	Qx = new YMatLite[1];
	Qx->InitPara(3,3);

	Qx->data = new double[Qx->MSize];
	Qx->data[0] = 10;
	Qx->data[1] = 0;
	Qx->data[2] = 0;
	Qx->data[3] = 0;
	Qx->data[4] = 0;
	Qx->data[5] = 0;
	Qx->data[6] = 0;
	Qx->data[7] = 0;
	Qx->data[8] = 0;

	//Qx = 10, 0, 0,
	//	    0, 0, 0,
	//	    0, 0, 0;


	Cd = new YMatLite[1];
	Cd->InitPara(1,3);

	Cd->data[0] = 0;
	Cd->data[1] = 0;
	Cd->data[2] = 1;

	CdT = new YMatLite[1];
	CdT->InitPara(3,1);
	CdT->data[0] = 0;
	CdT->data[1] = 0;
	CdT->data[2] = 1;

	Dc = 0;
	Dd = 0;

	LQDataLen = LQSIBufferSize; // Back Riccati Length

	// initial ���|�ܪ���
	for (int i = 0; i< LQDataLen ; i++)
	{
		AdStk[i].data[6] = 0;
		AdStk[i].data[7] = 0;
		AdStk[i].data[8] = 1;
		BdStk[i].data[2] = dt;
	}

	if (QueryPerformanceFrequency(&gnFreq))
		gFreqT = (float)(gnFreq.QuadPart);
	
	if (QueryPerformanceFrequency(&gnFreq_PMS))
		gFreqT_PMS = (float)(gnFreq_PMS.QuadPart);

	TimeCount=0;
	TimeCountPMS = 0;
}

LQSISolver::~LQSISolver(void)
{
	/******************************************************************
	input: void
	output: void

	Note:
	// Class destructor 
	******************************************************************/

	// �M���ʺA�O����
	delete[] Cd;
	delete[] CdT;
	delete[] Qx;

	delete[] CompuTemp;
	delete[] CompuTemp2;
	delete[] CompuTemp3;
	delete[] CompuTemp4;

}

void LQSISolver::Initval(double* InputCOGz)
{
	/******************************************************************
	input: InputCOGz
	output: void

	Note:
	// �q�~����JCOG���׭y�� �o�ˤ~���X�Ҧ��˳��\�ҫ��b�C�@���������A�x�}
	******************************************************************/

	int LenTemp = LQDataLen+8;
	double SampTime = dt;
								
	COGz = InputCOGz;

	// �ﭫ�߰��׭y��L��
	// �ɶ����ܵu ��10^-5
	DiffEqn(COGz,&LenTemp,delCOG,&SampTime);
	LenTemp = LQDataLen+4;
	DiffEqn(delCOG,&LenTemp,ddelCOG,&SampTime);
	// �ɶ����ܵu ��10^-5

	int cnt = 0;

	for (int i = 0 ; i < LQDataLen ; i++)
	{
		if (ddelCOG[i] <= (-GravityConst))
		{
			cnt += 1;
			ddelCOG[i] = (-GravityConst)+1;
			//printf("index = %d \n",i);
		}
	}

	if (cnt > 0)
	{
		printf("�Фp�� �� %d ��COG�[�t�׶W�L 9810mm/s^2 �Ъ`�N!! �ȳQ�j��令-9809�F",cnt);
	}



}


void LQSISolver::BackRiccati(double* ZMPx, double* ZMPy)
{
	/******************************************************************
	input: ZMPx ZMPy, ZMP�b������V���y�� �o�OLQSI controller�� reference input
	output: void

	Note:
	// ��JMZPx ZMPy��LQSI controller��@reference input
	******************************************************************/

	YMatLite EYE3(3,3);
	EYE3.data[0] = 1;
	EYE3.data[1] = 0;
	EYE3.data[2] = 0;
	EYE3.data[3] = 0;
	EYE3.data[4] = 1;
	EYE3.data[5] = 0;
	EYE3.data[6] = 0;
	EYE3.data[7] = 0;
	EYE3.data[8] = 1;

	// ���Xzero order hold �� ���A�x�}(state-space matrices)
	// 0.24ms --> 0.13ms
	for (int i = 0 ; i < LQDataLen ; i++)
	{
		C2DmHumanoid(i);
	}
	// 0.24ms


	//tic();

	// LQSI�����ܼ� AI = Ad - (identity matrix 3-by-3)
	// 0.2ms --> 0.018ms
	for (int i = 0; i < LQDataLen ; i++)
	{
		AIStk[i].data[0] = AdStk[i].data[0]-1;
		AIStk[i].data[1] = AdStk[i].data[1];
		AIStk[i].data[2] = AdStk[i].data[2];
		AIStk[i].data[3] = AdStk[i].data[3];
		AIStk[i].data[4] = AdStk[i].data[4]-1;
		AIStk[i].data[5] = AdStk[i].data[5];
		AIStk[i].data[6] = AdStk[i].data[6];
		AIStk[i].data[7] = AdStk[i].data[7];
		AIStk[i].data[8] = AdStk[i].data[8]-1;
	}
	// 0.2ms

	// �p��LQSI Control Law~ 
	// 0.08ms
	for (int i = 0; i < LQDataLen ; i++)
	{
		// ��l equation
		//gTempM = BdT_stk[i]*Qx*BdStk[i]; 
		//invMkStk[i] = 1.0/(gTempM.data()[0]+R); // invMkStk(1,i) = 1/(Bd'*Qx*Bd+R);

		MatMulAtB(BdStk[i].data,3,1,Qx->data,3,3,CompuTemp);
		MatMulAB(CompuTemp,1,3,BdStk[i].data,3,1,CompuTemp2);
		invMkStk[i] = 1.0/(CompuTemp2[0]+R); // invMkStk(1,i) = 1/(Bd'*Qx*Bd+R);

		//temp11 = (BdStk[i]>Qx[0])*BdStk[i];
		//invMkStk[i] = 1.0/(temp11.data[0]+R); // invMkStk(1,i) = 1/(Bd'*Qx*Bd+R);

	}
	//0.08ms


	// �p��LQSI Control Law~ 
	for (int i = 0; i < LQDataLen ; i++)
	{
		// 0.1ms
		MatScalarMul(BdStk[i].data,3,invMkStk+i,CompuTemp);
		MatMulABt(CompuTemp,3,1,BdStk[i].data,3,1,BinvMBT_Stk[i].data);
		// 0.1ms
	}

	// �p��LQSI Control Law~ 
	for (int i = 0; i < LQDataLen ; i++)
	{
		// 0.31ms
		MatMulAB(BinvMBT_Stk[i].data,3,3,Qx->data,3,3,CompuTemp);
		MatMulAB(CompuTemp,3,3,AIStk[i].data,3,3,CompuTemp2);
		MatMiuAB(AdStk[i].data,CompuTemp2,WkStk[i].data,9);
	}


	// �p��LQSI Control Law~ 
	// 0.24ms
	for (int i = 0; i < LQDataLen ; i++)
	{
	   // NkStk[i] = WkStk[i] - EYE3;
		NkStk[i].data[0] = WkStk[i].data[0]-1;
		NkStk[i].data[1] = WkStk[i].data[1];
		NkStk[i].data[2] = WkStk[i].data[2];
		NkStk[i].data[3] = WkStk[i].data[3];
		NkStk[i].data[4] = WkStk[i].data[4]-1;
		NkStk[i].data[5] = WkStk[i].data[5];
		NkStk[i].data[6] = WkStk[i].data[6];
		NkStk[i].data[7] = WkStk[i].data[7];
		NkStk[i].data[8] = WkStk[i].data[8]-1;

	}
	// 0.24ms 0.188

	double tempScale = 0;

	// �p��LQSI Control Law~ 
	/////// �o�̥i�H�� �w����n���̲׭Ȩ��N
	tempScale = P;

	vsStkX[(int)(LQDataLen)].data[0] = CdT->data[0]*(P*ZMPx[(int)(LQDataLen-1)]); // ���w�̫᪺��(backward riccati���}�l)
	vsStkX[(int)(LQDataLen)].data[1] = CdT->data[1]*(P*ZMPx[(int)(LQDataLen-1)]); // ���w�̫᪺��(backward riccati���}�l)
	vsStkX[(int)(LQDataLen)].data[2] = CdT->data[2]*(P*ZMPx[(int)(LQDataLen-1)]); // ���w�̫᪺��(backward riccati���}�l)

	vsStkY[(int)(LQDataLen)].data[0] = CdT->data[0]*(P*ZMPy[(int)(LQDataLen-1)]); // ���w�̫᪺��(backward riccati���}�l)
	vsStkY[(int)(LQDataLen)].data[1] = CdT->data[1]*(P*ZMPy[(int)(LQDataLen-1)]); // ���w�̫᪺��(backward riccati���}�l)
	vsStkY[(int)(LQDataLen)].data[2] = CdT->data[2]*(P*ZMPy[(int)(LQDataLen-1)]); // ���w�̫᪺��(backward riccati���}�l)

	MatScalarMul(Cd->data,3,&tempScale,CompuTemp);
	MatMulAB(CdT->data,3,1,CompuTemp,1,3,SsStk[(int)(LQDataLen)].data);

	/////// �o�̥i�H�� �w����n���̲׭Ȩ��N

	// �p��LQSI Control Law~ 
	// 88ms original
	for (int i = (int)(LQDataLen-1) ; i >=0 ; i--)
	{
		// MATLAB code
		////invDkStk(:,:,i) = inv(eye(3)+BinvMBStk(:,:,i)*Ss(:,:,i+1));
		////Ss(:,:,i) = Nk'*Qx*Nk+(invMkStk(1,i)*invMkStk(1,i)*R)*(AI'*Qx*Bd*Bd'*Qx*AI)+CQC+Ad'*Ss(:,:,i+1)*invDkStk(:,:,i)*Wk;
		////vs(:,i) = Cd'*Q*ZMPr(i)+Ad'*(eye(3)-Ss(:,:,i+1)*invDkStk(:,:,i)*BinvMBStk(:,:,i))*vs(:,i+1);
		// MATLAB code

		//invDkStk[i] = EYE3+BinvMBT_Stk[i]*SsStk[i+1]; // �U����檺�즡		
		MatMulAB(BinvMBT_Stk[i].data,3,3,SsStk[i+1].data,3,3,CompuTemp);
		MatAddAB(EYE3.data,CompuTemp,invDkStk[i].data,9);

		InvSqMat(invDkStk[i].data,3); // Take Inverse: call by reference ���ݭn�t�~�[�t

		// Ss �즡
		//SsStk[i] = NkTStk[i]*Qx*NkStk[i]+((invMkStk[i]*invMkStk[i]*R)*(AIT_stk[i]*Qx*BdStk[i]))*(BdT_stk[i]*Qx*AIStk[i]) + CdT*Q*Cd + AdT_stk[i]*SsStk[i+1]*invDkStk[i]*WkStk[i];

		// ��� Ss, �٤U 50ms
		MatMulAtB(NkStk[i].data,3,3,Qx->data,3,3,CompuTemp);
		MatMulAB(CompuTemp,3,3,NkStk[i].data,3,3,CompuTemp2); // �⧹ N'QxN �s�b CompuTemp2

		MatMulAB(Qx->data,3,3,BdStk[i].data,3,1,CompuTemp);
		MatMulAtB(AIStk[i].data,3,3,CompuTemp,3,1,CompuTemp3);

		tempScale = invMkStk[i]*invMkStk[i]*R;
		MatScalarMul(CompuTemp3,3,&tempScale,CompuTemp4);
		MatMulABt(CompuTemp4,3,1,CompuTemp3,3,1,CompuTemp); // �⧹���q�A�s�b CompuTemp

		//MatScalarMul(CompuTemp3,3,&tempScale,CompuTemp3);
		//MatMulAtB(BdStk[i].data(),3,1,Qx.data(),3,3,CompuTemp);
		//MatMulAB(CompuTemp,1,3,AIStk[i].data(),3,3,CompuTemp4);
		//MatMulAB(CompuTemp3,3,1,CompuTemp4,1,3,CompuTemp); // �⧹���q�A�s�b CompuTemp

		MatAddAB(CompuTemp,CompuTemp2,CompuTemp,9); // N'QN + ���q�A�s�b CompuTemp

		tempScale = Q;
		MatScalarMul(Cd->data,3,&tempScale,CompuTemp2);
		MatMulAtB(Cd->data,1,3,CompuTemp2,1,3,CompuTemp3); // �⧹ C'QC

		MatAddAB(CompuTemp,CompuTemp3,CompuTemp,9); // N'QN + ���q + C'QC�A�s�b CompuTemp

		MatMulAtB(AdStk[i].data,3,3,SsStk[i+1].data,3,3,CompuTemp2);
		MatMulAB(CompuTemp2,3,3,invDkStk[i].data,3,3,CompuTemp3);
		MatMulAB(CompuTemp3,3,3,WkStk[i].data,3,3,CompuTemp2);

		MatAddAB(CompuTemp,CompuTemp2,SsStk[i].data,9); // �⧹ Ss�A�s�b SsStk[i]
		// ��� Ss

		//// ��� vs �٤U30ms
		////vs_Stk[i] = CdT*Q*Input_ZMP[i]+AdT_stk[i]*(EYE3-SsStk[i+1]*invDkStk[i]*BinvMBT_Stk[i])*vs_Stk[i+1];

		//tempScale = Q*Input_ZMP[i];
		//MatScalarMul(Cd->data,3,&tempScale,CompuTemp); // C'Qr

		//MatMulAB(BinvMBT_Stk[i].data,3,3,vs_Stk[i+1].data,3,1,CompuTemp2);
		//MatMulAB(invDkStk[i].data,3,3,CompuTemp2,3,1,CompuTemp3);
		//MatMulAB(SsStk[i+1].data,3,3,CompuTemp3,3,1,CompuTemp2);

		//MatMiuAB(vs_Stk[i+1].data,CompuTemp2,CompuTemp3,3);
		//MatMulAtB(AdStk[i].data,3,3,CompuTemp3,3,1,CompuTemp2); // �⧹��b

		//MatAddAB(CompuTemp,CompuTemp2,vs_Stk[i].data,3);
		//// ��� vs

	}

	// for ZMPx
	for (int i = (int)(LQDataLen-1) ; i >=0 ; i--)
	{
		// ��� vs �٤U30ms
		//vs_Stk[i] = CdT*Q*Input_ZMP[i]+AdT_stk[i]*(EYE3-SsStk[i+1]*invDkStk[i]*BinvMBT_Stk[i])*vs_Stk[i+1];

		tempScale = Q*ZMPx[i];
		MatScalarMul(Cd->data,3,&tempScale,CompuTemp); // C'Qr

		MatMulAB(BinvMBT_Stk[i].data,3,3,vsStkX[i+1].data,3,1,CompuTemp2);
		MatMulAB(invDkStk[i].data,3,3,CompuTemp2,3,1,CompuTemp3);
		MatMulAB(SsStk[i+1].data,3,3,CompuTemp3,3,1,CompuTemp2);

		MatMiuAB(vsStkX[i+1].data,CompuTemp2,CompuTemp3,3);
		MatMulAtB(AdStk[i].data,3,3,CompuTemp3,3,1,CompuTemp2); // �⧹��b

		MatAddAB(CompuTemp,CompuTemp2,vsStkX[i].data,3);
		// ��� vs
	}

	// for ZMPy
	for (int i = (int)(LQDataLen-1) ; i >=0 ; i--)
	{
		// ��� vs �٤U30ms
		//vs_Stk[i] = CdT*Q*Input_ZMP[i]+AdT_stk[i]*(EYE3-SsStk[i+1]*invDkStk[i]*BinvMBT_Stk[i])*vs_Stk[i+1];

		tempScale = Q*ZMPy[i];
		MatScalarMul(Cd->data,3,&tempScale,CompuTemp); // C'Qr

		MatMulAB(BinvMBT_Stk[i].data,3,3,vsStkY[i+1].data,3,1,CompuTemp2);
		MatMulAB(invDkStk[i].data,3,3,CompuTemp2,3,1,CompuTemp3);
		MatMulAB(SsStk[i+1].data,3,3,CompuTemp3,3,1,CompuTemp2);

		MatMiuAB(vsStkY[i+1].data,CompuTemp2,CompuTemp3,3);
		MatMulAtB(AdStk[i].data,3,3,CompuTemp3,3,1,CompuTemp2); // �⧹��b

		MatAddAB(CompuTemp,CompuTemp2,vsStkY[i].data,3);
		// ��� vs

	}

}


void LQSISolver::DummyControl(void)
{
	/******************************************************************
	input: void
	output: void

	Note:
	// DummyControl �N�Ҧ��w�g��n��state matrices and control laws �Ӻ�X�������߭y��
	// �Ҧ���feedback���ϥ��|�N�����G �����i�H��X����reference ������COG�y��
	******************************************************************/

	YMatLite EYE3(3,3);
	EYE3.data[0] = 1;
	EYE3.data[1] = 0;
	EYE3.data[2] = 0;
	EYE3.data[3] = 0;
	EYE3.data[4] = 1;
	EYE3.data[5] = 0;
	EYE3.data[6] = 0;
	EYE3.data[7] = 0;
	EYE3.data[8] = 1;

	// state variable
	XState[0].data[0] = 0;
	XState[0].data[1] = 0;
	XState[0].data[2] = 0;

	YMatLite Bkuk(3,1);

	for (int i = 0 ; i<LQDataLen ; i++)
	{
		// sample matlab code
		// Bkuk = BinvMBStk(:,:,i)*(-(Qx*AI+Ss(:,:,i+1)*invDkStk(:,:,i)*WkStk(:,:,i))*x+(eye(3)-Ss(:,:,i+1)*invDkStk(:,:,i)*BinvMBStk(:,:,i))*vs(:,i+1)); 	
		// x = AkStk(:,:,i)*x + Bkuk; 

		//Bkuk = BinvMBT_Stk[i]*(-(Qx*AIStk[i]+SsStk[i+1]*invDkStk[i]*WkStk[i])*XState[i]+(EYE3-SsStk[i+1]*invDkStk[i]*BinvMBT_Stk[i])*vs_Stk[i+1]);
		//XState[i+1] = AdStk[i]*XState[i] + Bkuk;

		MatMulAB(WkStk[i].data,3,3,XState[i].data,3,1,CompuTemp); // Wx
		MatMulAB(invDkStk[i].data,3,3,CompuTemp,3,1,CompuTemp2); // inv(D)Wx
		MatMulAB(SsStk[i+1].data,3,3,CompuTemp2,3,1,CompuTemp); // Sinv(D)Wx

		MatMulAB(AIStk[i].data,3,3,XState[i].data,3,1,CompuTemp2); // AIx
		MatMulAB(Qx->data,3,3,CompuTemp2,3,1,CompuTemp3); // QxAIx

		MatAddAB(CompuTemp,CompuTemp3,CompuTemp2,3); // CompuTemp2 = Sinv(D)Wx+QxAIx

		// get (EYE3-SsStk[i+1]*invDkStk[i]*BinvMBT_Stk[i])*vs_Stk[i+1]
		MatMulAB(BinvMBT_Stk[i].data,3,3,vsStkX[i+1].data,3,1,CompuTemp); // BMBv
		MatMulAB(invDkStk[i].data,3,3,CompuTemp,3,1,CompuTemp3); // inv(D)BMBv
		MatMulAB(SsStk[i+1].data,3,3,CompuTemp3,3,1,CompuTemp); // Sinv(D)BMBv

		MatMiuAB(vsStkX[i+1].data,CompuTemp,CompuTemp,3); // v-Sinv(D)BMBv
		MatMiuAB(CompuTemp,CompuTemp2,CompuTemp,3); // (v-Sinv(D)BMBv) - (Sinv(D)Wx+QxAIx)
		
		MatMulAB(BinvMBT_Stk[i].data,3,3,CompuTemp,3,1,Bkuk.data); // Bkuk found

		MatMulAB(AdStk[i].data,3,3,XState[i].data,3,1,CompuTemp);
		//MatAddAB(CompuTemp,Bkuk.data(),XState[i+1].data(),3);
		MatAddAB(CompuTemp,Bkuk.data,CompuTemp,3);

		// xk+1 = Axk+Bkuk
		MatMulAB(AdStk[i].data,3,3,XState[i].data,3,1,CompuTemp);
		MatAddAB(CompuTemp,Bkuk.data,XState[i+1].data,3);
	}


	for (int i = 0 ; i<LQDataLen ; i++)
	{
		// sample matlab code
		// Bkuk = BinvMBStk(:,:,i)*(-(Qx*AI+Ss(:,:,i+1)*invDkStk(:,:,i)*WkStk(:,:,i))*x+(eye(3)-Ss(:,:,i+1)*invDkStk(:,:,i)*BinvMBStk(:,:,i))*vs(:,i+1)); 	
		// x = AkStk(:,:,i)*x + Bkuk; 

		//Bkuk = BinvMBT_Stk[i]*(-(Qx*AIStk[i]+SsStk[i+1]*invDkStk[i]*WkStk[i])*XState[i]+(EYE3-SsStk[i+1]*invDkStk[i]*BinvMBT_Stk[i])*vs_Stk[i+1]);
		//XState[i+1] = AdStk[i]*XState[i] + Bkuk;

		MatMulAB(WkStk[i].data,3,3,YState[i].data,3,1,CompuTemp); // Wx
		MatMulAB(invDkStk[i].data,3,3,CompuTemp,3,1,CompuTemp2); // inv(D)Wx
		MatMulAB(SsStk[i+1].data,3,3,CompuTemp2,3,1,CompuTemp); // Sinv(D)Wx

		MatMulAB(AIStk[i].data,3,3,YState[i].data,3,1,CompuTemp2); // AIx
		MatMulAB(Qx->data,3,3,CompuTemp2,3,1,CompuTemp3); // QxAIx

		MatAddAB(CompuTemp,CompuTemp3,CompuTemp2,3); // CompuTemp2 = Sinv(D)Wx+QxAIx

		// get (EYE3-SsStk[i+1]*invDkStk[i]*BinvMBT_Stk[i])*vs_Stk[i+1]
		MatMulAB(BinvMBT_Stk[i].data,3,3,vsStkY[i+1].data,3,1,CompuTemp); // BMBv
		MatMulAB(invDkStk[i].data,3,3,CompuTemp,3,1,CompuTemp3); // inv(D)BMBv
		MatMulAB(SsStk[i+1].data,3,3,CompuTemp3,3,1,CompuTemp); // Sinv(D)BMBv

		MatMiuAB(vsStkY[i+1].data,CompuTemp,CompuTemp,3); // v-Sinv(D)BMBv
		MatMiuAB(CompuTemp,CompuTemp2,CompuTemp,3); // (v-Sinv(D)BMBv) - (Sinv(D)Wx+QxAIx)
		
		MatMulAB(BinvMBT_Stk[i].data,3,3,CompuTemp,3,1,Bkuk.data); // Bkuk found

		MatMulAB(AdStk[i].data,3,3,YState[i].data,3,1,CompuTemp);
		//MatAddAB(CompuTemp,Bkuk.data(),XState[i+1].data(),3);
		MatAddAB(CompuTemp,Bkuk.data,CompuTemp,3);

		// xk+1 = Axk+Bkuk
		MatMulAB(AdStk[i].data,3,3,YState[i].data,3,1,CompuTemp);
		MatAddAB(CompuTemp,Bkuk.data,YState[i+1].data,3);

	}

	#if SaveLQSIStates
		// �nprint���G�X�Ӫ��ܡA�Х��}�U��

		fstream Fx;

		Fx.open("statesx.txt",ios::out);
		Fx.precision(10);
		for (int i=0 ; i<= LQDataLen ; i++)
		{
			Fx << XState[i].data[0] << " " << XState[i].data[1] << " " << XState[i].data[2] <<  endl;
		}

		Fx.close();

		fstream Fy;

		Fy.open("statesy.txt",ios::out);
		Fy.precision(10);
		for (int i=0 ; i<= LQDataLen ; i++)
		{
			Fy << YState[i].data[0] << " " << YState[i].data[1] << " " << YState[i].data[2] <<  endl;
		}

		Fy.close();

		// �nprint���G�X�Ӫ��ܡA�Х��}�W��
	#endif
}

void LQSISolver::C2DmHumanoid(int i)
{
	/******************************************************************
	input: i
	output: void

	Note:
	// ��Ji ���禡�N�|�p���i�� sampling time�� state-space �Azero order hold ���᪺��
	******************************************************************/
	//extern Matrix<double,3,3>* AdStk;

	Wp = sqrt((ddelCOG[i]+GravityConst)/(COGz[i+4])); // �o�䪺wp = sqrt(wp) in paper
	sh = sinh(Wp*dt);
	ch = cosh(Wp*dt);

	AdStk[i].data[0] = ch;
	AdStk[i].data[1] = sh/Wp;
	AdStk[i].data[2] = 1-ch;
	AdStk[i].data[3] = Wp*sh;
	AdStk[i].data[4] = ch;
	AdStk[i].data[5] = -Wp*sh;

	BdStk[i].data[0] = dt-(sh/Wp);
	BdStk[i].data[1] = -ch+1;

	// sample matlab code
	////// From Matlab code LQSI_Planner2.m ///////////
    //sw = sqrt(w);
    //sh = sinh(sw*T);
    //ch = cosh(sw*T);
    //Ad = [ch sh/sw 1-ch ; sw*sh ch -sw*sh ; 0 0 1];
    //Bd = [T-sh/sw ; -ch+1 ; T];
    //Cd = C;
    //Dd = D;
	////// From Matlab code LQSI_Planner2.m ///////////

}
 
void LQSISolver::tic(void)
{
	/******************************************************************
	input: void
	output: void

	Note:
	// �p�PMATLAB�� tic(); function �p�ɶ}�l
	******************************************************************/
	QueryPerformanceCounter(&gStartTime);
}

void LQSISolver::ticPMS(void)
{
	/******************************************************************
	input: void
	output: void

	Note:
	// �p�PMATLAB�� tic(); function �p�ɶ}�l
	******************************************************************/
	QueryPerformanceCounter(&gStartTime_PMS);
}


void LQSISolver::toc(void)
{
	/******************************************************************
	input: void
	output: void

	Note:
	// �p�PMATLAB�� toc(); function �p�ɵ��� �åB�bcommand window �L�X�g�L�ɶ�
	******************************************************************/
	QueryPerformanceCounter(&gCurrentTime);	
	double gSysTime = (gCurrentTime.QuadPart - gStartTime.QuadPart)/gFreqT;
	//cout << gSysTime << endl;
			TimerRecord.open("timerdata.txt",ios::app);
			TimerRecord << gSysTime << "\t";
			TimerRecord.close();
}

void LQSISolver::toc2(void)
{
	/******************************************************************
	input: void
	output: void

	Note:
	// ���MATLAB�� toc(); function �p�ɵ��� �|�bcommand window �L�X�̤j�B��ɶ�
	******************************************************************/
	QueryPerformanceCounter(&gCurrentTime);	
	double gSysTime = (gCurrentTime.QuadPart - gStartTime.QuadPart)/gFreqT;
	if (gSysTime>MaxIterationTime)
	MaxIterationTime=gSysTime;
	cout << MaxIterationTime << endl;
		// save encoder
			TimerRecord.open("timerdata.txt",ios::app);
			TimerRecord << MaxIterationTime << "\t";
			TimerRecord.close();
		// save encoder
}

void LQSISolver::toc3(void)
{
	/******************************************************************
	input: void
	output: void

	Note:
	// �p�PMATLAB�� toc(); function �p�ɵ��� �åB�bcommand window �L�X�g�L�ɶ�
	******************************************************************/
	QueryPerformanceCounter(&gCurrentTime);	
	double gSysTime = (gCurrentTime.QuadPart - gStartTime.QuadPart)/gFreqT;

	LogSysTime[TimeCount]=gSysTime;
	TimeCount++;
}

void LQSISolver::toc3PMS(void)
{
	/******************************************************************
	input: void
	output: void

	Note:
	// �p�PMATLAB�� toc(); function �p�ɵ��� �åB�bcommand window �L�X�g�L�ɶ�
	******************************************************************/
	QueryPerformanceCounter(&gCurrentTime_PMS);	
	double gSysTime = (gCurrentTime_PMS.QuadPart - gStartTime_PMS.QuadPart)/gFreqT_PMS;

	LogPMSTime[TimeCountPMS]=gSysTime;
	TimeCountPMS++;
}

void DiffEqn(double* A, int* LenA, double* result, double* d_t)
{
	/******************************************************************
	input:  A ��J�n�Q�L�����}�C�A LenA �}�CA�����סA ���G�s�b result�A d_t�O�L����delta t 
	output: void

	Note:
	// ���L����k�|�ϱo�̥��̥k��Ӥ����Q�˱�A�ҥH�}�C���׷|��4
	******************************************************************/
	double temp = 1.0/12.0/(*d_t);
	for (int i=2 ; i< (*LenA)-2; i++)
	{
		result[i-2] = temp*(A[i-2]-8*A[i-1]+8*A[i+1]-A[i+2]);
	}
}

void DiffEqnShift(double* A, int* LenA, double* result, double* d_t)
{
	/******************************************************************
	input:  A ��J�n�Q�L�����}�C�A LenA �}�CA�����סA ���G�s�b result�A d_t�O�L����delta t 
	output: void

	Note:
	// ���L����k�|�ϱo�̥��̥k��Ӥ����Q�˱�A�ҥH�}�C���׷|��4 
	// ���O�ѩ�b�{�����N�s���Y������ƻs�⦸ �ҥH�i�H���������
	******************************************************************/
	double temp = 1.0/12.0/(*d_t);
	for (int i=2 ; i< (*LenA)-2; i++)
	{
		result[i] = temp*(A[i-2]-8*A[i-1]+8*A[i+1]-A[i+2]);
	}
	result[0] = result[2];
	result[1] = result[2];
	result[*LenA-1] = result[*LenA-3];
	result[*LenA-2] = result[*LenA-3];

}

void LowPassFilter(double* data, int dataLen, double* result, double gain)
{
	/******************************************************************
	input:  data ��J�n�Qfilter���}�C�A dataLen �}�CA�����סA ���G�s�b result�A gain�Ofilter��filter gain 
	output: void

	Note:
	// �Q�ήt����{���F���Ʀ��o�i��
	******************************************************************/
	result[0] = data[0];

	for (int i=1;i<dataLen;i++)
	{
		result[i] = 1.0/(gain+1.0)*(data[i]+data[i-1])+(gain-1.0)/(gain+1.0)*result[i-1];
	}
	// Matlab sample codes

	//L = length(data);
	//result = zeros(L,1);
	//result(1) = data(1);
	//for i = 2:L
	//    result(i) = 1/(k+1)*(data(i)+data(i-1))+(k-1)/(k+1)*result(i-1);
	//end
}

void GetMTSAcceleration(double* sig, double* h, double* q, int InputTrajLen, double* Boundary, double* ddQ)
{
	/******************************************************************
	input: sig,  h,  q,  InputTrajLen,  Boundary,  ddQ �����p�U
	output: void

	Note:
	// �p��MTS��k���y�񤺴�
	// sig = sigma
	// h = time intervals �w�g���J��q�۰ʥͦ��I���ɶ��q�}�C ����=TrajLen+1�q
	// q = the traj knots ��y��A�|�����J���I
	// boundary are the boundry conditions boundary = [v_1 v_end a_1 a_end]
	// TrajLen = ��l�y���
	// ddQ = double derivative of Q
	******************************************************************/

	// �ЫؼȦs��
	double* a;
	a = new double[InputTrajLen+1];
	double* b;
	b = new double[InputTrajLen+1];

	double* A;
	A = new double[InputTrajLen*InputTrajLen];
	double* B;
	B = new double[InputTrajLen];

	double temp1, temp2;
	double q_hat_2, q_hat_nm1; // q2 and qn-1

	// get all a and b
	for (int i = 0 ; i < InputTrajLen ; i++)
	{
		temp1 = 1.0/sig[i]/sig[i]/h[i];
		temp2 = sig[i]*h[i];
		a[i] = temp1-1.0/sig[i]/sinh(temp2);
		b[i] = 1.0/sig[i]/tanh(temp2)-temp1;
	}


	// Find A
		// clear A
	for (int i = 0 ; i < InputTrajLen*InputTrajLen ; i++)
	{
		A[i] = 0;
	}

	A[0] = b[0]+b[1]+h[0]*h[0]/6.0/h[1]+h[0]/6.0; // first one
	A[InputTrajLen*InputTrajLen-1] = b[InputTrajLen-2]+b[InputTrajLen-1]+h[InputTrajLen-1]*h[InputTrajLen-1]/6.0/h[InputTrajLen-2]+h[InputTrajLen-1]/6.0; // final one

	for ( int i = 1 ; i < InputTrajLen ; i++)
	{
		A[InputTrajLen*(i-1)+i] = a[i];
		A[InputTrajLen*i+i-1] = a[i];
	}
	for ( int i = 1 ; i < InputTrajLen-1 ; i++)
	{
		A[InputTrajLen*i+i] = b[i]+b[i+1];
	}
	// ��K�g�{�� �̫�b�ɤW�t��
	A[InputTrajLen] -= h[0]*h[0]/6.0/h[1];
	A[InputTrajLen*(InputTrajLen-1)-1] -= h[InputTrajLen-2]*h[InputTrajLen-2]/6.0/h[InputTrajLen-3];


	// Find B
		// q2 and qn-1
		// boundary = [v_1 v_end a_1 a_end]
	q_hat_2 = q[0]+h[0]*Boundary[0]+h[0]*h[0]*Boundary[2]/3.0;
	q_hat_nm1 = q[InputTrajLen-1]-h[InputTrajLen-1]*Boundary[1]+h[InputTrajLen-1]*h[InputTrajLen-1]*Boundary[3]/3.0; 


	B[0] = (q[1]-q_hat_2)/h[1]-(q_hat_2-q[0])/h[0]-a[0]*Boundary[2];
	B[1] = (q[2]-q[1])/h[2]-(q[1]-q_hat_2)/h[1];
	B[InputTrajLen-1] = (q[InputTrajLen-1]-q_hat_nm1)/h[InputTrajLen-1]-(q_hat_nm1-q[InputTrajLen-2])/h[InputTrajLen-2]-a[InputTrajLen-1]*Boundary[3];
	B[InputTrajLen-2] = (q_hat_nm1-q[InputTrajLen-2])/h[InputTrajLen-2]-(q[InputTrajLen-2]-q[InputTrajLen-3])/h[InputTrajLen-3];

	for ( int i = 2 ; i < InputTrajLen-2 ; i++)
	{
		B[i] = (q[i+1]-q[i])/h[i+1]-(q[i]-q[i-1])/h[i];
	}

	InvSqMat(A,InputTrajLen);

	// �p��[�t��
	MatMulAB(A,InputTrajLen,InputTrajLen,B,InputTrajLen,1,ddQ);


	for (int pp = 0 ; pp<InputTrajLen ; pp++)
	{
		printf(" %f\n",ddQ[pp]);
	}

	// �M���ʺA�O����
	delete[] a;
	delete[] b;
	delete[] A;
	delete[] B;

}

void MovingAvergeSmooth(double* Traj, int LengthOfTraj, int FrameSize, double* result)
{
	/******************************************************************
	input: Traj ��l��J�y��,  LengthOfTraj ��l�y�����,  FrameSize �n�������]�t�y��d��,  ���G�s�bresult
	output: void

	Note:
	// �Q��Moving Average��k�D�o������
	// ����k���u�I�O�S��phase lag �i�H�������u 
	// �ʽ��low pass filter ���I���P

	******************************************************************/

	int a,b;
	int Size;

	if (FrameSize%2 == 0)
		Size = FrameSize-1;
	else
		Size = FrameSize;

	a = (Size-1)/2;

	for (int i = 0 ; i < LengthOfTraj ; i++)
	{
		result[i] = 0;
		b = LengthOfTraj-1-i;

		if (i < a)
		{
			for (int j = 0 ; j <= 2*i ; j++)
			{
				result[i] += Traj[j];
			}
			result[i] /= (i*2+1);
		}
		else if (i+a > LengthOfTraj-1)
		{
			for (int j = i-b ; j <= LengthOfTraj-1 ; j++)
			{
				result[i] += Traj[j];
			}
			result[i] /= (b*2+1);
		}
		else
		{
			for (int j = i-a ; j <= i+a ; j++)
			{
				result[i] += Traj[j];
			}
			result[i] /= (Size);
		}
	}
}

void LQSISolver::ZMPFeedbackControl(double* ZMPx, double* ZMPy,double* COGx ,double* COGy , int index)
{
	/******************************************************************
	input: ZMPx, ZMPy, COGx, COGy, �}�C����(�W���w�w)
	output: void
	Note:
	// ��DummyControl ���ܦӨ�, 
	   �N�Ҧ��w�g��n��state matrices and control laws �Ӻ�X�������߭y��
	// �MDummyControl���P�B��,feedback���A�ϥ��|�N�����G 
	   �ӥ�ManinLoops.cpp  C2MWrite2Txt() ����Case 5 ��update�s��ZMP��m
	   �]���ݨC��iteration(5ms)�N�u����ZMP��^state space����������m
	   �M��ΦA�M�w����n��control laws �Ӻ�X�C��5ms��COG���������߭y��
	******************************************************************/
//	double CompuTemp[400]; // �x�}���k���ݭn���ƧQ�Ϊ�function�ݭn��ӼȦs�� ���M�|�мg�쥿�b�Ϊ����
//	double CompuTemp2[400];
//	double CompuTemp3[400];
//	double CompuTemp4[400];

	YMatLite EYE3(3,3);
	EYE3.data[0] = 1;
	EYE3.data[1] = 0;
	EYE3.data[2] = 0;
	EYE3.data[3] = 0;
	EYE3.data[4] = 1;
	EYE3.data[5] = 0;
	EYE3.data[6] = 0;
	EYE3.data[7] = 0;
	EYE3.data[8] = 1;

	//XState[0].data[0] = 0;
	//XState[0].data[1] = 0;
	//XState[0].data[2] = 0;

	//Matrix<double,3,1> Bkuk;

	YMatLite Bkuk(3,1);
	double Aka=0;
	//tic();

	//for (int i = 0 ; i<LQDataLen ; i++)
	//{
		// Bkuk = BinvMBStk(:,:,i)*(-(Qx*AI+Ss(:,:,i+1)*invDkStk(:,:,i)*WkStk(:,:,i))*x+(eye(3)-Ss(:,:,i+1)*invDkStk(:,:,i)*BinvMBStk(:,:,i))*vs(:,i+1)); 	
		// x = AkStk(:,:,i)*x + Bkuk; 

		//Bkuk = BinvMBT_Stk[i]*(-(Qx*AIStk[i]+SsStk[i+1]*invDkStk[i]*WkStk[i])*XState[i]+(EYE3-SsStk[i+1]*invDkStk[i]*BinvMBT_Stk[i])*vs_Stk[i+1]);
		//XState[i+1] = AdStk[i]*XState[i] + Bkuk;


		MatMulAB(WkStk[index].data,3,3,XState[index].data,3,1,CompuTemp); // Wx
		MatMulAB(invDkStk[index].data,3,3,CompuTemp,3,1,CompuTemp2); // inv(D)Wx
		MatMulAB(SsStk[index+1].data,3,3,CompuTemp2,3,1,CompuTemp); // Sinv(D)Wx

		MatMulAB(AIStk[index].data,3,3,XState[index].data,3,1,CompuTemp2); // AIx
		MatMulAB(Qx->data,3,3,CompuTemp2,3,1,CompuTemp3); // QxAIx

		MatAddAB(CompuTemp,CompuTemp3,CompuTemp2,3); // CompuTemp2 = Sinv(D)Wx+QxAIx

		// get (EYE3-SsStk[i+1]*invDkStk[i]*BinvMBT_Stk[i])*vs_Stk[i+1]
		MatMulAB(BinvMBT_Stk[index].data,3,3,vsStkX[index+1].data,3,1,CompuTemp); // BMBv
		MatMulAB(invDkStk[index].data,3,3,CompuTemp,3,1,CompuTemp3); // inv(D)BMBv
		MatMulAB(SsStk[index+1].data,3,3,CompuTemp3,3,1,CompuTemp); // Sinv(D)BMBv

		MatMiuAB(vsStkX[index+1].data,CompuTemp,CompuTemp,3); // v-Sinv(D)BMBv
		MatMiuAB(CompuTemp,CompuTemp2,CompuTemp,3); // (v-Sinv(D)BMBv) - (Sinv(D)Wx+QxAIx)
		
		MatMulAB(BinvMBT_Stk[index].data,3,3,CompuTemp,3,1,Bkuk.data); // Bkuk found

		MatMulAB(AdStk[index].data,3,3,XState[index].data,3,1,CompuTemp);
		//MatAddAB(CompuTemp,Bkuk.data(),XState[i+1].data(),3);
		MatAddAB(CompuTemp,Bkuk.data,CompuTemp,3);

		// xk+1 = Axk+Bkuk
		MatMulAB(AdStk[index].data,3,3,XState[index].data,3,1,CompuTemp);
		MatAddAB(CompuTemp,Bkuk.data,XState[index+1].data,3);


	//}

		double QQ=0;
	//for (int i = 0 ; i<LQDataLen ; i++)
	//{
		// Bkuk = BinvMBStk(:,:,i)*(-(Qx*AI+Ss(:,:,i+1)*invDkStk(:,:,i)*WkStk(:,:,i))*x+(eye(3)-Ss(:,:,i+1)*invDkStk(:,:,i)*BinvMBStk(:,:,i))*vs(:,i+1)); 	
		// x = AkStk(:,:,i)*x + Bkuk; 

		//Bkuk = BinvMBT_Stk[i]*(-(Qx*AIStk[i]+SsStk[i+1]*invDkStk[i]*WkStk[i])*XState[i]+(EYE3-SsStk[i+1]*invDkStk[i]*BinvMBT_Stk[i])*vs_Stk[i+1]);
		//XState[i+1] = AdStk[i]*XState[i] + Bkuk;


		MatMulAB(WkStk[index].data,3,3,YState[index].data,3,1,CompuTemp); // Wx
		MatMulAB(invDkStk[index].data,3,3,CompuTemp,3,1,CompuTemp2); // inv(D)Wx
		MatMulAB(SsStk[index+1].data,3,3,CompuTemp2,3,1,CompuTemp); // Sinv(D)Wx

		MatMulAB(AIStk[index].data,3,3,YState[index].data,3,1,CompuTemp2); // AIx
		MatMulAB(Qx->data,3,3,CompuTemp2,3,1,CompuTemp3); // QxAIx

		MatAddAB(CompuTemp,CompuTemp3,CompuTemp2,3); // CompuTemp2 = Sinv(D)Wx+QxAIx

		// get (EYE3-SsStk[i+1]*invDkStk[i]*BinvMBT_Stk[i])*vs_Stk[i+1]
		MatMulAB(BinvMBT_Stk[index].data,3,3,vsStkY[index+1].data,3,1,CompuTemp); // BMBv
		MatMulAB(invDkStk[index].data,3,3,CompuTemp,3,1,CompuTemp3); // inv(D)BMBv
		MatMulAB(SsStk[index+1].data,3,3,CompuTemp3,3,1,CompuTemp); // Sinv(D)BMBv

		MatMiuAB(vsStkY[index+1].data,CompuTemp,CompuTemp,3); // v-Sinv(D)BMBv
		MatMiuAB(CompuTemp,CompuTemp2,CompuTemp,3); // (v-Sinv(D)BMBv) - (Sinv(D)Wx+QxAIx)
		
		MatMulAB(BinvMBT_Stk[index].data,3,3,CompuTemp,3,1,Bkuk.data); // Bkuk found

		MatMulAB(AdStk[index].data,3,3,YState[index].data,3,1,CompuTemp);
		//MatAddAB(CompuTemp,Bkuk.data(),XState[i+1].data(),3);
		MatAddAB(CompuTemp,Bkuk.data,CompuTemp,3);
		
		// xk+1 = Axk+Bkuk
		MatMulAB(AdStk[index].data,3,3,YState[index].data,3,1,CompuTemp);
		MatAddAB(CompuTemp,Bkuk.data,YState[index+1].data,3);
		
	//}
		QQ=AdStk[index].data[0];
		QQ=AdStk[index].data[1];
		QQ=AdStk[index].data[2];
		QQ=AdStk[index].data[3];
		QQ=AdStk[index].data[4];
		QQ=AdStk[index].data[5];
		QQ=AdStk[index].data[6];
		QQ=AdStk[index].data[7];
		QQ=AdStk[index].data[8];

	//// �nprint�X�Ӫ��ܡA�Х��}�U��

	//fstream Fx;

	//Fx.open("statesx.txt",ios::out);
	//Fx.precision(10);
	//for (int i=0 ; i<= LQDataLen ; i++)
	//{
	//	Fx << XState[i].data[0] << " " << XState[i].data[1] << " " << XState[i].data[2] <<  endl;
	//}

	//Fx.close();

	//fstream Fy;

	//Fy.open("statesy.txt",ios::out);
	//Fy.precision(10);
	//for (int i=0 ; i<= LQDataLen ; i++)
	//{
	//	Fy << YState[i].data[0] << " " << YState[i].data[1] << " " << YState[i].data[2] <<  endl;
	//}

	//Fy.close();

	//// �nprint�X�Ӫ��ܡA�Х��}�W��
}