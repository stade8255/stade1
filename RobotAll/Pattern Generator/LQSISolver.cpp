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
     本程式主要用在解出基於倒單擺模型的inverted pendulum 問題
	 輸入ZMP以及COG高度軌跡 就可以解出水平方向ZMP擺動之軌跡
	 各函式與變數之說明請詳見下方宣告與定義處之說明

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
	// Class constructor  初始化所有需要用到的變數與矩陣
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

	CompuTemp = new double[400]; // 矩陣乘法等需要重複利用的function需要兩個暫存區 不然會覆寫到正在用的資料
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

	// initial 不會變的值
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

	// 清除動態記憶體
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
	// 從外部輸入COG高度軌跡 這樣才能算出所有倒單擺模型在每一瞬間的狀態矩陣
	******************************************************************/

	int LenTemp = LQDataLen+8;
	double SampTime = dt;
								
	COGz = InputCOGz;

	// 對重心高度軌跡微分
	// 時間佔很短 約10^-5
	DiffEqn(COGz,&LenTemp,delCOG,&SampTime);
	LenTemp = LQDataLen+4;
	DiffEqn(delCOG,&LenTemp,ddelCOG,&SampTime);
	// 時間佔很短 約10^-5

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
		printf("請小心 有 %d 個COG加速度超過 9810mm/s^2 請注意!! 值被強制改成-9809了",cnt);
	}



}


void LQSISolver::BackRiccati(double* ZMPx, double* ZMPy)
{
	/******************************************************************
	input: ZMPx ZMPy, ZMP在水平方向的軌跡 這是LQSI controller的 reference input
	output: void

	Note:
	// 輸入MZPx ZMPy給LQSI controller當作reference input
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

	// 取出zero order hold 的 狀態矩陣(state-space matrices)
	// 0.24ms --> 0.13ms
	for (int i = 0 ; i < LQDataLen ; i++)
	{
		C2DmHumanoid(i);
	}
	// 0.24ms


	//tic();

	// LQSI中的變數 AI = Ad - (identity matrix 3-by-3)
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

	// 計算LQSI Control Law~ 
	// 0.08ms
	for (int i = 0; i < LQDataLen ; i++)
	{
		// 原始 equation
		//gTempM = BdT_stk[i]*Qx*BdStk[i]; 
		//invMkStk[i] = 1.0/(gTempM.data()[0]+R); // invMkStk(1,i) = 1/(Bd'*Qx*Bd+R);

		MatMulAtB(BdStk[i].data,3,1,Qx->data,3,3,CompuTemp);
		MatMulAB(CompuTemp,1,3,BdStk[i].data,3,1,CompuTemp2);
		invMkStk[i] = 1.0/(CompuTemp2[0]+R); // invMkStk(1,i) = 1/(Bd'*Qx*Bd+R);

		//temp11 = (BdStk[i]>Qx[0])*BdStk[i];
		//invMkStk[i] = 1.0/(temp11.data[0]+R); // invMkStk(1,i) = 1/(Bd'*Qx*Bd+R);

	}
	//0.08ms


	// 計算LQSI Control Law~ 
	for (int i = 0; i < LQDataLen ; i++)
	{
		// 0.1ms
		MatScalarMul(BdStk[i].data,3,invMkStk+i,CompuTemp);
		MatMulABt(CompuTemp,3,1,BdStk[i].data,3,1,BinvMBT_Stk[i].data);
		// 0.1ms
	}

	// 計算LQSI Control Law~ 
	for (int i = 0; i < LQDataLen ; i++)
	{
		// 0.31ms
		MatMulAB(BinvMBT_Stk[i].data,3,3,Qx->data,3,3,CompuTemp);
		MatMulAB(CompuTemp,3,3,AIStk[i].data,3,3,CompuTemp2);
		MatMiuAB(AdStk[i].data,CompuTemp2,WkStk[i].data,9);
	}


	// 計算LQSI Control Law~ 
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

	// 計算LQSI Control Law~ 
	/////// 這裡可以用 預先算好的最終值取代
	tempScale = P;

	vsStkX[(int)(LQDataLen)].data[0] = CdT->data[0]*(P*ZMPx[(int)(LQDataLen-1)]); // 指定最後的值(backward riccati的開始)
	vsStkX[(int)(LQDataLen)].data[1] = CdT->data[1]*(P*ZMPx[(int)(LQDataLen-1)]); // 指定最後的值(backward riccati的開始)
	vsStkX[(int)(LQDataLen)].data[2] = CdT->data[2]*(P*ZMPx[(int)(LQDataLen-1)]); // 指定最後的值(backward riccati的開始)

	vsStkY[(int)(LQDataLen)].data[0] = CdT->data[0]*(P*ZMPy[(int)(LQDataLen-1)]); // 指定最後的值(backward riccati的開始)
	vsStkY[(int)(LQDataLen)].data[1] = CdT->data[1]*(P*ZMPy[(int)(LQDataLen-1)]); // 指定最後的值(backward riccati的開始)
	vsStkY[(int)(LQDataLen)].data[2] = CdT->data[2]*(P*ZMPy[(int)(LQDataLen-1)]); // 指定最後的值(backward riccati的開始)

	MatScalarMul(Cd->data,3,&tempScale,CompuTemp);
	MatMulAB(CdT->data,3,1,CompuTemp,1,3,SsStk[(int)(LQDataLen)].data);

	/////// 這裡可以用 預先算好的最終值取代

	// 計算LQSI Control Law~ 
	// 88ms original
	for (int i = (int)(LQDataLen-1) ; i >=0 ; i--)
	{
		// MATLAB code
		////invDkStk(:,:,i) = inv(eye(3)+BinvMBStk(:,:,i)*Ss(:,:,i+1));
		////Ss(:,:,i) = Nk'*Qx*Nk+(invMkStk(1,i)*invMkStk(1,i)*R)*(AI'*Qx*Bd*Bd'*Qx*AI)+CQC+Ad'*Ss(:,:,i+1)*invDkStk(:,:,i)*Wk;
		////vs(:,i) = Cd'*Q*ZMPr(i)+Ad'*(eye(3)-Ss(:,:,i+1)*invDkStk(:,:,i)*BinvMBStk(:,:,i))*vs(:,i+1);
		// MATLAB code

		//invDkStk[i] = EYE3+BinvMBT_Stk[i]*SsStk[i+1]; // 下面兩行的原式		
		MatMulAB(BinvMBT_Stk[i].data,3,3,SsStk[i+1].data,3,3,CompuTemp);
		MatAddAB(EYE3.data,CompuTemp,invDkStk[i].data,9);

		InvSqMat(invDkStk[i].data,3); // Take Inverse: call by reference 不需要另外加速

		// Ss 原式
		//SsStk[i] = NkTStk[i]*Qx*NkStk[i]+((invMkStk[i]*invMkStk[i]*R)*(AIT_stk[i]*Qx*BdStk[i]))*(BdT_stk[i]*Qx*AIStk[i]) + CdT*Q*Cd + AdT_stk[i]*SsStk[i+1]*invDkStk[i]*WkStk[i];

		// 拆算 Ss, 省下 50ms
		MatMulAtB(NkStk[i].data,3,3,Qx->data,3,3,CompuTemp);
		MatMulAB(CompuTemp,3,3,NkStk[i].data,3,3,CompuTemp2); // 算完 N'QxN 存在 CompuTemp2

		MatMulAB(Qx->data,3,3,BdStk[i].data,3,1,CompuTemp);
		MatMulAtB(AIStk[i].data,3,3,CompuTemp,3,1,CompuTemp3);

		tempScale = invMkStk[i]*invMkStk[i]*R;
		MatScalarMul(CompuTemp3,3,&tempScale,CompuTemp4);
		MatMulABt(CompuTemp4,3,1,CompuTemp3,3,1,CompuTemp); // 算完中段，存在 CompuTemp

		//MatScalarMul(CompuTemp3,3,&tempScale,CompuTemp3);
		//MatMulAtB(BdStk[i].data(),3,1,Qx.data(),3,3,CompuTemp);
		//MatMulAB(CompuTemp,1,3,AIStk[i].data(),3,3,CompuTemp4);
		//MatMulAB(CompuTemp3,3,1,CompuTemp4,1,3,CompuTemp); // 算完中段，存在 CompuTemp

		MatAddAB(CompuTemp,CompuTemp2,CompuTemp,9); // N'QN + 中段，存在 CompuTemp

		tempScale = Q;
		MatScalarMul(Cd->data,3,&tempScale,CompuTemp2);
		MatMulAtB(Cd->data,1,3,CompuTemp2,1,3,CompuTemp3); // 算完 C'QC

		MatAddAB(CompuTemp,CompuTemp3,CompuTemp,9); // N'QN + 中段 + C'QC，存在 CompuTemp

		MatMulAtB(AdStk[i].data,3,3,SsStk[i+1].data,3,3,CompuTemp2);
		MatMulAB(CompuTemp2,3,3,invDkStk[i].data,3,3,CompuTemp3);
		MatMulAB(CompuTemp3,3,3,WkStk[i].data,3,3,CompuTemp2);

		MatAddAB(CompuTemp,CompuTemp2,SsStk[i].data,9); // 算完 Ss，存在 SsStk[i]
		// 拆算 Ss

		//// 拆算 vs 省下30ms
		////vs_Stk[i] = CdT*Q*Input_ZMP[i]+AdT_stk[i]*(EYE3-SsStk[i+1]*invDkStk[i]*BinvMBT_Stk[i])*vs_Stk[i+1];

		//tempScale = Q*Input_ZMP[i];
		//MatScalarMul(Cd->data,3,&tempScale,CompuTemp); // C'Qr

		//MatMulAB(BinvMBT_Stk[i].data,3,3,vs_Stk[i+1].data,3,1,CompuTemp2);
		//MatMulAB(invDkStk[i].data,3,3,CompuTemp2,3,1,CompuTemp3);
		//MatMulAB(SsStk[i+1].data,3,3,CompuTemp3,3,1,CompuTemp2);

		//MatMiuAB(vs_Stk[i+1].data,CompuTemp2,CompuTemp3,3);
		//MatMulAtB(AdStk[i].data,3,3,CompuTemp3,3,1,CompuTemp2); // 算完後半

		//MatAddAB(CompuTemp,CompuTemp2,vs_Stk[i].data,3);
		//// 拆算 vs

	}

	// for ZMPx
	for (int i = (int)(LQDataLen-1) ; i >=0 ; i--)
	{
		// 拆算 vs 省下30ms
		//vs_Stk[i] = CdT*Q*Input_ZMP[i]+AdT_stk[i]*(EYE3-SsStk[i+1]*invDkStk[i]*BinvMBT_Stk[i])*vs_Stk[i+1];

		tempScale = Q*ZMPx[i];
		MatScalarMul(Cd->data,3,&tempScale,CompuTemp); // C'Qr

		MatMulAB(BinvMBT_Stk[i].data,3,3,vsStkX[i+1].data,3,1,CompuTemp2);
		MatMulAB(invDkStk[i].data,3,3,CompuTemp2,3,1,CompuTemp3);
		MatMulAB(SsStk[i+1].data,3,3,CompuTemp3,3,1,CompuTemp2);

		MatMiuAB(vsStkX[i+1].data,CompuTemp2,CompuTemp3,3);
		MatMulAtB(AdStk[i].data,3,3,CompuTemp3,3,1,CompuTemp2); // 算完後半

		MatAddAB(CompuTemp,CompuTemp2,vsStkX[i].data,3);
		// 拆算 vs
	}

	// for ZMPy
	for (int i = (int)(LQDataLen-1) ; i >=0 ; i--)
	{
		// 拆算 vs 省下30ms
		//vs_Stk[i] = CdT*Q*Input_ZMP[i]+AdT_stk[i]*(EYE3-SsStk[i+1]*invDkStk[i]*BinvMBT_Stk[i])*vs_Stk[i+1];

		tempScale = Q*ZMPy[i];
		MatScalarMul(Cd->data,3,&tempScale,CompuTemp); // C'Qr

		MatMulAB(BinvMBT_Stk[i].data,3,3,vsStkY[i+1].data,3,1,CompuTemp2);
		MatMulAB(invDkStk[i].data,3,3,CompuTemp2,3,1,CompuTemp3);
		MatMulAB(SsStk[i+1].data,3,3,CompuTemp3,3,1,CompuTemp2);

		MatMiuAB(vsStkY[i+1].data,CompuTemp2,CompuTemp3,3);
		MatMulAtB(AdStk[i].data,3,3,CompuTemp3,3,1,CompuTemp2); // 算完後半

		MatAddAB(CompuTemp,CompuTemp2,vsStkY[i].data,3);
		// 拆算 vs

	}

}


void LQSISolver::DummyControl(void)
{
	/******************************************************************
	input: void
	output: void

	Note:
	// DummyControl 將所有已經算好的state matrices and control laws 來算出水平重心軌跡
	// 所有的feedback先使用疊代的結果 直接可以算出未來reference 的水平COG軌跡
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
		// 要print結果出來的話，請打開下面

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

		// 要print結果出來的話，請打開上面
	#endif
}

void LQSISolver::C2DmHumanoid(int i)
{
	/******************************************************************
	input: i
	output: void

	Note:
	// 輸入i 此函式就會計算第i個 sampling time的 state-space 再zero order hold 之後的值
	******************************************************************/
	//extern Matrix<double,3,3>* AdStk;

	Wp = sqrt((ddelCOG[i]+GravityConst)/(COGz[i+4])); // 這邊的wp = sqrt(wp) in paper
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
	// 如同MATLAB的 tic(); function 計時開始
	******************************************************************/
	QueryPerformanceCounter(&gStartTime);
}

void LQSISolver::ticPMS(void)
{
	/******************************************************************
	input: void
	output: void

	Note:
	// 如同MATLAB的 tic(); function 計時開始
	******************************************************************/
	QueryPerformanceCounter(&gStartTime_PMS);
}


void LQSISolver::toc(void)
{
	/******************************************************************
	input: void
	output: void

	Note:
	// 如同MATLAB的 toc(); function 計時結束 並且在command window 印出經過時間
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
	// 改自MATLAB的 toc(); function 計時結束 會在command window 印出最大運算時間
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
	// 如同MATLAB的 toc(); function 計時結束 並且在command window 印出經過時間
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
	// 如同MATLAB的 toc(); function 計時結束 並且在command window 印出經過時間
	******************************************************************/
	QueryPerformanceCounter(&gCurrentTime_PMS);	
	double gSysTime = (gCurrentTime_PMS.QuadPart - gStartTime_PMS.QuadPart)/gFreqT_PMS;

	LogPMSTime[TimeCountPMS]=gSysTime;
	TimeCountPMS++;
}

void DiffEqn(double* A, int* LenA, double* result, double* d_t)
{
	/******************************************************************
	input:  A 輸入要被微分的陣列， LenA 陣列A的長度， 結果存在 result， d_t是微分的delta t 
	output: void

	Note:
	// 此微分方法會使得最左最右兩個元素被捨棄，所以陣列長度會少4
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
	input:  A 輸入要被微分的陣列， LenA 陣列A的長度， 結果存在 result， d_t是微分的delta t 
	output: void

	Note:
	// 此微分方法會使得最左最右兩個元素被捨棄，所以陣列長度會少4 
	// 但是由於在程式中將新的頭跟尾都複製兩次 所以可以維持原長度
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
	input:  data 輸入要被filter的陣列， dataLen 陣列A的長度， 結果存在 result， gain是filter的filter gain 
	output: void

	Note:
	// 利用差分方程式達成數位濾波器
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
	input: sig,  h,  q,  InputTrajLen,  Boundary,  ddQ 說明如下
	output: void

	Note:
	// 計算MTS方法之軌跡內插
	// sig = sigma
	// h = time intervals 已經插入兩段自動生成點的時間段陣列 長度=TrajLen+1段
	// q = the traj knots 原軌跡，尚未插入兩點
	// boundary are the boundry conditions boundary = [v_1 v_end a_1 a_end]
	// TrajLen = 原始軌跡長
	// ddQ = double derivative of Q
	******************************************************************/

	// 創建暫存區
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
	// 方便寫程式 最後在補上差值
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

	// 計算加速度
	MatMulAB(A,InputTrajLen,InputTrajLen,B,InputTrajLen,1,ddQ);


	for (int pp = 0 ; pp<InputTrajLen ; pp++)
	{
		printf(" %f\n",ddQ[pp]);
	}

	// 清除動態記憶體
	delete[] a;
	delete[] b;
	delete[] A;
	delete[] B;

}

void MovingAvergeSmooth(double* Traj, int LengthOfTraj, int FrameSize, double* result)
{
	/******************************************************************
	input: Traj 原始輸入軌跡,  LengthOfTraj 原始軌跡長度,  FrameSize 要平均的包含軌跡範圍,  結果存在result
	output: void

	Note:
	// 利用Moving Average方法求得平均值
	// 此方法的優點是沒有phase lag 可以撫平曲線 
	// 性質跟low pass filter 有點不同

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
	input: ZMPx, ZMPy, COGx, COGy, 陣列指標(規劃預定)
	output: void
	Note:
	// 由DummyControl 改變而來, 
	   將所有已經算好的state matrices and control laws 來算出水平重心軌跡
	// 和DummyControl不同處為,feedback不再使用疊代的結果 
	   而由ManinLoops.cpp  C2MWrite2Txt() 中的Case 5 來update新的ZMP位置
	   因此需每個iteration(5ms)將真正的ZMP放回state space中的對應位置
	   然後用再和預先算好的control laws 來算出每個5ms其COG之水平重心軌跡
	******************************************************************/
//	double CompuTemp[400]; // 矩陣乘法等需要重複利用的function需要兩個暫存區 不然會覆寫到正在用的資料
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

	//// 要print出來的話，請打開下面

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

	//// 要print出來的話，請打開上面
}