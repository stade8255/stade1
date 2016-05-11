/**************************************************************************************************
Copyright, 2010-2012, Robotics Lab., Dept. of M.E., National Taiwan University
File Name: MatOptInterface.cpp

Author: Jiu-Lou Yan
Version: 1.0
Date: 2010/07/10

Functions:
     InvSqMat() AbsDetSqMat()  SubMat() SubMatIndex() MatScalarMul()
     MatMulAB() MatMulAtB() MatMulABt() MatMiuAB() MatAddAB()

Classes: None

Description:
     ���{���D�n�Φb�إߤ@�M�x�}�B��禡�w�B�w�q�F�x�}�B�⤧�禡�A
	 �]�t�ϯx�}�B��C���ȡB�x�}�[��k���B��A
	 �U�禡�P�ܼƤ������иԨ��U��ŧi�P�w�q�B������

Note: None
***************************************************************************************************/
#include "stdafx.h"
#include "MatOptInterface.h"



void InvSqMat(double * Data, int NCol)
{
	/******************************************************************
	input: �x�}�����}�Y���СA�x�}column��
	output: �����x�s�b�x�}������m

	Note: ��J�@�w�n�O��}!!
	******************************************************************/

	// �ǳƥᵹCLAPACK���ܼ� �٦��O������t
	integer Dim = NCol;
	doublereal *work = new doublereal[Dim*Dim];
    integer INFO; 

    integer * ipiv = new integer[Dim*Dim];

	// CLAPACK �� LU decomposition
    dgetrf_(&Dim,&Dim,Data,&Dim,ipiv,&INFO);

	if (INFO != 0)
	{
		cout << "Fail to Calculate LU decomposition" << endl;
		system("pause");
	}
	// CLAPACK �� �x�} inverse
    dgetri_(&Dim,Data,&Dim,ipiv,work,&Dim,&INFO);

	if (INFO != 0)
	{
		cout << "Fail to Calculate Matrix Inversion" << endl;
		system("pause");
	}

	// �M���ʺA�O����
	delete[] work;
	delete[] ipiv;

}


double AbsDetSqMat(double * Data, int NCol)
{
	/******************************************************************
	input: �x�}�����}�Y���СA�x�}column��
	output: �^�Ǧ�C���Ȫ������

	Note:
	// �`�N!! ���ާ@�u�^�ǥ��ȡA���YDeterimant�O�t���A�|�^�ǥ���
	// ��deterimant ���ӯ}�a�x�}�ȡA�n���M���᪺Jacobian�|�Τ��F
	******************************************************************/

	double detValue = 1;

	// copy the elements of data
	double* LU = new double[NCol*NCol];
	for (int i =0 ; i < NCol*NCol ; i++)
		LU[i] = Data[i];

	// �ǳƭn��CLAPACK�� LU decomposition
	// ���t�O����P�Ы��ܼƥH�Ѩϥ�
	integer Dim = NCol;
	doublereal *work = new doublereal[Dim*Dim];
    integer lwork = Dim;
    integer INFO; 
    integer * ipiv = new integer[Dim*Dim];

	// LU decomposition
    dgetrf_(&Dim,&Dim,LU,&Dim,ipiv,&INFO);

	// detValue = U�x�}���﨤�u�s��
	for (int j = 0; j < Dim; j++)
		detValue *= LU[j*Dim+j];

	if (detValue < 0)
		detValue = -detValue;

	// ����ʺA�O����
	delete[] work;
	delete[] ipiv;
	delete[] LU;

	return detValue;

}


void SubMat(double *MMother, int ColMother, int RowMother, double *MSon, int *CutRegion)
{
	/******************************************************************
	input: *MMother�x�}���СAColMother���x�}column�ơARowMother���x�}row�ơA*MSon�l�x�}���СA*CutRegion�ŵ��϶�
	output: void

	Note:
	// ���o�n�ŤU�x�}���L�סA CutRegion[0]�N��Row���U��  
	   CutRegion[1]�N��Row���W�� CutRegion[2]�N��Col���U��  CutRegion[3]�N��Col���W��
	// ������۰šA�n���۰šA�Шϥ�SubMatIndex
	******************************************************************/
	int RowSon = CutRegion[1]-CutRegion[0]+1; 
	int ColSon = CutRegion[3]-CutRegion[2]+1;

	// �x�}�Ũ�
	for (int i = 0 ; i < RowSon ; i++)
		for (int j = 0 ; j < ColSon ; j++)
			MSon[i*ColSon+j] = MMother[(i+CutRegion[0]-1)*ColMother+CutRegion[2]+j-1];


}


void SubMatIndex(double *MMother, int ColMother, int RowMother, double *MSon, int *ColRegion, int *RowRegion)
{
	/******************************************************************
	input: *MMother�x�}���СAColMother���x�}column�ơARowMother���x�}row�ơA*MSon�l�x�}���СA*ColRegion *RowRegion ���ŵ��϶�
	output: void

	Note:
	      �򥻤W���P��SubMat()�A�h�F�i�H���۰Ū��\��
	******************************************************************/	


	int RowSon = RowRegion[0]; // ��i�h���Ĥ@�ӭȡA�N��ťX�Ӫ��x�}����(���XRow) �᭱�������N��n�����Row�ơA�i�H���۶�
	int ColSon = ColRegion[0]; // ��i�h���Ĥ@�ӭȡA�N��ťX�Ӫ��x�}����(���XCol) �᭱�������N��n�����Col�ơA�i�H���۶�

	// �x�}�Ũ�
	for (int i = 1 ; i <= RowSon ; i++)
		for (int j = 1 ; j <= ColSon ; j++)
			MSon[(i-1)*ColSon+j-1] = MMother[(RowRegion[i]-1)*ColMother+ColRegion[j]-1];

}

void MatMulAB(double* A, int MA, int NA, double* B, int MB, int NB, double* result)
{

	/******************************************************************
	input:  A �x�}A�}�Y���� , MA �x�}A row�� , NA �x�}A col��, B �x�}B�}�Y����,  MB �x�}B row��,  NB �x�}B col��, result �x�s���G�}�Y����
	output: void

	Note:
	// �x�}���k ��ӯx�}�����׭n��� ���i�H�O���i�����x�}
	******************************************************************/

	if (NA != MB)
	{
		printf("Wrong Matrix Dimension for Mat_Mul");
		system("pause");
	}

	double tempAcc = 0;
	int p = 0;
	int r = 0;

	// ����x�}���k
	for (int i = 0; i < MA*NB ; i++)
	{
		for (int j = 0 ; j < NA; j ++)
		{
			tempAcc += A[p*NA+j]*B[r+j*NB];
		}
		result[i] = tempAcc;
		tempAcc = 0;

		if (r == NB-1)
		{
			r = 0;
			p += 1;
		}
		else
		{
			r += 1;
		}

	}
}

//
//void MatMulAB(double* A, int MA, int NA, double* B, int MB, intNB, double* result)
//{
//
//	if (NA != MB)
//	{
//		printf("Wrong Matrix Dimension for Mat_Mul");
//		system("pause");
//	}
//
//	// C++����
//	//double tempAcc = 0;
//	//int p, r;
//
//	//for (int i = 0; i < MA*NB ; i++)
//	//{
//	//	p = i/NB;
//	//	r  =i%NB;
//	//	for (int j = 0 ; j < NA; j ++)
//	//	{
//	//		//for (int k = 0 ; k<MB ; k++)
//	//			tempAcc += A[p*NA+j]*B[r+j*NB];
//	//	}
//	//	result[i] = tempAcc;
//	//	tempAcc = 0;
//	//}
//
//	float dummy;
//	int Cnt = MA*NB-1;
//
//	__asm{
//
//
//		
//			mov ch, 8 // ch = 8 because double = 8 bytes
//			mov eax, 8 
//
//			mul NB    // jump a row = 8 * NB
//			mov cl, al // move jump a row to cl
//
//			mov esi, A	 // load matrix
//			mov edi, B	 // load matrix
//
//
//		mul_loop: // �p��x�}���k
//
//				mov esi, A	 // load matrix
//				mov edi, B	 // load matrix
//
//			
//				//mov eax, 0
//				mov edx,0 // initialize before div
//				mov eax, Cnt // move the index Cnt to eax, ���V��J��X�x�}����m
//				div NB   // �D�o�ĴX��A�ĴX�C�A�|�Q�s�b bh, bl
//
//				mov bh, al // al, eax = p
//				mov bl, dl // dl, edx = r
//
//				mul NA
//				add eax, edx
//
//				mov edx, 0
//				mov dl, ch
//				mul edx
//
//				add esi, eax
//
//
//				mov eax, 0 // clear ���줸
//				mov al, dl
//				mul NB
//				add al, bl
//				mul ch
//				add edi, eax
//
//				mov dl, 0  // clear dl, �� for �j���
//				mov dh, byte ptr NA
//
//				fldz		 // st = 0
//
//			add_loop:
//
//				fld qword ptr [esi]
//				fmul qword ptr [edi]
//				fadd st(1),st(0)
//				fstp dummy
//
//				inc dl  // j ++
//
//				add esi,8
//				mov eax, 0
//				mov al, cl
//				add edi, eax
//
//				cmp dl, dh
//				jnz	add_loop
//
//				mov eax, Cnt
//
//				mov ebx, 8
//				mul ebx
//				add eax, result
//			
//				fstp qword ptr [eax]
//
//				dec Cnt
//				cmp Cnt, -1
//				jz end_this_loop
//
//				jmp mul_loop
//
//			end_this_loop:
//
//	}
//
//}


//void MatMulAB(double* A, int MA, int NA, double* B, int MB, int NB, double* result)
//{
//
//	if (NA != MB)
//	{
//		printf("Wrong Matrix Dimension for Mat_Mul");
//		system("pause");
//	}
//
//	// C++����
//	//double tempAcc = 0;
//	//int p, r;
//
//	//for (int i = 0; i < MA*NB ; i++)
//	//{
//	//	p = i/NB;
//	//	r  =i%NB;
//	//	for (int j = 0 ; j < NA; j ++)
//	//	{
//	//		//for (int k = 0 ; k<MB ; k++)
//	//			tempAcc += A[p*NA+j]*B[r+j*NB];
//	//	}
//	//	result[i] = tempAcc;
//	//	tempAcc = 0;
//	//}
//
//	float dummy;
//	int Cnt = MA*NB-1;
//
//	__asm{
//
//
//		
//			mov ch, 8 // ch = 8 because double = 8 bytes
//			mov eax, 8 
//
//			mul NB    // jump a row = 8 * NB
//			mov cl, al // move jump a row to cl
//
//			mov esi, A	 // load matrix
//			mov edi, B	 // load matrix
//
//
//		mul_loop: // �p��x�}���k
//
//				mov esi, A	 // load matrix
//				mov edi, B	 // load matrix
//
//			
//				//mov eax, 0
//				mov edx,0 // initialize before div
//				mov eax, Cnt // move the index Cnt to eax, ���V��J��X�x�}����m
//				div NB   // �D�o�ĴX��A�ĴX�C�A�|�Q�s�b bh, bl
//
//				mov bh, al // al, eax = p
//				mov bl, dl // dl, edx = r
//
//				mul NA
//				add eax, edx
//
//				mov edx, 0
//				mov dl, ch
//				mul edx
//
//				add esi, eax
//
//
//				mov eax, 0 // clear ���줸
//				mov al, dl
//				mul NB
//				add al, bl
//				mul ch
//				add edi, eax
//
//				mov dl, 0  // clear dl, �� for �j���
//				mov dh, byte ptr NA
//
//				fldz		 // st = 0
//
//			add_loop:
//
//				fld qword ptr [esi]
//				fmul qword ptr [edi]
//				fadd st(1),st(0)
//				fstp dummy
//
//				inc dl  // j ++
//
//				add esi,8
//				mov eax, 0
//				mov al, cl
//				add edi, eax
//
//				cmp dl, dh
//				jnz	add_loop
//
//				mov eax, Cnt
//
//				mov ebx, 8
//				mul ebx
//				add eax, result
//			
//				fstp qword ptr [eax]
//
//				dec Cnt
//				cmp Cnt, -1
//				jz end_this_loop
//
//				jmp mul_loop
//
//			end_this_loop:
//
//	}
//
//}


//
//// �Ĥ@���Aí�w�O�A�i�H�B�z���j�x�}
//void MatMulAB(double* A, int MA, int NA, double* B, int MB, int NB, double* result)
//{
//
//	if (NA != MB)
//	{
//		printf("Wrong Matrix Dimension for Mat_Mul");
//		system("pause");
//	}
//
//	double dummy;
//
//	//int S_D = 8; // size of double
//	int p, r;
//	int warp = NB*8;
//
//	__asm{
//
//			
//
//			mov eax, MA
//			//mov esi, NB  // esi = row �ƶq
//			mul NB      // eax = eax * NB
//			dec eax
//			mov ecx, eax 
//
//
//		mul_loop: // �p��x�}���k
//
//			//fldz		 // st = 0
//			//fld1		 // st = 1 0
//
//			mov esi, A	 // load matrix
//			mov edi, B	 // load matrix
//
//			mov eax, ecx
//
//			mov edx, 0
//			mov ebx, 0
//			div NB		 // eax = �� edx = �l��	
//			mov r, edx
//			mov p, eax
//
//				// get p*NA+j
//				//mov eax, p
//				mul NA		 // p*NA
//				add eax, ebx		 // p*NA+j
//				rol eax, 3	// to byte address
//				add esi, eax
//
//				mov eax,ebx // j          
//				mul NB		// j*NB
//				add eax,r // r+ j*NB
//				rol eax, 3		// to byte address
//				add edi, eax 
//				
//				fldz		 // st = 0
//
//			add_loop:
//
//
//				fld qword ptr [esi]
//				fmul qword ptr [edi]
//				fadd st(1),st(0)
//				fstp dummy
//	
//
//				add esi,8
//				add edi,warp
//
//				inc ebx
//				mov eax, NA
//				cmp ebx, eax
//
//				jnz	add_loop
//
//				mov eax, ecx
//				rol eax, 3
//		
//				add eax, result
//				fstp qword ptr [eax]
//
//				dec ecx
//				cmp ecx, -1
//				jnz mul_loop
//
//	}
//
//}

void MatMulAtB(double* A, int MA, int NA, double* B, int MB, int NB, double* result)
{
	/******************************************************************
	input:  A �x�}A�}�Y���� , MA �x�}A row�� , NA �x�}A col��, B �x�}B�}�Y����,  MB �x�}B row��,  NB �x�}B col��, result �x�s���G�}�Y����
	output: void

	Note:
	// �x�}���k ��ӯx�}�����׭n��� ���i�H�O���i�����x�}
	// �]�N�O�� transpose(A)*B �n�i�� 
	// �åB��J��MA NA �OA�쥻������!!
	******************************************************************/
	if (MA != MB)
	{
		printf("Wrong Matrix Dimension for Mat_Mul");
		system("pause");
	}

	int p = 0;
	int r = 0;

	double tempAcc = 0;

	// ����x�}���k
	for (int i = 0; i < NA*NB ; i++)
	{
		for (int j = 0 ; j < MA; j ++)
		{
				tempAcc += A[j*NA+p]*B[r+j*NB];
		}
		result[i] = tempAcc;
		tempAcc = 0;

		if (r == NB-1)
		{
			r = 0;
			p += 1;
		}
		else
		{
			r += 1;
		}
	}
}


void MatMulABt(double* A, int MA, int NA, double* B, int MB, int NB, double* result)
{
	/******************************************************************
	input:  A �x�}A�}�Y���� , MA �x�}A row�� , NA �x�}A col��, B �x�}B�}�Y����,  MB �x�}B row��,  NB �x�}B col��, result �x�s���G�}�Y����
	output: void

	Note:
	// �x�}���k ��ӯx�}�����׭n��� ���i�H�O���i�����x�}
	// �]�N�O�� A*transpose(B) �n�i�� 
	// �åB��J��MB NB �OB�쥻������!!
	******************************************************************/

	if (NA != NB)
	{
		printf("Wrong Matrix Dimension for Mat_Mul");
		system("pause");
	}

	int p = 0;
	int r = 0;
	double tempAcc = 0;

	// ����x�}���k
	for (int i = 0; i < MA*MB ; i++)
	{
		for (int j = 0 ; j < NA; j ++)
		{
				tempAcc += A[p*NA+j]*B[j+r*NB];
		}
		result[i] = tempAcc;
		tempAcc = 0;

		if (r == MB-1)
		{
			r = 0;
			p += 1;
		}
		else
		{
			r += 1;
		}


	}
}


void MatScalarMul(double* A, int LenA, double* Scalar, double* result)
{
	/******************************************************************
	input:  A �x�}A�}�Y���� , LenA �x�}A ������ , *Scalar ��j���v, result �x�s���G�}�Y����
	output: void

	Note: None
	******************************************************************/

	for (int i=0; i<LenA; i++)
	{
		result[i] = A[i]*Scalar[0];
	}
}

void MatScalarMul(double* A, int LenA, double Scalar, double* result)
{
	/******************************************************************
	input:  A �x�}A�}�Y���� , LenA �x�}A ������ , *Scalar ��j���v, result �x�s���G�}�Y����
	output: void

	Note: None
	******************************************************************/

	for (int i=0; i<LenA; i++)
	{
		result[i] = A[i]*Scalar;
	}
}

void MatMiuAB(double* A, double* B, double* result, int Len) // �x�}�۴�
{
	/******************************************************************
	input:  A �x�}A�}�Y����, B �x�}B�}�Y����,  result �x�s���G�}�Y����, Len ����x�}��������
	output: void

	Note:
	// �x�}�[��k ��ӯx�}�����׭n�@�� 

	// ��l�X
	for (int i = 0; i < Len ;  i++)
		result[i] = A[i]-B[i];

	******************************************************************/

	// �զX�y��
	__asm{

		mov eax, A
		mov ebx, B
		mov edx, result

		mov ecx, Len // �O�h�� �N�|�i�h�X���^��

	// for loop
	mat_miu: // ecx �|�۰ʴ�@����0 ���X�^��

		fld QWORD PTR [eax] // load A[i] to FPU stack 0 ( st(0) )
		fsub QWORD PTR [ebx] // add B[i] to st(0)
		
		fstp QWORD PTR [edx] // pop the value 

		add eax,8
		add ebx,8
		add edx,8

		dec ecx
		cmp ecx, 0
		jnz mat_miu
		//loop mat_add
	 //for loop

	}



}

void MatAddAB(double* A, double* B, double* result, int Len) // �x�}�ۥ[
{
	/******************************************************************
	input:  A �x�}A�}�Y����, B �x�}B�}�Y����,  result �x�s���G�}�Y����, Len ����x�}��������
	output: void

	Note:
	// �x�}�[��k ��ӯx�}�����׭n�@�� 

	// ��l�X
	for (int i = 0; i < Len ;  i++)
		result[i] = A[i]+B[i];

	******************************************************************/

	__asm{

		mov eax, A
		mov ebx, B
		mov edx, result

		mov ecx, Len // �O�h�� �N�|�i�h�X���^��

	// for loop
	mat_add: // ecx �|�۰ʴ�@����0 ���X�^��

		fld QWORD PTR [eax] // load A[i] to FPU stack 0 ( st(0) )
		fadd QWORD PTR [ebx] // add B[i] to st(0)
		
		fstp QWORD PTR [edx] // pop the value 

		add eax,8
		add ebx,8
		add edx,8

		//loop mat_add

		dec ecx
		cmp ecx, 0
		jnz mat_add

	// for loop

	}

}