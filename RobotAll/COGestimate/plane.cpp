
#include <cmath>
#include <iostream>
#include "GlobalConst.h"
#include "plane.h"
using namespace std;


cPlaneEKF::cPlaneEKF() 
{  	 

    setDim(3, 1, 3, 3, 3);  //設定matrix dimention  n state = 4 (X = A X   X vector 的大小)   m (measurement vector  的大小)
	/*設定matrix dimention
		setDim setDim  (  
		K_UINT_32  n_,    # of state      
		K_UINT_32  nu_,   # of input 
		K_UINT_32  nw_,   # of standrd process 
		K_UINT_32  m_,    # of measure  
		K_UINT_32  nv_    # of measurement   noise 

    K_UINT_32 n;        //!< Size of the state vector.
    K_UINT_32 nu;       //!< Size of the input vector.
    K_UINT_32 nw;       //!< Size of the process noise vector.
    K_UINT_32  m;       //!< Size of the measurement vector.
    K_UINT_32 nv;       //!< Size of the measurement noise vector.


*/
	   COG_height = 0.666 ; // COG 離地高度 (單位mm)
	   samplingtime = 0.005;    //step time 5ms 
	   ww = sqrt( 9.8/ COG_height) ;
	   mass = 730; //機器人質量 單位 kg  from adams z-force
	  

}



void cPlaneEKF::makeBaseA()
{
    A(1,1) = 1.0;
	A(1,2) = samplingtime;
	A(1,3) = 0.0;

	A(2,1) = ww*ww*samplingtime;
	A(2,2) = 1;
	A(2,3) = -ww*ww*samplingtime;
	
	A(3,1) = 0.0;
	A(3,2) = 0.0;
	A(3,3) = 1.0;


	}


void cPlaneEKF::makeA()
{
	// A(1,1) = 1.0;
	//A(1,2) = samplingtime;
	//// A(1,3) = 0.0;
	//// A(1,4) = 0.0;

	//// A(2,1) = 0.0;
	//A(2,2) = 1;
	//// A(2,3) = 0.0;
	//// A(2,4) = 0.0;

	//// A(3,1) = 0.0;
	//A(3,2) = 0;
	// A(3,3) = 1.0;
	// A(3,4) = Period;
	

}



void cPlaneEKF::makeBaseW()   //根據f 做partial w 為process noise 每個state 都有process noise 
{
	W(1,1) = 1.0; 
	W(1,2) = 0.0;
	W(1,3) = 0.0;
	
	W(2,1) = 0.0;
	W(2,2) = 1.0;
	W(2,3) = 0.0;
		
	W(3,1) = 0.0;
	W(3,2) = 0.0;
	W(3,3) = 1.0;





}



void cPlaneEKF::makeBaseQ()  //standard  process covariance matrix 為 4X4 描述各個variable間的covariance 
{     
	//自己跟自己 0.001 
	Q(1,1) = 0.001;          //將x(4) external force 調大  讓其相信measurement
	Q(1,2) = 0.001/50;
	Q(1,3) = 0.001/50;
	
	Q(2,1) = 0.001/50;
	Q(2,2) = 0.001;
	Q(2,3) = 0.001/50;
	
	Q(3,1) = 0.001/50;
	Q(3,2) = 0.001/50;
	Q(3,3) = 0.001;



}





void cPlaneEKF::makeBaseH()  ///matrix C 
{
	H(1,1) = 1.0;
	H(1,2) = 0.0;
	H(1,3) = 0.0;
	
	H(2,1) = 0.0;
	H(2,2) = 0.0;
	H(2,3) = 1.0;
	
	H(3,1) = ww*ww;
	H(3,2) = 0.0;
	H(3,3) = -ww*ww;
	


}







void cPlaneEKF::makeH()
{
	/*H(1,1) = 1.0;
	H(1,2) = 0.0;
	H(1,3) = 0.0;
	
	H(2,1) = 0.0;
	H(2,2) = 0.0;
	H(2,3) = 1.0;
	
	H(3,1) = ww*ww;
	H(3,2) = 0.0;
	H(3,3) = -ww*ww;*/
}



void cPlaneEKF::makeBaseV()
{
	V(1,1) = 1.0;
	V(1,2) = 0.0;
	V(1,3) = 0.0;
	
	
	V(2,1) = 0.0;
	V(2,2) = 1.0;
    V(2,3) = 0.0;
	

	V(3,1) = 0.0;
	V(3,2) = 0.0;
    V(3,3) = 1.0;
	
}

void cPlaneEKF::makeBaseR()  //measurement noise  (依據每個sensor會有不同) 測量方式:將sensor平放，獲得covariance
{
	if (AdamsSMode)
	{
	//process noise = 0.001
	//ADAMS MODE 參數
	R(1,1) = 0.01;  // 由joint encoder 回算的COG 位置
	R(2,2) = 0.0001;  // 由力規算出的 ZMP
	R(3,3) = 0.0001;  //IMU 量測的 COG 加速度 
	}

	else
	{
	////EXP MODE 參數
	R(1,1) = 0.0001;  // 由joint encoder 回算的COG 位置
	R(2,2) = 0.001223;  // 由力規算出的 ZMP
	R(3,3) = 0.00022;  //IMU 量測的 COG 加速度 
	}


}


void cPlaneEKF::makeProcess() //  doing model  描述state間的關係 依據matrix A   
{
	Vector x_(x.size());
	
	x_(1) = x(1) +  samplingtime*x(2);
	x_(2) = ww*ww*samplingtime*x(1)+x(2)-ww*ww*samplingtime*x(3)+(samplingtime/mass)* x(4);
	x_(3) = x(3); 
	x.swap(x_);
}


void cPlaneEKF::makeMeasure()    //描述measurement與state間的關係  依據 matrix C  x(4) 為 (1/kg)*N   1/mass*x(4)*1000 要乘1000 
{
	z(1)=x(1);
	z(2)=x(3);
	z(3)= ww*ww*x(1) - ww*ww*x(3) ;
	
}

//轉動
rPlaneEKF::rPlaneEKF() 
{  	 

    setDim(7, 0, 7, 7, 7);  //設定matrix dimention  n state = 4 (X = A X   X vector 的大小)   m (measurement vector  的大小)
	/*設定matrix dimention
		setDim setDim  (  
		K_UINT_32  n_,    # of state      
		K_UINT_32  nu_,   # of input 
		K_UINT_32  nw_,   # of standrd process 
		K_UINT_32  m_,    # of measure  
		K_UINT_32  nv_    # of measurement   noise 

    K_UINT_32 n;        //!< Size of the state vector.
    K_UINT_32 nu;       //!< Size of the input vector.
    K_UINT_32 nw;       //!< Size of the process noise vector.
    K_UINT_32  m;       //!< Size of the measurement vector.
    K_UINT_32 nv;       //!< Size of the measurement noise vector.

	*/
 
	    // COG 離地高度 (單位mm)
	   samplingtime = 0.005;    //step time 5ms 
	   COG_height = 0.666 ; 
	   ww = sqrt( 9.8/ COG_height) ;
	  

}

void rPlaneEKF::makeBaseA()
{
    //A(1,1) = 1.0;
	A(1,2) = 0.0;
	A(1,3) = 0.0;
	A(1,4) = 0.0;
	//A(1,5) = 0.0;
	//A(1,6) = 0.0;
	//A(1,7) = 0.0;

	A(2,1) = 0.0;
	//A(2,2) = 0.0;
	A(2,3) = 0.0;
	A(2,4) = 0.0;
	//A(2,5) = 0.0;
	//A(2,6) = 0.0;
	//A(2,7) = 0.0;
	
	A(3,1) = 0.0;
	A(3,2) = 0.0;
	//A(3,3) = 0.0;
	A(3,4) = 0.0;
	//A(3,5) = 0.0;
	//A(3,6) = 0.0;
	//A(3,7) = 0.0;

	A(4,1) = 0.0;
	A(4,2) = 0.0;
	A(4,3) = 0.0;
	//A(4,4) = 0.0;
	//A(4,5) = 0.0;
	//A(4,6) = 0.0;
	//A(4,7) = 0.0;

	A(5,1) = 0.0;
	A(5,2) = 0.0;
	A(5,3) = 0.0;
	A(5,4) = 0.0;
	A(5,5) = 1.0;
	A(5,6) = 0.0;
	A(5,7) = 0.0;

	A(6,1) = 0.0;
	A(6,2) = 0.0;
	A(6,3) = 0.0;
	A(6,4) = 0.0;
	A(6,5) = 0.0;
	A(6,6) = 1.0;
	A(6,7) = 0.0;

	A(7,1) = 0.0;
	A(7,2) = 0.0;
	A(7,3) = 0.0;
	A(7,4) = 0.0;
	A(7,5) = 0.0;
	A(7,6) = 0.0;
	A(7,7) = 1.0;

}

void rPlaneEKF::makeA()
{
	A(1,1) = cos( 1/2*samplingtime*sqrt( x(5)*x(5)+x(6)*x(6)+x(7)*x(7) ) );
	//A(1,2) = 0.0;
	//A(1,3) = 0.0;
	//A(1,4) = 0.0;
	A(1,5) = - x(2)*1/2*samplingtime;
	A(1,6) = - x(3)*1/2*samplingtime;
	A(1,7) = - x(4)*1/2*samplingtime;

	//A(2,1) = 0.0;
	A(2,2) = cos( 1/2*samplingtime*sqrt( x(5)*x(5)+x(6)*x(6)+x(7)*x(7) ) );
	//A(2,3) = 0.0;
	//A(2,4) = 0.0;
	A(2,5) =   x(1)*1/2*samplingtime;
	A(2,6) = - x(4)*1/2*samplingtime;
	A(2,7) =   x(3)*1/2*samplingtime;
	
	//A(3,1) = 0.0;
	//A(3,2) = 0.0;
	A(3,3) = cos( 1/2*samplingtime*sqrt( x(5)*x(5)+x(6)*x(6)+x(7)*x(7) ) );
	//A(3,4) = 0.0;
	A(3,5) =   x(4)*1/2*samplingtime;
	A(3,6) =   x(1)*1/2*samplingtime;
	A(3,7) =   x(2)*1/2*samplingtime;

	//A(4,1) = 0.0;
	//A(4,2) = 0.0;
	//A(4,3) = 0.0;
	A(4,4) = cos( 1/2*samplingtime*sqrt( x(5)*x(5)+x(6)*x(6)+x(7)*x(7) ) );
	A(4,5) = - x(3)*1/2*samplingtime;
	A(4,6) =   x(2)*1/2*samplingtime;
	A(4,7) =   x(1)*1/2*samplingtime;
	

}

void rPlaneEKF::makeBaseW()   //根據f 做partial w 為process noise 每個state 都有process noise 
{
	W(1,1) = 1.0; 
	W(1,2) = 0.0;
	W(1,3) = 0.0;
	W(1,4) = 0.0;
	W(1,5) = 0.0;
	W(1,6) = 0.0;
	W(1,7) = 0.0;
	
	W(2,1) = 0.0;
	W(2,2) = 1.0;
	W(2,3) = 0.0;
	W(2,4) = 0.0;
	W(2,5) = 0.0;
	W(2,6) = 0.0;
	W(2,7) = 0.0;
		
	W(3,1) = 0.0;
	W(3,2) = 0.0;
	W(3,3) = 1.0;
	W(3,4) = 0.0;
	W(3,5) = 0.0;
	W(3,6) = 0.0;
	W(3,7) = 0.0;

	W(4,1) = 0.0;
	W(4,2) = 0.0;
	W(4,3) = 0.0;
	W(4,4) = 1.0;
	W(4,5) = 0.0;
	W(4,6) = 0.0;
	W(4,7) = 0.0;

	W(5,1) = 0.0;
	W(5,2) = 0.0;
	W(5,3) = 0.0;
	W(5,4) = 0.0;
	W(5,5) = 1.0;
	W(5,6) = 0.0;
	W(5,7) = 0.0;

	W(6,1) = 0.0;
	W(6,2) = 0.0;
	W(6,3) = 0.0;
	W(6,4) = 0.0;
	W(6,5) = 0.0;
	W(6,6) = 1.0;
	W(6,7) = 0.0;

	W(7,1) = 0.0;
	W(7,2) = 0.0;
	W(7,3) = 0.0;
	W(7,4) = 0.0;
	W(7,5) = 0.0;
	W(7,6) = 0.0;
	W(7,7) = 1.0;

}


void rPlaneEKF::makeBaseQ()  //standard  process covariance matrix 為 4X4 描述各個variable間的covariance 
{     
	//自己跟自己 0.001   
	Q(1,1) =0.001;         //將x(4) external force 調大  讓其相信measurement
	Q(1,2) = 0.001/50;
	Q(1,3) = 0.001/50;
	Q(1,4) = 0.001/50;
	Q(1,5) = 0.001/50;
	Q(1,6) = 0.001/50;
	Q(1,7) = 0.001/50;
	
	Q(2,1) = 0.001/50;   
	Q(2,2) = 0.001;
	Q(2,3) = 0.001/50;
	Q(2,4) = 0.001/50;
	Q(2,5) = 0.001/50;
	Q(2,6) = 0.001/50;
	Q(2,7) = 0.001/50;
	
	Q(3,1) = 0.001/50;
	Q(3,2) = 0.001/50;
	Q(3,3) = 0.001;
	Q(3,4) = 0.001/50;
	Q(3,5) = 0.001/50;
	Q(3,6) = 0.001/50;
	Q(3,7) = 0.001/50;

	Q(4,1) = 0.001/50;
	Q(4,2) = 0.001/50;
	Q(4,3) = 0.001/50;
	Q(4,4) = 0.001;
	Q(4,5) = 0.001/50;
	Q(4,6) = 0.001/50;
	Q(4,7) = 0.001/50;

	Q(5,1) = 0.001/50;   
	Q(5,2) = 0.001/50;
	Q(5,3) = 0.001/50;
	Q(5,4) = 0.001/50;
	Q(5,5) = 0.001;
	Q(5,6) = 0.0;
	Q(5,7) = 0.0;

	Q(6,1) = 0.001/50;      
	Q(6,2) = 0.001/50;
	Q(6,3) = 0.001/50;
	Q(6,4) = 0.001/50;
	Q(6,5) = 0.0;
	Q(6,6) = 0.001;
	Q(6,7) = 0.0;

	Q(7,1) = 0.001/50;      
	Q(7,2) = 0.001/50;
	Q(7,3) = 0.001/50;
	Q(7,4) = 0.001/50;
	Q(7,5) = 0.0;
	Q(7,6) = 0.0;
	Q(7,7) = 0.001;


}


void rPlaneEKF::makeBaseH()  ///matrix C 
{
	H(1,1) = 1.0;
	H(1,2) = 0.0;
	H(1,3) = 0.0;
	H(1,4) = 0.0;
	H(1,5) = 0.0;
	H(1,6) = 0.0;
	H(1,7) = 0.0;
	
	H(2,1) = 0.0;
	H(2,2) = 1.0;
	H(2,3) = 0.0;
	H(2,4) = 0.0;
	H(2,5) = 0.0;
	H(2,6) = 0.0;
	H(2,7) = 0.0;
	
	H(3,1) = 0.0;
	H(3,2) = 0.0;
	H(3,3) = 1.0;
	H(3,4) = 0.0;
	H(3,5) = 0.0;
	H(3,6) = 0.0;
	H(3,7) = 0.0;

	H(4,1) = 0.0;
	H(4,2) = 0.0;
	H(4,3) = 0.0;
	H(4,4) = 1.0;
	H(4,5) = 0.0;
	H(4,6) = 0.0;
	H(4,7) = 0.0;

	H(5,1) = 0.0;
	H(5,2) = 0.0;
	H(5,3) = 0.0;
	H(5,4) = 0.0;
	H(5,5) = 1.0;
	H(5,6) = 0.0;
	H(5,7) = 0.0;

	H(6,1) = 0.0;
	H(6,2) = 0.0;
	H(6,3) = 0.0;
	H(6,4) = 0.0;
	H(6,5) = 0.0;
	H(6,6) = 1.0;
	H(6,7) = 0.0;

	H(7,1) = 0.0;
	H(7,2) = 0.0;
	H(7,3) = 0.0;
	H(7,4) = 0.0;
	H(7,5) = 0.0;
	H(7,6) = 0.0;
	H(7,7) = 1.0;


}


void rPlaneEKF::makeH()
{
	

}


void rPlaneEKF::makeBaseV()
{
	V(1,1) = 1.0;
	V(1,2) = 0.0;
	V(1,3) = 0.0;
	V(1,4) = 0.0;
	V(1,5) = 0.0;
	V(1,6) = 0.0;
	V(1,7) = 0.0;
	
	V(2,1) = 0.0;
	V(2,2) = 1.0;
    V(2,3) = 0.0;
	V(2,4) = 0.0;
	V(2,5) = 0.0;
	V(2,6) = 0.0;
	V(2,7) = 0.0;

	V(3,1) = 0.0;
	V(3,2) = 0.0;
    V(3,3) = 1.0;
	V(3,4) = 0.0;
	V(3,5) = 0.0;
	V(3,6) = 0.0;
	V(3,7) = 0.0;

	V(4,1) = 0.0;
	V(4,2) = 0.0;
    V(4,3) = 0.0;
	V(4,4) = 1.0;
	V(4,5) = 0.0;
	V(4,6) = 0.0;
	V(4,7) = 0.0;

	V(5,1) = 0.0;
	V(5,2) = 0.0;
    V(5,3) = 0.0;
	V(5,4) = 0.0;
	V(5,5) = 1.0;
	V(5,6) = 0.0;
	V(5,7) = 0.0;

	V(6,1) = 0.0;
	V(6,2) = 0.0;
    V(6,3) = 0.0;
	V(6,4) = 0.0;
	V(6,5) = 0.0;
	V(6,6) = 1.0;
	V(6,7) = 0.0;

	V(7,1) = 0.0;
	V(7,2) = 0.0;
    V(7,3) = 0.0;
	V(7,4) = 0.0;
	V(7,5) = 0.0;
	V(7,6) = 0.0;
	V(7,7) = 1.0;
	
}

void rPlaneEKF::makeBaseR()  //measurement noise  (依據每個sensor會有不同) 測量方式:將sensor平放，獲得covariance
{

	////EXP MODE 參數
	R(1,1) = 1.0604e-009;  // w
	R(2,2) = 6.6062e-007;  // x
	R(3,3) = 1.5372e-007;  // y
	R(4,4) = 2.3701e-006;  // z
	R(5,5) = 2.7086e-005;  // angular velocity x 
	R(6,6) = 3.5514e-005;  // angular velocity y
	R(7,7) = 2.0369e-005;  // angular velocity z 
	
 
	//ADAMS MODE 參數
	//R(1,1) = 0.0001;   // angle x
	//R(2,2) = 0.001223; // angle y
	//R(3,3) = 0.00022;  // angle z
	//R(4,4) = 0.00022;  // angular velocity x 
	//R(5,5) = 0.00022;  // angular velocity y
	//R(6,6) = 0.00022;  // angular velocity z 
	


}


void rPlaneEKF::makeProcess() //  doing model  描述state間的關係 依據matrix A   
{
	Vector x_(x.size());
	
	x_(1) = x(1)*cos( 1/2*samplingtime*sqrt( x(5)*x(5)+x(6)*x(6)+x(7)*x(7) ) ) - 1/2*x(5)*x(2)*samplingtime - 1/2*x(6)*x(3)*samplingtime - 1/2*x(7)*x(4)*samplingtime;
	x_(2) = x(2)*cos( 1/2*samplingtime*sqrt( x(5)*x(5)+x(6)*x(6)+x(7)*x(7) ) ) + 1/2*x(5)*x(1)*samplingtime - 1/2*x(6)*x(4)*samplingtime + 1/2*x(7)*x(3)*samplingtime;
	x_(3) = x(3)*cos( 1/2*samplingtime*sqrt( x(5)*x(5)+x(6)*x(6)+x(7)*x(7) ) ) + 1/2*x(5)*x(4)*samplingtime + 1/2*x(6)*x(1)*samplingtime + 1/2*x(7)*x(2)*samplingtime;
	x_(4) = x(4)*cos( 1/2*samplingtime*sqrt( x(5)*x(5)+x(6)*x(6)+x(7)*x(7) ) ) - 1/2*x(5)*x(3)*samplingtime + 1/2*x(6)*x(2)*samplingtime + 1/2*x(7)*x(1)*samplingtime;
    x_(5) = x(5);
	x_(6) = x(6);
	x_(7) = x(7);
	x.swap(x_);
}


void rPlaneEKF::makeMeasure()    //描述measurement與state間的關係  依據 matrix C  x(4) 為 (1/kg)*N   1/mass*x(4)*1000 要乘1000 
{
	//QuaternionTrans QuaternionTrans1;

	//QuaternionTrans1.w = x(1);
	//QuaternionTrans1.x = x(2);
	//QuaternionTrans1.y = x(3);
	//QuaternionTrans1.z = x(4);

	//EulerAngle ea = QuaternionTrans1.ToEulerAngle();

	//z(1) = ea.fRoll;
	//z(2) = ea.fPitch;
	//z(3) = ea.fYaw;
	z(1) = x(1);
	z(2) = x(2);
	z(3) = x(3);
	z(4) = x(4);
	z(5) = x(5);
	z(6) = x(6);
	z(7) = x(7);
	
}


