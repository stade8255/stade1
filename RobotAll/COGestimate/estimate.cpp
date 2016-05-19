#include "stdafx.h"
#include "estimate.h"
#include <vector>
#include <iomanip>//控制cout 輸出小數點位數

rPlaneEKF rfilter;
cPlaneEKF filter;
extern int gFlagSimulation; 
estimate ::estimate()
{   		
	
	for(int i = 0; i<10000 ; i++)
	{
		Cogstatelateral[i]=0;
		Dcogstatelateral[i]=0;
		state3lateral[i]=0;

		Cogstatesaggital[i]=0;
		Dcogstatesaggital[i]=0;
		state3saggital[i]=0;

		AngleX[i]=0;
		AngleY[i]=0;
		AngleZ[i]=0;

		initflag = false;
		rinitflag = false;
	}


};

int estimate::rcompute ( float anglex ,float  angley , float anglez, float angularratex, float angularratey, float angularratez, float cogaccelx, float cogaccely, float cogaccelz, float &filtercogaccelx, float &filtercogaccely, float &filtercogaccelz, int estimatecount) 
{
	
	const unsigned NTRY = 1200*10; //data數
	
	const unsigned n = 7;	//nb states
	const unsigned m = 7;	//nb measures
	
	QuaternionTrans QuaternionTrans1;

	double  timestep  = 0.005 ; 

	double Rotation[3][3];		// Rotation matrix (Quaternion形式)
	
	double cogaccelationRoll = cogaccely, cogaccelationPitch = cogaccelx, cogaccelationYaw = -cogaccelz;  // 暫存用

	
	Vector x(n);   // 開state vector 
	Vector rstate(n) ;
	Vector z(m);

	EulerAngle ea;		// 暫存運算用之EulerAngle
	
	ea.fRoll = -anglex;
	ea.fPitch = -angley;
	ea.fYaw = anglez;

	QuaternionTrans1.FromEulerAngle(ea);	// 將EulerAngle轉為Quaternion並儲存

	//初始化
	if (rinitflag == false )
	{
		x(1) = QuaternionTrans1.w;
		x(2) = QuaternionTrans1.x;  
		x(3) = QuaternionTrans1.y ;  
		x(4) = QuaternionTrans1.z ; 
		x(5) = -angularratex;  
		x(6) = -angularratey;  
		x(7) = angularratez;  

		static const double _P0[] = { 0.1 ,0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 
								  0.0 ,0.1, 0.0, 0.0, 0.0, 0.0, 0.0, 
								  0.0 ,0.0, 0.1, 0.0, 0.0, 0.0, 0.0,
								  0.0 ,0.0, 0.0, 0.1, 0.0, 0.0, 0.0,
								  0.0 ,0.0, 0.0, 0.0, 0.1, 0.0, 0.0,
								  0.0 ,0.0, 0.0, 0.0, 0.0, 0.1, 0.0,
								  0.0 ,0.0, 0.0, 0.0, 0.0, 0.0, 0.1
	                                   };       //initial error 

		Matrix P0(n, n, _P0); //建立 initial error  matrix  
		rfilter.init(x, P0);

		rinitflag = true;
	}
	

	//******旋轉filter

	/*rz(1) = w;  
	rz(2) = x;  
	rz(3) = y ;  
	rz(4) = z ;  
	rz(5) = wRoll ;  
	rz(6) = wPitch ;  
	rz(7) = wYaw ;  
	*/
		
	z(1) = QuaternionTrans1.w; 
	z(2) = QuaternionTrans1.x;  
	z(3) = QuaternionTrans1.y ;  
	z(4) = QuaternionTrans1.z ; 
	z(5) = -angularratex;  
	z(6) = -angularratey;  
	z(7) = angularratez;  

		
	Vector u(0);

	rfilter.step(u, z);

	rstate   = rfilter.getX()  ;


	// 修正加速度至世界座標軸	
	Rotation[0][0] = rstate(1)*rstate(1) + rstate(2)*rstate(2) - rstate(3)*rstate(3) - rstate(4)*rstate(4);
	Rotation[0][1] = 2*( rstate(2)*rstate(3)-rstate(1)*rstate(4) );
	Rotation[0][2] = 2*( rstate(2)*rstate(4)+rstate(1)*rstate(3) );
	Rotation[1][0] = 2*( rstate(2)*rstate(3)+rstate(1)*rstate(4) );
	Rotation[1][1] = rstate(1)*rstate(1) - rstate(2)*rstate(2) + rstate(3)*rstate(3) - rstate(4)*rstate(4);
	Rotation[1][2] = 2*( rstate(3)*rstate(4)-rstate(1)*rstate(2) );
	Rotation[2][0] = 2*( rstate(2)*rstate(4)-rstate(1)*rstate(3) );
	Rotation[2][1] = 2*( rstate(3)*rstate(4)+rstate(1)*rstate(2) );
	Rotation[2][2] = rstate(1)*rstate(1) - rstate(2)*rstate(2) - rstate(3)*rstate(3) + rstate(4)*rstate(4);

	filtercogaccelx = -(Rotation[1][0]*cogaccely + Rotation[1][1]*cogaccelx - Rotation[1][2]*cogaccelz);
	filtercogaccely = -(Rotation[0][0]*cogaccely + Rotation[0][1]*cogaccelx - Rotation[0][2]*cogaccelz);
	filtercogaccelz = +(Rotation[2][0]*cogaccely + Rotation[2][1]*cogaccelx - Rotation[2][2]*cogaccelz)-9.780318;

	// 將經EKF後的Quaternion轉回EulerAngle
	QuaternionTrans1.w = rstate(1);
	QuaternionTrans1.x = rstate(2);
	QuaternionTrans1.y = rstate(3);
	QuaternionTrans1.z = rstate(4);

	ea = QuaternionTrans1.ToEulerAngle();
		
	AngleX[estimatecount] = -ea.fRoll;
	AngleY[estimatecount] = -ea.fPitch;
	AngleZ[estimatecount] = ea.fYaw;


	return EXIT_SUCCESS;

}


int estimate::compute ( double cog ,double  *zmp ,double  cogaccel,int estimatecount, int direction  ) {
	
	//direction = 1 ,lateral  direction 
	//direction = 2 ,saggital direction  
	
	

	const unsigned NTRY = 1200*10; //data數
	

	int n;	//nb states
	int m;	//nb measures

	if ( filter.ZMPflag == 0 )
	{
		n = 3;
		m = 2;
	}
	else
	{
		n = 3;
		m = 3;
	}

	double  timestep  = 0.005 ; 
	
	double  Dzmp = 0   , Dzmpbefore = 0 ; //前個時刻的zmp 一次微分量 
	double  DDzmp = 0  , DDzmpbefore = 0 ; //前個時刻的zmp 二次微分量 


	double IMU_COG  = 666/732;

	double exp_gain = 1 ;    //cm/s^2  to mm/s^2
		
	Vector x(n);   // 開state vector 
	
	Vector state(n) ;
	
	Vector z(m);
	
	
	//zmp'' 數值微分處理
	
	
	estimate_lock = true;  // lock 避免掉值



	if ( gFlagSimulation == 2 )
	   exp_gain = 100; 
	
		
	
	//P0(0,0) = 0 ;

	// 初始化
	if ( initflag == false )
	{
		if ( filter.ZMPflag == 0 )
		{
			x(1) = cog;
			x(2) = cogaccel* IMU_COG *exp_gain*timestep;
			x(3) = cogaccel* IMU_COG *exp_gain;
		}
		else
		{
			x(1) = cog;
			x(2) = *zmp;
			x(3) = cogaccel* IMU_COG *exp_gain;
		}

		static const double _P0[] = { 0.1 ,0.0, 0.0,  
							   	      0.0, 0.1 , 0.0 , 
									  0.0,  0.0,  0.1 
									};       //initial error 
		Matrix P0(n, n, _P0); //建立 initial error  matrix
		filter.init(x, P0);

		initflag = true;
	}

	
	// 丟入sensor的值
	if ( filter.ZMPflag == 0 )
	{
		z(1) = cog;  
		z(2) = cogaccel* IMU_COG *exp_gain; //將measurement 寫入 IMU安裝位置的比例分配
	}
	else
	{
		z(1) = cog;  
		z(2) = *zmp;  
		z(3) = cogaccel* IMU_COG *exp_gain;  //將measurement 寫入 IMU安裝位置的比例分配
	}
	    
		
	   
	// u  control input    zmp二次微分
		
	if ( estimatecount < 2 ) 
	{
		 Dzmp = 0; 
		 Dzmpbefore  = 0; 
	}
	else
	{	
		Dzmp  =  ( *zmp - *(zmp-2) ) / timestep; 
		Dzmpbefore = ( *(zmp-2) - *(zmp-4)  ) / timestep; 
	}

	DDzmp = ( Dzmp - Dzmpbefore )/ timestep; 

	Vector u(1);
	//u(1) = DDzmp;
	u(1) = 0;
		
	filter.step(u, z);

	state  = filter.getX();

		
		
	if ( direction == 1 )
	{
		Cogstatelateral[estimatecount] =  state (1)  ;  //將state存入  cogstate matrix  
		Dcogstatelateral[estimatecount] = state (2)  ;
		state3lateral[estimatecount] =  state (3)  ;
	}
	else if ( direction == 2 )
	{		
		Cogstatesaggital[estimatecount]  =  state (1)  ;  //將state存入  cogstate matrix  
		Dcogstatesaggital[estimatecount] =  state (2)  ;
		state3saggital[estimatecount]  =  state (3)  ;
				
	}

	estimate_lock = false;    // lock 避免掉值
		 

	return EXIT_SUCCESS;

	
}







estimate :: ~ estimate(){} ;





