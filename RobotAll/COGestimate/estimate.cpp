#include "stdafx.h"
#include "estimate.h"
#include <vector>
#include <iomanip>//����cout ��X�p���I���

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
	
	const unsigned NTRY = 1200*10; //data��
	
	const unsigned n = 7;	//nb states
	const unsigned m = 7;	//nb measures
	
	QuaternionTrans QuaternionTrans1;

	double  timestep  = 0.005 ; 

	double Rotation[3][3];		// Rotation matrix (Quaternion�Φ�)
	
	double cogaccelationRoll = cogaccely, cogaccelationPitch = cogaccelx, cogaccelationYaw = -cogaccelz;  // �Ȧs��

	
	Vector x(n);   // �}state vector 
	Vector rstate(n) ;
	Vector z(m);

	EulerAngle ea;		// �Ȧs�B��Τ�EulerAngle
	
	ea.fRoll = -anglex;
	ea.fPitch = -angley;
	ea.fYaw = anglez;

	QuaternionTrans1.FromEulerAngle(ea);	// �NEulerAngle�ରQuaternion���x�s

	//��l��
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

		Matrix P0(n, n, _P0); //�إ� initial error  matrix  
		rfilter.init(x, P0);

		rinitflag = true;
	}
	

	//******����filter

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


	// �ץ��[�t�צܥ@�ɮy�жb	
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

	// �N�gEKF�᪺Quaternion��^EulerAngle
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
	
	

	const unsigned NTRY = 1200*10; //data��
	

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
	
	double  Dzmp = 0   , Dzmpbefore = 0 ; //�e�Ӯɨ誺zmp �@���L���q 
	double  DDzmp = 0  , DDzmpbefore = 0 ; //�e�Ӯɨ誺zmp �G���L���q 


	double IMU_COG  = 666/732;

	double exp_gain = 1 ;    //cm/s^2  to mm/s^2
		
	Vector x(n);   // �}state vector 
	
	Vector state(n) ;
	
	Vector z(m);
	
	
	//zmp'' �ƭȷL���B�z
	
	
	estimate_lock = true;  // lock �קK����



	if ( gFlagSimulation == 2 )
	   exp_gain = 100; 
	
		
	
	//P0(0,0) = 0 ;

	// ��l��
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
		Matrix P0(n, n, _P0); //�إ� initial error  matrix
		filter.init(x, P0);

		initflag = true;
	}

	
	// ��Jsensor����
	if ( filter.ZMPflag == 0 )
	{
		z(1) = cog;  
		z(2) = cogaccel* IMU_COG *exp_gain; //�Nmeasurement �g�J IMU�w�˦�m����Ҥ��t
	}
	else
	{
		z(1) = cog;  
		z(2) = *zmp;  
		z(3) = cogaccel* IMU_COG *exp_gain;  //�Nmeasurement �g�J IMU�w�˦�m����Ҥ��t
	}
	    
		
	   
	// u  control input    zmp�G���L��
		
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
		Cogstatelateral[estimatecount] =  state (1)  ;  //�Nstate�s�J  cogstate matrix  
		Dcogstatelateral[estimatecount] = state (2)  ;
		state3lateral[estimatecount] =  state (3)  ;
	}
	else if ( direction == 2 )
	{		
		Cogstatesaggital[estimatecount]  =  state (1)  ;  //�Nstate�s�J  cogstate matrix  
		Dcogstatesaggital[estimatecount] =  state (2)  ;
		state3saggital[estimatecount]  =  state (3)  ;
				
	}

	estimate_lock = false;    // lock �קK����
		 

	return EXIT_SUCCESS;

	
}







estimate :: ~ estimate(){} ;





