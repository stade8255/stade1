#include "stdafx.h"
#include "QuaternionTrans.hpp"
#include <math.h>


#define CLAMP(x , min , max) ((x) > (max) ? (max) : ((x) < (min) ? (min) : x))

//核心算法1，欧拉角转四元数
void  QuaternionTrans  :: FromEulerAngle(const EulerAngle &ea) 
 {
         double fCosHRoll = cos(ea.fRoll * .5f);
         double fSinHRoll = sin(ea.fRoll * .5f);
         double fCosHPitch = cos(ea.fPitch * .5f);
         double fSinHPitch = sin(ea.fPitch * .5f);
         double fCosHYaw = cos(ea.fYaw * .5f);
         double fSinHYaw = sin(ea.fYaw * .5f);
         w = fCosHRoll * fCosHPitch * fCosHYaw + fSinHRoll * fSinHPitch * fSinHYaw;
         x = fSinHRoll * fCosHPitch * fCosHYaw - fCosHRoll * fSinHPitch * fSinHYaw;
         y = fCosHRoll * fSinHPitch * fCosHYaw + fSinHRoll * fCosHPitch * fSinHYaw;
         z = fCosHRoll * fCosHPitch * fSinHYaw - fSinHRoll * fSinHPitch * fCosHYaw;
 }
 
//核心算法2，四元数转欧拉角
EulerAngle QuaternionTrans::ToEulerAngle() const  //將quaternion 轉成eulerangle 
 {
         EulerAngle ea;
         ea.fRoll  = atan2(2 * (y * z + x * w) , 1 - 2 * (x * x + y * y));
         ea.fPitch = asin(CLAMP(-2 * (x * z - w * y) , -1.0f , 1.0f));
         ea.fYaw   = atan2(2 * (x * y + w * z) , 1 - 2 * (y * y + z * z));
         return ea;
 }
 
 
 
//核心算法3，四元数乘法
QuaternionTrans QuaternionTrans::Multiply(const QuaternionTrans &b)
 {
         QuaternionTrans c;
         c.w=w*b.w        -x*b.x        -y*b.y        -z*b.z;
         c.x=w*b.x        +x*b.w        +y*b.z        -z*b.y;
         c.y=w*b.y        -x*b.z        +y*b.w        +z*b.x;
         c.z=w*b.z        +x*b.y        -y*b.x        +z*b.w;
         c.Normalize();
         return c;
 }
 
//次要的规范化算法：
void  QuaternionTrans::Normalize(){
         double s=getS();
         w/=s;
         x/=s;
         y/=s;
         z/=s;
 }



 double QuaternionTrans::getS(){
         return sqrt(w*w+x*x+y*y+z*z);
 }
 






   void calculate (double *angx,double *angy,double *angz) {
 	
  int intx, inty,intz;
  double  pitch,roll,yaw; 
  int i = 0  ; 
  


  EulerAngle dt;
  QuaternionTrans dQ;
  EulerAngle nowG;
  QuaternionTrans nowQ;
  
 
  dt.fRoll   = 10**angx; 
  dt.fPitch  = 10**angy ;
  dt.fYaw    = 10**angz; 



  dQ.FromEulerAngle(dt);
  
  nowQ= nowQ.Multiply(dQ);
  
  nowG=  nowQ.ToEulerAngle();  //存下轉換成的角度值

	
	
	

  
	 //datacount ++;
	




    
  }