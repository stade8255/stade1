#pragma once

#include "EulerAngle.hpp"

class QuaternionTrans
{
public:
	double x , y , z , w;

	QuaternionTrans() : x(0.0f) , y(0.0f) , z(0.0f) , w(1.0f) {}
	
	
	void  FromEulerAngle(const EulerAngle &ea) ;
	 
	EulerAngle ToEulerAngle() const;
	
	QuaternionTrans Multiply(const QuaternionTrans &b);
    
	void Normalize();

	double QuaternionTrans::getS();


	~QuaternionTrans() {}
};









