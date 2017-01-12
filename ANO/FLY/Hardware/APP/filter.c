#include <math.h>
#include "myStruct.h"
#include "filter.h"
#define PI 3.1415926f
float _a0,_a1,_a2,_b0,_b1,_b2;
void setCutOffFrequency(float fs,float cutOffFreq)
{
	float fr = 2*PI*cutOffFreq/fs;//归一化数字角频率
	float preDistortion = atan(fr/2);
	
	_a0 = 1 + sqrt(2) * preDistortion + preDistortion * preDistortion;
	_a1 = 2 * preDistortion * preDistortion - 2;
	_a2 = 1 - sqrt(2) * preDistortion + preDistortion * preDistortion;
	_b0 = preDistortion * preDistortion;
	_b1 = 2 * _b0;
	_b2 = _b0;
}
void ButterWorthLPF_2order(Int16xyz *acc_in,Int16xyz *acc_out)
{
	static Int16xyz lastOut;
	static Int16xyz preLastOut;
	static Int16xyz lastIn;
	static Int16xyz preLastIn;
	acc_out->x = -_a2/_a0*preLastOut.x - _a1/_a0*lastOut.x + _b2/_a0*preLastIn.x + _b1/_a0*lastIn.x + _b0/_a0*acc_in->x;
	acc_out->y = -_a2/_a0*preLastOut.y - _a1/_a0*lastOut.y + _b2/_a0*preLastIn.y + _b1/_a0*lastIn.y + _b0/_a0*acc_in->y;
	acc_out->z = -_a2/_a0*preLastOut.z - _a1/_a0*lastOut.z + _b2/_a0*preLastIn.z + _b1/_a0*lastIn.z + _b0/_a0*acc_in->z;
	preLastOut = lastOut;
	lastOut = *acc_out;
	preLastIn = lastIn;
	lastIn = *acc_in;
}
float inv(float input)
{
	return 1.0/input;
}
void kalManFilter(Int16xyz *acc_in,Int16xyz *acc_out)
{
	int F=1;
	int H=1;
	int Q = 0;
	int R=1;
	
	static floatxyz P = {1,1,1};

	static floatxyz X={1000,1000,1000};
	static floatxyz preX;
	static floatxyz preP;
	static floatxyz K;
	static floatxyz e;
	
	preX.x = X.x;
	
	preP.x=P.x+Q;
	K.x=preP.x*1.0/(preP.x+R);
	e.x=acc_in->x-preX.x;  
	P.x=(1-K.x)*preP.x; 
	
	X.x=preX.x+K.x*e.x;
	
	acc_out->x = X.x;
	////////////////////////////////
//	preX.y = X.y;
//	
//	preP.y=P.y+Q;
//	K.y=preP.y*inv(preP.y+R);
//	e.y=acc_in->y-preX.y;  
//	P.y=(1-K.y)*preP.y; 
//	
//	X.y=preX.y+K.y*e.y;
//	
//	acc_out->y = X.y;
acc_out->y = 0;
	////////////////////////////////
//	preX.z = X.z;
//	
//	preP.z=P.z+Q;
//	K.z=preP.z*inv(preP.z+R);
//	e.z=acc_in->z-preX.z;  
//	P.z=(1-K.z)*preP.z; 
//	
//	X.z=preX.z+K.z*e.z;
//	
//	acc_out->z = X.z;
acc_out->z = 0;
}
void kalManFilterAx(int16_t u,int16_t *out)
{
	int F=1;
	int H=1;
	int Q = 0;
	static float P = 1;
	int R=11;
	static float X = 1;
	static float preX;
	static float preP;
	static float K;
	static float e;
	
	preX = F*X;
	
	preP=F*P*inv(F)+Q;
	K=preP*inv(H)*inv(H*preP*inv(H)+R);
	e=u-H*preX;  
	P=(1-K*H)*preP; 
	
	X=preX+K*e;
	
	*out = X;
}
void kalManFilterAy(int16_t u,int16_t *out)
{
	int F=1;
	int H=1;
	int Q = 0;
	static float P = 1;
	int R=11;
	static float X = 1;
	static float preX;
	static float preP;
	static float K;
	static float e;
	
	preX = F*X;
	
	preP=F*P*inv(F)+Q;
	K=preP*inv(H)*inv(H*preP*inv(H)+R);
	e=u-H*preX;  
	P=(1-K*H)*preP; 
	
	X=preX+K*e;
	
	*out = X;
}
void kalManFilterAz(int16_t u,int16_t *out)
{
	int F=1;
	int H=1;
	int Q = 0;
	static float P = 1;
	int R=11;
	static float X = 1;
	static float preX;
	static float preP;
	static float K;
	static float e;
	
	preX = F*X;
	
	preP=F*P*inv(F)+Q;
	K=preP*inv(H)*inv(H*preP*inv(H)+R);
	e=u-H*preX;  
	P=(1-K*H)*preP; 
	
	X=preX+K*e;
	
	*out = X;
}
