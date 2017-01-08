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
