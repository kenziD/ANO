#ifndef __FILTER_H_
#define	__FILTER_H_

#include "stm32f10x.h"
#include "myStruct.h"
void setCutOffFrequency(float fs,float cutOffFreq);
void ButterWorthLPF_2order(Int16xyz *acc_in,Int16xyz *acc_out);
void setCutOffFrequency_HYSRF05(float fs,float cutOffFreq);
void ButterWorthLPF_2order_HYSRF05(float height_in,float *height_out);
#endif
