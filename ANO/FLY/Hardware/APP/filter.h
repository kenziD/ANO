#ifndef __FILTER_H_
#define	__FILTER_H_

#include "stm32f10x.h"
#include "myStruct.h"
void setCutOffFrequency(float fs,float cutOffFreq);
void ButterWorthLPF_2order(Int16xyz *acc_in,Int16xyz *acc_out);

void kalManFilterAx(int16_t u,int16_t *out);
void kalManFilterAy(int16_t u,int16_t *out);
void kalManFilterAz(int16_t u,int16_t *out);


void kalManFilter(Int16xyz *acc_in,Int16xyz *acc_out);
#endif
