#ifndef __RC_H
#define __RC_H

#include "stm32f10x.h"
#include "string.h"
typedef struct 
{
	int throttle;
	float roll;
	float pitch;
	float yaw;
	char status[5];
} Define_Rc_Data ;


void Rc_Data_Analyze(u8 *rcDataBuf,Define_Rc_Data *rc_data);
#endif
