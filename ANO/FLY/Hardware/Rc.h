#ifndef __RC_H
#define __RC_H

#include "stm32f10x.h"
#include "string.h"
typedef struct 
{
	int throttle;
	uint16_t roll;
	uint16_t pitch;
	uint16_t yaw;
	u8 start;
	u16 aux1;
	u16 aux2;
	u16 aux3;
} Define_Rc_Data ;


void Rc_Data_Analyze(u8 *rcDataBuf,Define_Rc_Data *rc_data);
#endif
