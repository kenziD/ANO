#ifndef __KEY_H
#define __KEY_H	 
#include "sys.h"
#define MODE_KEY  GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_1)
#define FUN_KEY  GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_9)

#define MODE_KEY_DOWN 1
#define FUN_KEY_DOWN 2
void KEY_Init(void);
u8 KEY_scan(void);
#endif
