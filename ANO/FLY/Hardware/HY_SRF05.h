#ifndef __HYSRF05_H
#define __HYSRF05_H
#include "sys.h"

#define Trig_ON  		GPIO_SetBits(GPIOB, GPIO_Pin_8);
#define Trig_OFF 			GPIO_ResetBits(GPIOB, GPIO_Pin_8);
void Tim4_Init(void);
void HYSRF05_Init(void);
void Ultrasound_Start(void);
float getHeight(float *measureHeight);
#endif

