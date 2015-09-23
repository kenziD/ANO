#ifndef __MOTOR_H
#define __MOTOR_H
#include "config.h"

void TIM2_Init(u16 arr,u16 psc);
void MOT0_GPIO_init();
void MOT0_PWM_init();
void MOT1_GPIO_init();
void MOT1_PWM_init();
void MOT2_GPIO_init();
void MOT2_PWM_init();
void MOT3_GPIO_init();
void MOT3_PWM_init();
void MOT_GPIO_init();
void MOT_PWM_init();
#endif
