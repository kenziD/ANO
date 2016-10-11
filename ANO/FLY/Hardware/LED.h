#ifndef __LED_H
#define __LED_H

#include "stm32f10x.h"

#define LED2_OFF  		GPIO_SetBits(GPIOB, GPIO_Pin_3);
#define LED2_ON 			GPIO_ResetBits(GPIOB, GPIO_Pin_3);

#define LED3_ON  		GPIO_SetBits(GPIOC, GPIO_Pin_13);
#define LED3_OFF 			GPIO_ResetBits(GPIOC, GPIO_Pin_13);
void LED_Init(void);
void LED_Flash(u8 times, u32 time);

#endif /* __LED_H */

