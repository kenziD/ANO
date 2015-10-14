#ifndef __LED_H
#define __LED_H

#include "stm32f10x.h"

#define LED_OFF        GPIOB->BSRR = GPIO_Pin_5 /* GPIO_SetBits(GPIOB , GPIO_Pin_5)   */
#define LED_ON         GPIOB->BRR  = GPIO_Pin_5 /* GPIO_ResetBits(GPIOB , GPIO_Pin_5) */

void LED_Init(void);
void LED_Flash(u8 times, u32 time);

#endif /* __LED_H */

