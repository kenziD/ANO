#ifndef __LED_H
#define __LED_H

#include "stm32f10x.h"

#define LED1_OFF        GPIOB->BSRR = GPIO_Pin_11 /* GPIO_SetBits(GPIOB , GPIO_Pin_11)   */
#define LED1_ON         GPIOB->BRR  = GPIO_Pin_11 /* GPIO_ResetBits(GPIOB , GPIO_Pin_11) */
#define LED2_OFF        GPIOB->BSRR = GPIO_Pin_12 /* GPIO_SetBits(GPIOB , GPIO_Pin_12)   */
#define LED2_ON         GPIOB->BRR  = GPIO_Pin_12 /* GPIO_ResetBits(GPIOB , GPIO_Pin_12) */
void LED_Init(void);
void LED1_Flash(u8 times, u32 time);
void LED2_Flash(u8 times, u32 time);
#endif /* __LED_H */

