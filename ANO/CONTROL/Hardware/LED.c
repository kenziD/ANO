#include "config.h"

void LED_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB , ENABLE); 
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11|GPIO_Pin_12;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	
    GPIO_Init(GPIOB, &GPIO_InitStructure);
		GPIO_SetBits(GPIOB , GPIO_Pin_11);
	GPIO_SetBits(GPIOB , GPIO_Pin_12);
    //LED_Flash(5, 100);
}

void LED1_Flash(u8 times, u32 time)
{
    while(times--)
    {
        LED1_ON;
        delay_us(time);
        LED1_OFF;
        delay_us(time);
    }
    LED1_ON;
}

void LED2_Flash(u8 times, u32 time)
{
    while(times--)
    {
        LED2_ON;
        delay_us(time);
        LED2_OFF;
        delay_us(time);
    }
    LED2_ON;
}
