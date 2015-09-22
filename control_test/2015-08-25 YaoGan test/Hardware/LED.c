#include "config.h"

void LED_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB , ENABLE); 
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11|GPIO_Pin_12;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    //LED_Flash(5, 100);
}

//void LED_Flash(u8 times, u32 time)
//{
//    while(times--)
//    {
//        LED_ON;
//        delay_us(time);
//        LED_OFF;
//        delay_us(time);
//    }
//    LED_ON;
//}
