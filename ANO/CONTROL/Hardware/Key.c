#include "Key.h"
#include "delay.h"
#include "usart.h"
void KEY_Init(void) //IO初始化
{ 
 	GPIO_InitTypeDef GPIO_InitStructure;
	//初始化MODE-->GPIOB.1,FUN-->GPIOB.9  上拉输入
 	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE);//使能PORTB时钟

	GPIO_InitStructure.GPIO_Pin  = GPIO_Pin_1;//PB1
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU; //设置成上拉输入
	GPIO_Init(GPIOB, &GPIO_InitStructure);//初始化GPIOB

	GPIO_InitStructure.GPIO_Pin  = GPIO_Pin_9;//PB1
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU; //设置成上拉输入
	GPIO_Init(GPIOB, &GPIO_InitStructure);//初始化GPIOB

}

u8 KEY_scan(void)
{
	if(MODE_KEY==0||FUN_KEY==0)
	{
		delay_ms(100);
		if(MODE_KEY==0)
			return MODE_KEY_DOWN;
		if(FUN_KEY==0)
			return FUN_KEY_DOWN;
	}
	return 0;
}
