#include "Key.h"
#include "delay.h"
#include "usart.h"
void KEY_Init(void) //IO��ʼ��
{ 
 	GPIO_InitTypeDef GPIO_InitStructure;
	//��ʼ��MODE-->GPIOB.1,FUN-->GPIOB.9  ��������
 	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE);//ʹ��PORTBʱ��

	GPIO_InitStructure.GPIO_Pin  = GPIO_Pin_1;//PB1
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU; //���ó���������
	GPIO_Init(GPIOB, &GPIO_InitStructure);//��ʼ��GPIOB

	GPIO_InitStructure.GPIO_Pin  = GPIO_Pin_9;//PB1
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU; //���ó���������
	GPIO_Init(GPIOB, &GPIO_InitStructure);//��ʼ��GPIOB

}

u8 KEY_scan(void)
{
	static u8 key_up = 0;
	//if(key_up && (MODE_KEY==0||FUN_KEY==0))
	if(MODE_KEY==0||FUN_KEY==0)
	{
		//key_up=0;
		delay_ms(100);
		//printf("Mode_key:%d",MODE_KEY);
		if(MODE_KEY==0)
			return 1;
		if(FUN_KEY==0)
			return 2;
	}
	return 0;
}
