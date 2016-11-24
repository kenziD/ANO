#include "sys.h"
void NVIC_Configuration(void)
{   
	NVIC_InitTypeDef NVIC_InitStructure;
	
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	//TIM3
	NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn; //TIM3 中断
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0; //抢占优先级0(数字越小优先级越高)
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1; //响应优先级1
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; //IRQ 通道被使能
	NVIC_Init(&NVIC_InitStructure);  //初始化NVIC寄存器
	
	//USART
	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3 ; //抢占优先级3
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1; //响应优先级1
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; //IRQ 通道被使能
	NVIC_Init(&NVIC_InitStructure); //初始化NVIC寄存器

}
