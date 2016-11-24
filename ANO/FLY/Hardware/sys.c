#include "sys.h"
void NVIC_Configuration(void)
{   
	NVIC_InitTypeDef NVIC_InitStructure;
	
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	//TIM3
	NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn; //TIM3 �ж�
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0; //��ռ���ȼ�0(����ԽС���ȼ�Խ��)
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1; //��Ӧ���ȼ�1
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; //IRQ ͨ����ʹ��
	NVIC_Init(&NVIC_InitStructure);  //��ʼ��NVIC�Ĵ���
	
	//USART
	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3 ; //��ռ���ȼ�3
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1; //��Ӧ���ȼ�1
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; //IRQ ͨ����ʹ��
	NVIC_Init(&NVIC_InitStructure); //��ʼ��NVIC�Ĵ���

}
