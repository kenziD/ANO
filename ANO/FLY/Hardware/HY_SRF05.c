#include "HY_SRF05.h"

void HYSRF05_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE);	 //ʹ��PB�˿�ʱ��
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;	
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(GPIOB, &GPIO_InitStructure);//��ʼ��Trig
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;	
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;
	GPIO_Init(GPIOB, &GPIO_InitStructure);//��ʼ��ECHO
	
}
TIM_ICInitTypeDef  TIM4_ICInitStructure;
void Tim4_Init()
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	
	TIM_DeInit(TIM4);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4,ENABLE);
	TIM_TimeBaseStructure.TIM_Period=0XFFFF;
	//prescaler is 1200,that is 72000000/72/49999=20Hz;full ARR is 0.05s. Enough to test Ultrasound:if test distance is 3m,need 0.0176s,49999ARR is enough to represent.
	TIM_TimeBaseStructure.TIM_Prescaler=72-1;
	//set clock division 
	TIM_TimeBaseStructure.TIM_ClockDivision=TIM_CKD_DIV1; //or TIM_CKD_DIV2 or TIM_CKD_DIV4
	//count up
	TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up;
	
	TIM_TimeBaseInit(TIM4,&TIM_TimeBaseStructure);
	
	TIM4_ICInitStructure.TIM_Channel = TIM_Channel_4; //CC1S=01 	ѡ������� IC1ӳ�䵽TI1��
	TIM4_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;	//�����ز���
	TIM4_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI; //ӳ�䵽TI1��
	TIM4_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;	 //���������Ƶ,����Ƶ 
	TIM4_ICInitStructure.TIM_ICFilter = 0x00;//IC1F=0000 ���������˲��� ���˲�
	TIM_ICInit(TIM4, &TIM4_ICInitStructure);
	
	//clear the TIM2 overflow interrupt flag.repeat with interrupt function.
	//TIM_ClearFlag(TIM3,TIM_FLAG_Update);
	
	//TIM2 overflow interrupt enable
	TIM_ITConfig(TIM4,TIM_IT_Update|TIM_IT_CC4,ENABLE);//��������ж� ,����CC1IE�����ж�	

	//enable TIM3
	//TIM_Cmd(TIM4,ENABLE);
	TIM_Cmd(TIM4, ENABLE);
}

void Ultrasound_Start(void)
{
	Trig_ON;
	delay_us(20);//delay 10us to start eject Ultrasound
	Trig_OFF;
}

