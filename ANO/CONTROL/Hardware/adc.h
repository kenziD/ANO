#ifndef __ADC_H
#define __ADC_H	
#include "sys.h"
#include "stm32f10x.h"
//������ֻ��ѧϰʹ�ã�δ��������ɣ��������������κ���;
//ALIENTEKս��STM32������
//ADC ����	   
//����ԭ��@ALIENTEK
//������̳:www.openedv.com
//�޸�����:2012/9/7
//�汾��V1.0
//��Ȩ���У�����ؾ���
//Copyright(C) ������������ӿƼ����޹�˾ 2009-2019
//All rights reserved									  
////////////////////////////////////////////////////////////////////////////////// 

void ADC1_Init(void);
u16 voltage1(void);
u16 voltage2(void);
u16 voltage3(void);
u16 voltage4(void);
#endif 
