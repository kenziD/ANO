 #include "adc.h"
 #include "delay.h"
#include "led.h"
//////////////////////////////////////////////////////////////////////////////////	 
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
	   
		   
//��ʼ��ADC
//�������ǽ��Թ���ͨ��Ϊ��
//����Ĭ�Ͻ�����ͨ��0~3			
#define ADC1_DR_Address ((u32)0x40012400+0x4c)
__IO uint16_t ADC_ConvertedValue[80];//��ת��ֵ����ADC_ConvertedValue����

static void ADC1_GPIO_Config(void)
{
GPIO_InitTypeDef GPIO_InitStructure;
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA |RCC_APB2Periph_ADC1, ENABLE );	  //ʹ��ADC1ͨ��ʱ��

	RCC_ADCCLKConfig(RCC_PCLK2_Div6);   //����ADC��Ƶ����6 72M/6=12,ADC���ʱ�䲻�ܳ���14M

	//PA0 1 2��Ϊģ��ͨ����������        
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0| GPIO_Pin_1|GPIO_Pin_2|GPIO_Pin_3|GPIO_Pin_4| GPIO_Pin_5|GPIO_Pin_6|GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN; 
	GPIO_Init(GPIOA, &GPIO_InitStructure);	
}
			  
static void ADC1_Mode_Config(void)
{
	DMA_InitTypeDef DMA_InitStructure;
	ADC_InitTypeDef ADC_InitStructure;
	/* DMA channel1 configuration */
	DMA_DeInit(DMA1_Channel1);
//	DMA_InitStructure.DMA_PeripheralBaseAddr = ADC1_DR_Address;	 			//ADC��ַ
 DMA_InitStructure.DMA_PeripheralBaseAddr=(u32)&ADC1->DR;
	DMA_InitStructure.DMA_MemoryBaseAddr = (u32)&ADC_ConvertedValue;	//ò��Ҫ�ѵ�ַ����ȥ��
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
	DMA_InitStructure.DMA_BufferSize = 80;                             //�������Ĵ�СҲҪ�޸�
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;	//�����ַ�̶�
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;  				//�ڴ��ַ�̶����Ҫ�޸ģ�
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;	//����
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
	DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;										          //ѭ������
	DMA_InitStructure.DMA_Priority = DMA_Priority_High;
	DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
	DMA_Init(DMA1_Channel1, &DMA_InitStructure);
	/* Enable DMA channel1 */
	DMA_Cmd(DMA1_Channel1, ENABLE);
	
	
	/* ADC1 configuration */	
	ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;			//����ADCģʽ
	ADC_InitStructure.ADC_ScanConvMode = ENABLE ; 	 				//�޸ģ�����ɨ��ģʽ�����ڶ�ͨ��ɨ�� 
	ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;			//��������ת��ģʽ������ͣ�ؽ���ADCת��
	ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;	//��ʹ���ⲿ����ת��
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right; 	//�ɼ������Ҷ���
	ADC_InitStructure.ADC_NbrOfChannel = 8;	 								//Ҫת����ͨ����Ŀ4
	ADC_Init(ADC1, &ADC_InitStructure);


	/*����ADCʱ�ӣ�ΪPCLK2��8��Ƶ����9MHz*/
	RCC_ADCCLKConfig(RCC_PCLK2_Div8); 
	/*����ADC1��ͨ��1Ϊ55.	5���������ڣ�����Ϊ1 */ 

	ADC_RegularChannelConfig(ADC1, ADC_Channel_4, 1, ADC_SampleTime_55Cycles5);
	ADC_RegularChannelConfig(ADC1, ADC_Channel_5, 2, ADC_SampleTime_55Cycles5);
	ADC_RegularChannelConfig(ADC1, ADC_Channel_6, 3, ADC_SampleTime_55Cycles5);
	ADC_RegularChannelConfig(ADC1, ADC_Channel_7, 4, ADC_SampleTime_55Cycles5);
	ADC_RegularChannelConfig(ADC1, ADC_Channel_0, 5, ADC_SampleTime_55Cycles5);
	ADC_RegularChannelConfig(ADC1, ADC_Channel_1, 6, ADC_SampleTime_55Cycles5);
	ADC_RegularChannelConfig(ADC1, ADC_Channel_2, 7, ADC_SampleTime_55Cycles5);
	ADC_RegularChannelConfig(ADC1, ADC_Channel_3, 8, ADC_SampleTime_55Cycles5);
	/* Enable ADC1 DMA */
	ADC_DMACmd(ADC1, ENABLE);
	
	/* Enable ADC1 */
	ADC_Cmd(ADC1, ENABLE);
	
	/*��λУ׼�Ĵ��� */   
	ADC_ResetCalibration(ADC1);
	/*�ȴ�У׼�Ĵ�����λ��� */
	while(ADC_GetResetCalibrationStatus(ADC1));
	
	/* ADCУ׼ */
	ADC_StartCalibration(ADC1);
	/* �ȴ�У׼���*/
	while(ADC_GetCalibrationStatus(ADC1));
	
	/* ����û�в����ⲿ����������ʹ���������ADCת�� */ 
	ADC_SoftwareStartConvCmd(ADC1, ENABLE);
}

void ADC1_Init(void)
{
	ADC1_GPIO_Config();
	ADC1_Mode_Config();
}

u16 voltage1(void)  //ͨ��1�ĵ�ѹƽ��ֵ
{  
     u8 i = 0;  
    u16 sum = 0;   
    //ȡʮ��ƽ��ֵ  
    for (i = 0;i < 10;i++)  
    {  
        sum += ADC_ConvertedValue[i * 8];   			
    }  
    sum /= 10;  	
    return sum;  
}  
u16 voltage2(void)  //ͨ��2�ĵ�ѹƽ��ֵ
{  
    u8 i = 0;  
    u16 sum = 0;    
    //ȡʮ��ƽ��ֵ  
    for (i = 0;i < 10;i++)  
    {   
        sum += ADC_ConvertedValue[i * 8+1];   
    }  
    sum /= 10;     
    return sum;  
}  
u16 voltage3(void)  //ͨ��3�ĵ�ѹƽ��ֵ
{  
     u8 i = 0;  
    u16 sum = 0;   
    //ȡʮ��ƽ��ֵ  
    for (i = 0;i < 10;i++)  
    {  
        sum += ADC_ConvertedValue[i * 8+2];   
    }  
    sum /= 10;      
    return sum;  
}  
 
u16 voltage4(void)  //ͨ��4�ĵ�ѹƽ��ֵ
{  
     u8 i = 0;  
    u16 sum = 0;     
    //ȡʮ��ƽ��ֵ  
    for (i = 0;i < 10;i++)  
    {  
        sum += ADC_ConvertedValue[i * 8+3];   
    }  
    sum /= 10;      
    return sum;  
}  
 
u16 voltage5(void)  //ͨ��4�ĵ�ѹƽ��ֵ
{  
     u8 i = 0;  
    u16 sum = 0;     
    //ȡʮ��ƽ��ֵ  
    for (i = 0;i < 10;i++)  
    {  
        sum += ADC_ConvertedValue[i * 8+4];   
    }  
    sum /= 10;      
    return sum;  
}
u16 voltage6(void)  //ͨ��4�ĵ�ѹƽ��ֵ
{  
     u8 i = 0;  
    u16 sum = 0;     
    //ȡʮ��ƽ��ֵ  
    for (i = 0;i < 10;i++)  
    {  
        sum += ADC_ConvertedValue[i * 8+5];   
    }  
    sum /= 10;      
    return sum;  
}
u16 voltage7(void)  //ͨ��4�ĵ�ѹƽ��ֵ
{  
     u8 i = 0;  
    u16 sum = 0;     
    //ȡʮ��ƽ��ֵ  
    for (i = 0;i < 10;i++)  
    {  
        sum += ADC_ConvertedValue[i * 8+6];   
    }  
    sum /= 10;      
    return sum;  
}
u16 voltage8(void)  //ͨ��4�ĵ�ѹƽ��ֵ
{  
     u8 i = 0;  
    u16 sum = 0;     
    //ȡʮ��ƽ��ֵ  
    for (i = 0;i < 10;i++)  
    {  
        sum += ADC_ConvertedValue[i * 8+7];   
    }  
    sum /= 10;      
    return sum;  
}


























