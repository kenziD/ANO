 #include "adc.h"
 #include "delay.h"
#include "led.h"
//////////////////////////////////////////////////////////////////////////////////	 
//本程序只供学习使用，未经作者许可，不得用于其它任何用途
//ALIENTEK战舰STM32开发板
//ADC 代码	   
//正点原子@ALIENTEK
//技术论坛:www.openedv.com
//修改日期:2012/9/7
//版本：V1.0
//版权所有，盗版必究。
//Copyright(C) 广州市星翼电子科技有限公司 2009-2019
//All rights reserved									  
////////////////////////////////////////////////////////////////////////////////// 
	   
		   
//初始化ADC
//这里我们仅以规则通道为例
//我们默认将开启通道0~3			
#define ADC1_DR_Address ((u32)0x40012400+0x4c)
__IO uint16_t ADC_ConvertedValue[80];//将转换值存在ADC_ConvertedValue变量

static void ADC1_GPIO_Config(void)
{
GPIO_InitTypeDef GPIO_InitStructure;
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA |RCC_APB2Periph_ADC1, ENABLE );	  //使能ADC1通道时钟

	RCC_ADCCLKConfig(RCC_PCLK2_Div6);   //设置ADC分频因子6 72M/6=12,ADC最大时间不能超过14M

	//PA0 1 2作为模拟通道输入引脚        
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
//	DMA_InitStructure.DMA_PeripheralBaseAddr = ADC1_DR_Address;	 			//ADC地址
 DMA_InitStructure.DMA_PeripheralBaseAddr=(u32)&ADC1->DR;
	DMA_InitStructure.DMA_MemoryBaseAddr = (u32)&ADC_ConvertedValue;	//貌似要把地址符号去掉
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
	DMA_InitStructure.DMA_BufferSize = 80;                             //缓冲区的大小也要修改
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;	//外设地址固定
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;  				//内存地址固定这个要修改！
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;	//半字
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
	DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;										          //循环传输
	DMA_InitStructure.DMA_Priority = DMA_Priority_High;
	DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
	DMA_Init(DMA1_Channel1, &DMA_InitStructure);
	/* Enable DMA channel1 */
	DMA_Cmd(DMA1_Channel1, ENABLE);
	
	
	/* ADC1 configuration */	
	ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;			//独立ADC模式
	ADC_InitStructure.ADC_ScanConvMode = ENABLE ; 	 				//修改：开启扫描模式，用于多通道扫描 
	ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;			//开启连续转换模式，即不停地进行ADC转换
	ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;	//不使用外部触发转换
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right; 	//采集数据右对齐
	ADC_InitStructure.ADC_NbrOfChannel = 8;	 								//要转换的通道数目4
	ADC_Init(ADC1, &ADC_InitStructure);


	/*配置ADC时钟，为PCLK2的8分频，即9MHz*/
	RCC_ADCCLKConfig(RCC_PCLK2_Div8); 
	/*配置ADC1的通道1为55.	5个采样周期，序列为1 */ 

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
	
	/*复位校准寄存器 */   
	ADC_ResetCalibration(ADC1);
	/*等待校准寄存器复位完成 */
	while(ADC_GetResetCalibrationStatus(ADC1));
	
	/* ADC校准 */
	ADC_StartCalibration(ADC1);
	/* 等待校准完成*/
	while(ADC_GetCalibrationStatus(ADC1));
	
	/* 由于没有采用外部触发，所以使用软件触发ADC转换 */ 
	ADC_SoftwareStartConvCmd(ADC1, ENABLE);
}

void ADC1_Init(void)
{
	ADC1_GPIO_Config();
	ADC1_Mode_Config();
}

u16 voltage1(void)  //通道1的电压平均值
{  
     u8 i = 0;  
    u16 sum = 0;   
    //取十次平均值  
    for (i = 0;i < 10;i++)  
    {  
        sum += ADC_ConvertedValue[i * 8];   			
    }  
    sum /= 10;  	
    return sum;  
}  
u16 voltage2(void)  //通道2的电压平均值
{  
    u8 i = 0;  
    u16 sum = 0;    
    //取十次平均值  
    for (i = 0;i < 10;i++)  
    {   
        sum += ADC_ConvertedValue[i * 8+1];   
    }  
    sum /= 10;     
    return sum;  
}  
u16 voltage3(void)  //通道3的电压平均值
{  
     u8 i = 0;  
    u16 sum = 0;   
    //取十次平均值  
    for (i = 0;i < 10;i++)  
    {  
        sum += ADC_ConvertedValue[i * 8+2];   
    }  
    sum /= 10;      
    return sum;  
}  
 
u16 voltage4(void)  //通道4的电压平均值
{  
     u8 i = 0;  
    u16 sum = 0;     
    //取十次平均值  
    for (i = 0;i < 10;i++)  
    {  
        sum += ADC_ConvertedValue[i * 8+3];   
    }  
    sum /= 10;      
    return sum;  
}  
 
u16 voltage5(void)  //通道4的电压平均值
{  
     u8 i = 0;  
    u16 sum = 0;     
    //取十次平均值  
    for (i = 0;i < 10;i++)  
    {  
        sum += ADC_ConvertedValue[i * 8+4];   
    }  
    sum /= 10;      
    return sum;  
}
u16 voltage6(void)  //通道4的电压平均值
{  
     u8 i = 0;  
    u16 sum = 0;     
    //取十次平均值  
    for (i = 0;i < 10;i++)  
    {  
        sum += ADC_ConvertedValue[i * 8+5];   
    }  
    sum /= 10;      
    return sum;  
}
u16 voltage7(void)  //通道4的电压平均值
{  
     u8 i = 0;  
    u16 sum = 0;     
    //取十次平均值  
    for (i = 0;i < 10;i++)  
    {  
        sum += ADC_ConvertedValue[i * 8+6];   
    }  
    sum /= 10;      
    return sum;  
}
u16 voltage8(void)  //通道4的电压平均值
{  
     u8 i = 0;  
    u16 sum = 0;     
    //取十次平均值  
    for (i = 0;i < 10;i++)  
    {  
        sum += ADC_ConvertedValue[i * 8+7];   
    }  
    sum /= 10;      
    return sum;  
}


























