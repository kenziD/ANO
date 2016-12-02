
#include "adc.h"

#define ADC1_DR_Address    ((u32)0x4001244C)

__IO u16 ADC_ConvertedValue;

static void ADC1_GPIO_Config(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;

  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1 | RCC_APB2Periph_GPIOB, ENABLE);

  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
  GPIO_Init(GPIOB, &GPIO_InitStructure);				
}

static void ADC1_Mode_Config(void)
{
	DMA_InitTypeDef DMA_InitStructure;
	ADC_InitTypeDef ADC_InitStructure;

	/* DMA channel1 configuration */
  DMA_DeInit(DMA1_Channel1);
  DMA_InitStructure.DMA_PeripheralBaseAddr = ADC1_DR_Address;
  DMA_InitStructure.DMA_MemoryBaseAddr = (u32)&ADC_ConvertedValue;
  DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
  DMA_InitStructure.DMA_BufferSize = 1;
  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Disable;
  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
  DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
  DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
  DMA_InitStructure.DMA_Priority = DMA_Priority_High;
  DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
  DMA_Init(DMA1_Channel1, &DMA_InitStructure);
  
  /* Enable DMA channel1 */
  DMA_Cmd(DMA1_Channel1, ENABLE);
     
  /* ADC1 configuration */
  ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;	//¶ÀÁ¢Ä£Ê½ Ã¿¸öADC¶ÀÁ¢¹¤×÷
  ADC_InitStructure.ADC_ScanConvMode = ENABLE;				 //Ê¹ÓÃÉ¨ÃèÄ£Ê½  scanÎ»ÉèÖÃ
  ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;	// contÎ»ÉèÖÃ Á¬Ðø×ª»»Ä£Ê½
  ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;	;//EXTSEL Ñ¡ÔñÆô¶¯¹æÔòÍ¨µÀ×é×ª»»µÄÍâ²¿ÊÂ¼þ ÉèÖÃ³ÉÓÐÈí¼þ¿ØÖÆ
  ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;	//Êý¾Ý¶ÔÆë ÓÉÈí¼þÖÃÎ»ºÍÇå³þ   ÕâÀïÉèÖÃ³ÉÓÒ¶ÔÆë
  ADC_InitStructure.ADC_NbrOfChannel = 1;		//¹æÔòÍ¨µÀÐòÁÐ³¤¶È ÕâÐ©Î»ÓÉÈí¼þ¶¨ÒåÔÚ¹æÔòÍ¨µÀ×ª»»ÐòÁÐÖÐµÄÍ¨µÀÊýÄ¿ 1¸ö×ª»» Ö¸¶¨ÓÉ¶àÉÙ¸öÍ¨µÀ±»×ª»»
  ADC_Init(ADC1, &ADC_InitStructure);

  /* ADC1 regular channel11 configuration */ 
  ADC_RegularChannelConfig(ADC1, ADC_Channel_9, 1, ADC_SampleTime_55Cycles5);	//×ª»»Ê±¼äÊÇ55.5¸öÖÜÆÚ

  /* Enable ADC1 DMA */
  ADC_DMACmd(ADC1, ENABLE);
  
  /* Enable ADC1 */
  ADC_Cmd(ADC1, ENABLE);

  /* Enable ADC1 reset calibaration register */   
  ADC_ResetCalibration(ADC1);
  /* Check the end of ADC1 reset calibration register */
  while(ADC_GetResetCalibrationStatus(ADC1));

  /* Start ADC1 calibaration */
  ADC_StartCalibration(ADC1);
  /* Check the end of ADC1 calibration */
  while(ADC_GetCalibrationStatus(ADC1));
     
  /* Start ADC1 Software Conversion */ 
  ADC_SoftwareStartConvCmd(ADC1, ENABLE);
}


void ADC1_Init(void)
{
	ADC1_GPIO_Config();
	ADC1_Mode_Config();
}
