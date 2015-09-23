#include "HSEconf.h"

void RCC_HSE_Configuration(void)
{
			RCC_DeInit();//将外设RCC寄存器重设为缺省值   
			RCC_HSEConfig(RCC_HSE_ON);
			//设置外部高速晶振(HSE)  HSE晶振打开(ON)   
			if(RCC_WaitForHSEStartUp()==SUCCESS)//等待HSE起振，SUCCESS;HSE晶振稳定且就绪   
				{RCC_HCLKConfig(RCC_SYSCLK_Div1);//设置AHB时钟(HCLK)  RCC_SYSCLK_Div1 ----- AHB时钟 = 系统时钟    
					RCC_PCLK2Config(RCC_HCLK_Div1);//设置高速AHB时钟(PCLK2)  RCC_HCLK_Div1 ----- APB2时钟 = HCLK    
					RCC_PCLK1Config(RCC_HCLK_Div2);//设置低速AHB时钟(PCLK1)  RCC_HCLK_Div2 ----- APB1时钟 = HCLK/2        
					 /* Enable Prefetch Buffer */
					FLASH->ACR |= FLASH_ACR_PRFTBE;
					FLASH->ACR &= (uint32_t)((uint32_t)~FLASH_ACR_LATENCY);
					FLASH->ACR |= (uint32_t)FLASH_ACR_LATENCY_2;    //设置FLASH存储器延时时钟周期数FLASH_Latency_2   2延时周期    
					//FLASH_PrefetchBufferCmd(FLASH_PrefetchBuffer_Enable);//选择FLASH预取指缓存的模,预取指缓存使能        
					RCC_PLLConfig(RCC_PLLSource_HSE_Div2,RCC_PLLMul_9);//设置PLL时钟源及倍频系RCC_PLLConfig(外部时钟分频因子,PLL倍频系数)    
					RCC_PLLCmd(ENABLE);//使能PLL    
					while(RCC_GetFlagStatus(RCC_FLAG_PLLRDY)==RESET);//检查指定的RCC标志位(PLL准备好标志)设置与否        
					RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);//设置系统时钟(SYSCLK)    
					while(RCC_GetSYSCLKSource()!=0x08);//0x08:PLL作为系统时钟   
	}
}
