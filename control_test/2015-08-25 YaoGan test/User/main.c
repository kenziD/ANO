#include "config.h"
#include "stdint.h"

int main(void)
{
	float v0=0,v1=0,v2=0,v3=0;

	SysTick_Init();

	LED_Init();

	ADC1_Init();
	USART1_Config(9600);
		
	while(1)
  {
		v0=(float)voltage1()*(3.3/4096);		 //得到X-通道ADC电压值
		printf("v0=%f\t", v0);

		v1=(float)voltage2()*(3.3/4096);		 //得到X-通道ADC电压值
		printf("v1=%f\t", v1);

		v2=(float)voltage3()*(3.3/4096);		 //得到X-通道ADC电压值
		printf("v2=%f\t", v2);

		v3=(float)voltage4()*(3.3/4096);		 //得到X-通道ADC电压值
		printf("v3=%f\t", v3);
		printf("\r\n");
  }
}
