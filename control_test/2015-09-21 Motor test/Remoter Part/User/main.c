#include "config.h"
#include "stdint.h"

u8 tmp_buf[5] = "";
u8 status = 0;
u8 key = 0;
int main(void)
{

	SysTick_Init();
	Tim3_Init(500);
	Nvic_Init();
	LED_Init();
	ADC1_Init();
	USART1_Config(115200);
	KEY_Init();
	NRF24L01_Init();
	while (NRF24L01_Check())	//检查NRF24L01是否在位.要放在设置TX_MODE之前，否则设置的发送地址会变成check中的0XA5 ,将与原接收地址对不上
	{
		LED2_ON;
		LED1_OFF;
		printf("no");
	}
	LED1_ON;
	LED2_OFF;
	NRF24L01_TX_Mode();

	while (1)
	{
		key = KEY_scan();
		if (NRF24L01_TxPacket(tmp_buf) == TX_OK)//早就超过5ms，不能放在中断里哦
			{
				LED1_ON;
			} 
		else
			{
				LED1_OFF;
			}
	}
}
