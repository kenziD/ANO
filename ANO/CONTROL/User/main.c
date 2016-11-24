#include "config.h"
#include "stdint.h"

u8 tmp_buf[32] = {0};
u8 rc_buf[32] = {0};
u8 status = 0;
u8 key = 0;
int main(void)
{
	int dianya_fly = 0;
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
	//TX mode
	//NRF24L01_TX_Mode();
	//NRF24L01_Mode_Config(2);
	
	//RX mode
	NRF24L01_Mode_Config(3);
	while (1)
	{
		key = KEY_scan();

		
		if (NRF24L01_RxPacket(tmp_buf) == 0)
		{
			NRF24L01_TxPacket_AP(rc_buf);
			LED1_ON;
			if(tmp_buf[0]==0x88 && tmp_buf[1]==0xAE && tmp_buf[2]==0x1C)
			{
				dianya_fly = tmp_buf[17]<<8|tmp_buf[18];
				if(dianya_fly<36 && dianya_fly>20)
				{
					LED2_ON;
				}
			}
			send_wave(32,tmp_buf);
		}
		else
		{
			LED1_OFF;
		}
	}
}
