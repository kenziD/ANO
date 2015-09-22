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
	while (NRF24L01_Check())	//���NRF24L01�Ƿ���λ.Ҫ��������TX_MODE֮ǰ���������õķ��͵�ַ����check�е�0XA5 ,����ԭ���յ�ַ�Բ���
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
		if (NRF24L01_TxPacket(tmp_buf) == TX_OK)//��ͳ���5ms�����ܷ����ж���Ŷ
			{
				LED1_ON;
			} 
		else
			{
				LED1_OFF;
			}
	}
}
