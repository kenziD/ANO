#include "config.h"
#include "stdint.h"
u8 tmp_buf[32] = "";
int main(void)
{	
	//u16 Roll = 0;
	
	SysTick_Init();
	Tim3_Init(500);
	Nvic_Init();
	LED_Init();
	ADC1_Init();
	USART1_Config(115200);
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
		//Pitch = voltage4();
		//printf("Pitch:%d\r\n",Pitch);//��������ͨ��������ǰ����
		//Roll = voltage3();
		//printf("Roll:%d\r\n",Roll);//��������ͨ�����������ҷ���
		//NRF24L01���ͳ������ж���
	}
}
