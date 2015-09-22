#include "led.h"
#include "delay.h"
#include "key.h"
#include "sys.h"
#include "lcd.h"
#include "usart.h"
#include "24l01.h"
//ALIENTEKս��STM32������ʵ��32
//����ͨ�� ʵ��
//����֧�֣�www.openedv.com
//������������ӿƼ����޹�˾
int main(void)
{
	u8 key, mode;
	u16 t = 0;
	float re=0;
	u8 tmp_buf[5] = "";
	u32 count=0;
	delay_init();	    	 //��ʱ������ʼ��
	NVIC_Configuration(); 	 //����NVIC�жϷ���2:2λ��ռ���ȼ���2λ��Ӧ���ȼ�
	uart_init(115200);	 	//���ڳ�ʼ��Ϊ9600
	LED_Init();			     //LED�˿ڳ�ʼ��
	LCD_Init();	//��ʼ��LCD
	KEY_Init();	 //������ʼ��

	NRF24L01_Init();    	//��ʼ��NRF24L01

	POINT_COLOR = RED; //��������Ϊ��ɫ
	LCD_ShowString(60, 50, 200, 16, 16, "WarShip STM32");
	LCD_ShowString(30, 70, 200, 16, 16, "NRF24L01 TEST by kenzi D");
	LCD_ShowString(60, 90, 200, 16, 16, "ATOM@ALIENTEK");
	LCD_ShowString(60, 110, 200, 16, 16, "2014/8/11");
	while (NRF24L01_Check())	//���NRF24L01�Ƿ���λ.
	{
		LCD_ShowString(60, 130, 200, 16, 16, "NRF24L01 Error");
		delay_ms(200);
		LCD_Fill(60, 130, 239, 130 + 16, WHITE);
		delay_ms(200);
	}
	LCD_ShowString(60, 130, 200, 16, 16, "NRF24L01 OK");

	LCD_Fill(10, 150, 240, 166, WHITE); //����������ʾ
	POINT_COLOR = BLUE; //��������Ϊ��ɫ

		LCD_ShowString(60, 150, 200, 16, 16, "NRF24L01 RX_Mode");
		NRF24L01_RX_Mode();
		//mode=' ';//�ӿո����ʼ
		while (1)
		{

			if (NRF24L01_RxPacket(tmp_buf) == 0)
			{
				tmp_buf[4]=0;
				re = (float)(tmp_buf[1]<<8|tmp_buf[0]);
				LCD_ShowString(60, 170, 239, 32, 16, "RECIEVED DATA:");
				//LCD_ShowString(60, 190, 239, 32, 16, tmp_buf);
				LCD_ShowNum(60,190,re,3,16);

			} else
			{
				LCD_ShowString(60, 170, 239, 32, 16, "Recieved Failed ");
				LCD_Fill(0, 188, 240, 218, WHITE); //����������ʾ
			};
			LED0 = !LED0;
			delay_ms(1500);
		}
	//}
}


