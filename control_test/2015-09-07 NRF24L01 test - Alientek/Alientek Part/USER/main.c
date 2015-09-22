#include "led.h"
#include "delay.h"
#include "key.h"
#include "sys.h"
#include "lcd.h"
#include "usart.h"
#include "24l01.h"
//ALIENTEK战舰STM32开发板实验32
//无线通信 实验
//技术支持：www.openedv.com
//广州市星翼电子科技有限公司
int main(void)
{
	u8 key, mode;
	u16 t = 0;
	float re=0;
	u8 tmp_buf[5] = "";
	u32 count=0;
	delay_init();	    	 //延时函数初始化
	NVIC_Configuration(); 	 //设置NVIC中断分组2:2位抢占优先级，2位响应优先级
	uart_init(115200);	 	//串口初始化为9600
	LED_Init();			     //LED端口初始化
	LCD_Init();	//初始化LCD
	KEY_Init();	 //按键初始化

	NRF24L01_Init();    	//初始化NRF24L01

	POINT_COLOR = RED; //设置字体为红色
	LCD_ShowString(60, 50, 200, 16, 16, "WarShip STM32");
	LCD_ShowString(30, 70, 200, 16, 16, "NRF24L01 TEST by kenzi D");
	LCD_ShowString(60, 90, 200, 16, 16, "ATOM@ALIENTEK");
	LCD_ShowString(60, 110, 200, 16, 16, "2014/8/11");
	while (NRF24L01_Check())	//检查NRF24L01是否在位.
	{
		LCD_ShowString(60, 130, 200, 16, 16, "NRF24L01 Error");
		delay_ms(200);
		LCD_Fill(60, 130, 239, 130 + 16, WHITE);
		delay_ms(200);
	}
	LCD_ShowString(60, 130, 200, 16, 16, "NRF24L01 OK");

	LCD_Fill(10, 150, 240, 166, WHITE); //清空上面的显示
	POINT_COLOR = BLUE; //设置字体为蓝色

		LCD_ShowString(60, 150, 200, 16, 16, "NRF24L01 RX_Mode");
		NRF24L01_RX_Mode();
		//mode=' ';//从空格键开始
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
				LCD_Fill(0, 188, 240, 218, WHITE); //清空上面的显示
			};
			LED0 = !LED0;
			delay_ms(1500);
		}
	//}
}


