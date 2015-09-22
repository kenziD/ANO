#include "config.h"
#include <stdio.h>
#include <string.h>
extern unsigned char ID;

int main(void)
{
	float q[4];
	float ypr[3]; // yaw pitch roll
	u8 tmp_buf[5] = {0};
	float Receive_Data = 0;
	int throttle = 0;//油门 999为满油门
	char status[] = "stop";

	RCC_HSE_Configuration();
	SysTick_Init();
	USART1_Config(115200);
	LED_Init();
	IIC_Init();
	Initial_Timer3();
	TIM2_Init(999,0);
	Mpu6050_Init_offset();
	Mpu6050init();
	MOT_GPIO_init();
	MOT_PWM_init();
	NRF24L01_Init();    	

	while (NRF24L01_Check())	
	{
		printf("no");
	}

	NRF24L01_RX_Mode();
	TIM_SetCompare1(TIM2,0);//刚开始先别转 要不然很吓人
	TIM_SetCompare2(TIM2,0);
	TIM_SetCompare3(TIM2,0);
	TIM_SetCompare4(TIM2,0);
	while (1)
	{
//		Read_Mpu6050();
//		IMU_getQ(q);
//		IMU_getYawPitchRoll(ypr);

		if (NRF24L01_RxPacket(tmp_buf) == 0)
		{
			Receive_Data = (float)(tmp_buf[1] << 8 | tmp_buf[0]) / 1000.0;//
			throttle = (int)(Receive_Data * 999.0);
			printf("%d\n", throttle);
			if(tmp_buf[2]=='a')
				strcpy(status,"start");//字符串赋值的方法
			if(tmp_buf[2]=='b')
				strcpy(status,"stop");
			printf("%s",status);
		}
			if(!strcmp(status,"stop"))
			{
				TIM_SetCompare1(TIM2,0);
				TIM_SetCompare2(TIM2,0);
				TIM_SetCompare3(TIM2,0);
				TIM_SetCompare4(TIM2,0);
			}
			else if(!strcmp(status,"start"))
			{
			TIM_SetCompare1(TIM2,throttle);
			TIM_SetCompare2(TIM2,throttle);
			TIM_SetCompare3(TIM2,throttle);
			TIM_SetCompare4(TIM2,throttle);
			}

		//printf("id=%x\r\n",ID);
		//printf("yaw:%f\t", ypr[0]);
		//printf("pitch:%f\t", ypr[1]);
		//printf("roll:%f\t\r\n", ypr[2]);
	}
}
