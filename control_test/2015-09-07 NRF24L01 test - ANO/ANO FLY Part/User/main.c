#include "config.h"

extern unsigned char ID;
int count = 0;
int main(void)
{
	float q[4];
	float ypr[3]; // yaw pitch roll
	u8 tmp_buf[5] = "";
	float Receive_Data = 0;
	int throttle = 0;//油门 999为满油门
	char status[5] = "start";

	RCC_HSE_Configuration();
	SysTick_Init();
	USART1_Config(115200);
	LED_Init();
	IIC_Init();
	Initial_Timer3();
	//TIM2_Init(999,0);
	Mpu6050_Init_offset();
	Mpu6050init();
	//MOT_GPIO_init();
	//MOT_PWM_init();
	NRF24L01_Init();    	//³õÊ¼»¯NRF24L01

	while (NRF24L01_Check())	//¼ì²éNRF24L01ÊÇ·ñÔÚÎ».
	{
		printf("no");
	}

	NRF24L01_RX_Mode();

	while (1)
	{
//		Read_Mpu6050();
//		IMU_getQ(q);
//		IMU_getYawPitchRoll(ypr);
		if (NRF24L01_RxPacket(tmp_buf) == 0)
		{
			Receive_Data = (float)(tmp_buf[1] << 8 | tmp_buf[0]) / 1000.0;
			throttle = (int)(Receive_Data * 999.0);
			printf("%d\t", throttle);
		}
//			if(!strcmp(status,"stop"))
//			{
//				TIM_SetCompare1(TIM2,0);//Õ¼¿Õ±ÈM1
//				TIM_SetCompare2(TIM2,0);//Õ¼¿Õ±ÈM2
//				TIM_SetCompare3(TIM2,0);//Õ¼¿Õ±ÈM0
//				TIM_SetCompare4(TIM2,0);//Õ¼¿Õ±ÈM3
//			}
//			else if(!strcmp(status,"start"))
//			{
//			TIM_SetCompare1(TIM2,10*count);//Õ¼¿Õ±ÈM1
//			TIM_SetCompare2(TIM2,10*count);//Õ¼¿Õ±ÈM2
//			TIM_SetCompare3(TIM2,10*count);//Õ¼¿Õ±ÈM0
//			TIM_SetCompare4(TIM2,count*10);//Õ¼¿Õ±ÈM3
//			}

		//printf("id=%x\r\n",ID);
		//printf("yaw:%f\t", ypr[0]);
		//	printf("pitch:%f\t", ypr[1]);
		//printf("roll:%f\t\r\n", ypr[2]);
	}
}
