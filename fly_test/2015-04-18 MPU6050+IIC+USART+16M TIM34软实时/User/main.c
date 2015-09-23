#include "config.h"
extern unsigned char ID;
int main(void)
{
	float q[4];
	float ypr[3]; // yaw pitch roll
	RCC_HSE_Configuration();
	SysTick_Init();
	USART1_Config(115200);
	LED_Init();
	IIC_Init();
	Initial_Timer3();
	Mpu6050_Init_offset();
	Mpu6050init();
	while(1)
  {
		Read_Mpu6050();
		IMU_getQ(q);
		IMU_getYawPitchRoll(ypr);
		printf("id=%x\r\n",ID);
		printf("yaw:%f\t", ypr[0]);
		printf("pitch:%f\t", ypr[1]);
		printf("roll:%f\t\r\n", ypr[2]);
  }
}
