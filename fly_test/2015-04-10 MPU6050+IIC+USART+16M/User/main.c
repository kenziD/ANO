#include "config.h"
#include "stdint.h"
extern unsigned char ID;
extern float fGYRO_X, fGYRO_Y, fGYRO_Z;	
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
	
	Mpu6050init();
	Mpu6050_Init_offset();
	while(1)
  {
		Read_Mpu6050();
		IMU_getQ(q);
		IMU_getYawPitchRoll(ypr);
		printf("id=%x\r\n",ID);
		printf("yaw:%f\t", ypr[0]);
		printf("pitch:%f\t", ypr[1]);
		printf("roll:%f\t\r\n", ypr[2]);
		printf("fGYRO_X:%f\t", fGYRO_X);
		printf("fGYRO_Y:%f\t", fGYRO_Y);
		printf("fGYRO_Z:%f\t\r\n", fGYRO_Z);
	
  }
}
