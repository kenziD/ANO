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
	TIM2_Init(999,0);
	Mpu6050_Init_offset();
	Mpu6050init();
	MOT_GPIO_init();
	MOT_PWM_init();
	while(1)
  {
		Read_Mpu6050();
		IMU_getQ(q);
		IMU_getYawPitchRoll(ypr);
		//TIM_SetCompare1(TIM2,10);//占空比M1
	 TIM_SetCompare2(TIM2,10);//占空比M2
	 //TIM_SetCompare3(TIM2,10);//占空比M0
	// TIM_SetCompare4(TIM2,100);//占空比M3
		//printf("id=%x\r\n",ID);
		printf("yaw:%f\t", ypr[0]);
		printf("pitch:%f\t", ypr[1]);
		printf("roll:%f\t\r\n", ypr[2]);
  }
}
