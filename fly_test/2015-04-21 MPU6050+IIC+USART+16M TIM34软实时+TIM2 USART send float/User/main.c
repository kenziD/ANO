#include "config.h"
#include <stdio.h>

extern int end ;
extern float surRoll,surPitch;
extern int state;
extern int stop;
extern int16_t motor0, motor1, motor2, motor3;
int main(void)
{
	int i=0;
	float f=0;
	float q[4];
	float ypr[3]; // yaw pitch roll
	end = 0;
	state = 0;
	stop = 1;
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
	PID_Init();
	printf("start succesful!");
	while(1)
  {
		Read_Mpu6050();
		IMU_getQ(q);
		IMU_getYawPitchRoll(ypr);
		surRoll = ypr[2];
		surPitch = ypr[1];
		PID_Set();
		if(end == 1)
			change_PID();
		if(stop == 1)
			Set_PWM(1,1,1,1);
		else if(stop == 0)
		{
		Set_PWM(motor0,motor1,motor2,motor3);
		}

		//printf("yaw:%f\t", ypr[0]);
		//printf("pitch:%f\t", ypr[1]);
		//printf("roll:%f\t\r\n", ypr[2]);
  }
}
