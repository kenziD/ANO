#include "config.h"
#include "wave.h"

extern unsigned char ID;
int main(void)
{
	float q[4];
	float ypr[3]; // yaw pitch roll
	int i;          //计数变量
	unsigned char Send_Count; //串口需要发送的数据个数
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
		//for(i=0;i<2*3.14;i=i+0.5)
		//{
		Uart1_send_custom(0xA1,ypr[1],ypr[2],ypr[0]);
		send_wave(16);
	
		//}
		
	}
		
}
