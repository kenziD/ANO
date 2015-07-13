#include "config.h"
#include "DataScope.h"
extern unsigned char ID;
int main(void)
{
	float q[4];
	float ypr[3]; // yaw pitch roll
	unsigned char i;          //计数变量
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
		DataScope_Get_Channel_Data( ypr[0], 1 );  
		DataScope_Get_Channel_Data( ypr[1], 2 );  
		DataScope_Get_Channel_Data( ypr[2], 3 );  
		Send_Count = DataScope_Data_Generate(3); 
		for ( i = 0 ; i < Send_Count; i++) 
		{
			while ((USART1->SR & 0X40) == 0);
			USART1->DR = DataScope_OutPut_Buffer[i]; 
		}
		delay_ms(50);
  }
}
