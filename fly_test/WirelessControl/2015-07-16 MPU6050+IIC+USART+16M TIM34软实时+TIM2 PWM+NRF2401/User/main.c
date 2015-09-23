#include "config.h"

extern unsigned char ID;
int count =0;
int main(void)
{
	float q[4];
	float ypr[3]; // yaw pitch roll
	u8 tmp_buf[5] ={0};
	char status[5]="start";
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
	NRF24L01_Init();    	//初始化NRF24L01
	NRF24L01_RX_Mode();
	
	while(1)
  {
//		Read_Mpu6050();
//		IMU_getQ(q);
//		IMU_getYawPitchRoll(ypr);
		while (NRF24L01_Check())	//检查NRF24L01是否在位.
		{
			printf("no");
		}
		printf("yes");
			if (NRF24L01_RxPacket(tmp_buf) == 0)
			{
//				if(tmp_buf[1]==1)
//				printf("up");
//				if(tmp_buf[1]==2)
//				printf("left");
//				if(tmp_buf[1]==3)
//				printf("down");
//				if(tmp_buf[1]==4)
//				printf("right");

				printf("%d\t",tmp_buf[0]);
				printf("%d\t",tmp_buf[1]);
				printf("%d\t",tmp_buf[2]);
				printf("%d\t",tmp_buf[3]);
//				if(tmp_buf[0]=='+')
//				count=count+1;	
//				if(tmp_buf[0]=='-')
//				count=count-1;
//				if(!strcmp(tmp_buf,"stop"))
//				strcpy(status,"stop");
//				if(!strcmp(tmp_buf,"start"))
//				strcpy(status,"start");
//				if(count<0)
//				count=0;
				//printf("%s",status);
				//printf("%d\n",count);				
			}
//			if(!strcmp(status,"stop"))
//			{
//				TIM_SetCompare1(TIM2,0);//占空比M1
//				TIM_SetCompare2(TIM2,0);//占空比M2
//				TIM_SetCompare3(TIM2,0);//占空比M0
//				TIM_SetCompare4(TIM2,0);//占空比M3
//			}
//			else if(!strcmp(status,"start"))
//			{
//				TIM_SetCompare1(TIM2,10*count);//占空比M1
//				TIM_SetCompare2(TIM2,10*count);//占空比M2
//				TIM_SetCompare3(TIM2,10*count);//占空比M0
//				TIM_SetCompare4(TIM2,count*10);//占空比M3
//			}
		
		//printf("id=%x\r\n",ID);
		//printf("yaw:%f\t", ypr[0]);
	//	printf("pitch:%f\t", ypr[1]);
		//printf("roll:%f\t\r\n", ypr[2]);
  }
}
