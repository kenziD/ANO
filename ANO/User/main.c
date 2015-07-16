#include "config.h"
#include "wave.h"
extern unsigned char ID;
int speed =0;
int main(void)
{
	float q[4];
	float ypr[3]; // yaw pitch roll
	u8 tmp_buf[12] ={0};
	char status[5]="start";
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
	NRF24L01_Init();    	//初始化NRF24L01
	NRF24L01_RX_Mode();
	
	while(1)
  {
		Read_Mpu6050();
		IMU_getQ(q);
		IMU_getYawPitchRoll(ypr);
//		Uart1_send_custom(0xA1,ypr[1],ypr[2],ypr[0]);
//		send_wave(16);
		//Uart1_send_custom_int16((signed short int)ypr[1]);
    Uart1_Send_AF(0x00,0x00,0x00,0x00,0x00,0x00,(signed short int)(ypr[2]*100),(signed short int)(ypr[1]*100));
		send_wave(32);
		
		while (NRF24L01_Check())	//检查NRF24L01是否在位.
		{
			printf("no");
		}
		
			if (NRF24L01_RxPacket(tmp_buf) == 0)
			{
				printf("%s",tmp_buf);
				if(tmp_buf[0]=='+')
				speed=speed+1;	
				if(tmp_buf[0]=='-')
				speed=speed-1;
				if(!strcmp(tmp_buf,"stop"))
				strcpy(status,"stop");
				if(!strcmp(tmp_buf,"start"))
				strcpy(status,"start");
				if(speed<0)
				speed=0;
				printf("%s",status);
				printf("%d\n",speed);				
			}
			if(!strcmp(status,"stop"))
			{
				TIM_SetCompare1(TIM2,0);//占空比M1
	 //TIM_SetCompare2(TIM2,10);//占空比M2
	 //TIM_SetCompare3(TIM2,10);//占空比M0
	 //TIM_SetCompare4(TIM2,100);//占空比M3
			}
			else if(!strcmp(status,"start"))
			{
				TIM_SetCompare1(TIM2,10*speed);//占空比M1 2
	 //TIM_SetCompare2(TIM2,10);//占空比M2 3
	 //TIM_SetCompare3(TIM2,10);//占空比M0 1
	 //TIM_SetCompare4(TIM2,100);//占空比M3 4
			}
		Uart1_Send_AE(50,(uint16_t)(10*speed/1000.0*100),30,40,320);
		send_wave(32);
  }
}
