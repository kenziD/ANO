#include "config.h"
#include "wave.h"

extern int STA;//串口接收32字符完成的状态

extern int16_t motor0, motor1, motor2, motor3;
extern float surRoll,surPitch;
extern int expThro;
extern PID_ PID_ROLL,PID_PITCH;
extern uint8_t Res[32];
extern int p;

int speed =0;//NRF2401接收+-的次数

int main(void)
{
	float q[4];
	float ypr[3]; // yaw pitch roll
	u8 tmp_buf[12] ={0};
	char status[5]="stop";
	
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
	PID_Init();

	while(1)
  {
		Read_Mpu6050();
		IMU_getQ(q);
		IMU_getYawPitchRoll(ypr);
//	Uart1_send_custom_float(0xA1,ypr[1],ypr[2],ypr[0]);//发送姿态角 用自定义帧 float型
//	send_wave(16);
		while (NRF24L01_Check())	//检查NRF24L01是否在位.
		{
			printf("no");
		}
		
		if (NRF24L01_RxPacket(tmp_buf) == 0)
		{
			//printf("%s",tmp_buf);
			if(tmp_buf[0]=='+')
			speed=speed+1;	
			if(tmp_buf[0]=='-')
			speed=speed-1;
			if(!strcmp((const char *)tmp_buf,"stop"))
			strcpy(status,"stop");
			if(!strcmp((const char *)tmp_buf,"start"))
			strcpy(status,"start");
			if(speed<0)
			speed=0;
			//printf("%s",status);
			//printf("%d\n",speed);				
		}

			if(!strcmp(status,"stop"))
			{
        Set_PWM(0,0,0,0);
			}
			else if(!strcmp(status,"start"))
			{
      expThro=speed*30;

		  surRoll = ypr[2];
		  surPitch = ypr[1];
		  PID_Set();
			Set_PWM(motor0,motor1,motor2,motor3);
			}
		  //Uart1_Send_PID(320,PID_ROLL.KI,PID_ROLL.KD,1,0,0);
			//send_wave(32);
		if(STA ==1)
		{
	    receive_Data();
	  	STA = 0;
			p=0;
		}
			Uart1_Send_AF(0x00,0x00,0x00,0x00,0x00,0x00,(signed short int)(ypr[2]*100),(signed short int)(ypr[1]*100));
		  send_wave(32);
		  Uart1_Send_AE((uint16_t)(motor0/1000.0*100),(uint16_t)(motor1/1000.0*100),(uint16_t)(motor2/1000.0*100),(uint16_t)(motor3/1000.0*100),320);
		  send_wave(32);
			Uart1_send_temp((uint16_t)(speed*30));
			send_temp(6);
	
  }
}
