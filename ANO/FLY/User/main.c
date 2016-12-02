#include "config.h"
#include "wave.h"
#include "Rc.h"
extern int STA;

extern float surRoll, surPitch,surYaw;

extern PID_ PID_ROLL, PID_PITCH;
extern uint8_t Res[32];
extern int p;
extern float expRoll;
extern float expPitch;
extern float expYaw;
int16_t AngleOut[3];

extern u8 getMpu6050Data;
extern u8 calculateAngle;
extern u8 sendData;

u8 send_Senser = 0;
u8 send_Status = 0;
u8 send_PwmWave = 0;
u8 send_AngleWave = 0;
u8 send_desirePIDAngle = 0;
int cnt = 0;

float ypr[3];


Int16xyz ACC_AVG = {0,0,0};
float gCalibrate = 0;
extern Define_Rc_Data Rc_Data;
int main(void)
{

//	static u8 led_on = 0;
	static u8 send_Senser_cnt = 0;
	static u8 send_Status_cnt = 0;
	static u8 send_desirePIDAngle_cnt = 0;
	static u8 att_cnt = 0;
	RCC_HSE_Configuration();
	SysTick_Init();
	NVIC_Configuration();
	USART1_Config(115200);
	LED_Init();
	
	ANO_TC_I2C2_INIT(0xA6, 400000, 1, 1, 3, 3);
	//硬实时
	Tim3_Init(500);//0.005s
	

	NRF24L01_Init();
	while(NRF24L01_Check())
	{
		LED2_ON;
	}
	
	
	//TX mode
	NRF24L01_Mode_Config(4);
	TIM2_Init(999, 2);
	MOT_GPIO_init();
	MOT_PWM_init();
	
	Mpu6050init();
	//SPI1_Init();    		

	PID_Init();
	ADC1_Init();
	LED3_Flash(2,100);
	while (1)
	{
		if (getMpu6050Data == 1)//1ms period
		{

			att_cnt++;
			Read_Mpu6050(); 
			Mpu6050_Analyze();
			moveFilterAccData(fACCEL_X,fACCEL_Y,fACCEL_Z,&ACC_AVG );
			getMpu6050Data = 0;
			if(att_cnt==2)
			{
				att_cnt = 0;
				//LED2_ON;
				IMU_Quateration_Update((float)fGYRO_X , (float)fGYRO_Y , (float)fGYRO_Z , (float)ACC_AVG.x, (float)ACC_AVG.y, (float)ACC_AVG.z,ypr);
				surRoll = ypr[2];
				surPitch = ypr[1];
				surYaw = ypr[0];
				ControlPID(Rc_Data.throttle);
				calculateAngle = 0;
				//LED2_OFF;
			}
		}

		if (sendData == 1)//1ms period
		{
				send_Senser_cnt++;
				send_Status_cnt++;
				send_desirePIDAngle_cnt++;
				if(send_Senser_cnt==5)//5ms
				{
					send_Senser = 1;
					send_Senser_cnt = 0;
				}
				if(send_Status_cnt==5)
				{
					send_Status = 1;
					send_Status_cnt = 0;
				}
				//if(send_desirePIDAngle_cnt==20)//20ms
				//{
				//	send_desirePIDAngle = 1;
				//	send_desirePIDAngle_cnt = 0;
				//}

			//Uart1_send_custom_three_int16((int16_t)(ypr[2]),(int16_t)(ypr[1]),(int16_t)(ypr[0]));
			//send_wave(10);
		
			//zhen2_send_custom_four_int16(motor0,motor1,motor2,motor3);
			//send_wave(12);
			sendData = 0;
		}
		////Uart1_Send_PID(320,PID_ROLL.KI,PID_ROLL.KD,1,0,0);
		////send_wave(32);
		// if (STA == 1)
		// {
		// 	receive_Data();
		// 	STA = 0;
		// 	p = 0;
		// }
	}
}
