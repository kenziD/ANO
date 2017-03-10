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
extern int16_t fACCEL_X_zhihu_pix , fACCEL_Y_zhihu_pix , fACCEL_Z_zhihu_pix ;
u8 send_Senser = 0;
u8 send_Status = 0;
u8 send_PwmWave = 0;
u8 send_RcData = 0;
u8 send_desirePIDAngle = 0;
int cnt = 0;

float ypr[3];

floatEurlaAngle outAngle = {0.0,0.0,0.0};
floatEurlaAngle desireAngle = {0.0f,0.0f,0.0f};
Int16xyz ACC_AVG = {0,0,0};
Int16xyz AccFilterOut = {0,0,0};
Int16xyz MPU_ACC_READ={0,0,0};
float gCalibrate = 0;
extern Define_Rc_Data Rc_Data;
int main(void)
{
	static u8 led_on = 0;
	static u8 send_Senser_cnt = 0;
	static u8 send_Status_cnt = 0;
	static u8 send_RcData_cnt = 0;
	static u8 send_desirePIDAngle_cnt = 0;
	static u8 att_cnt = 0;
	static u8 outterPid_cnt = 0;
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
	PID_Init();
	ADC1_Init();
	LED3_Flash(2,100);
	setCutOffFrequency(400,10);//T=0.02ms fs=1/T=500,Fcut=28Hz;
	while (1)
	{
		if (getMpu6050Data == 1)//1ms period
		{
			//att_cnt++;
			Read_Mpu6050(); 
			Mpu6050_Analyze();
			moveFilterAccData(fACCEL_X,fACCEL_Y,fACCEL_Z,&ACC_AVG );
			//moveFilterAccData(fACCEL_X_zhihu_pix,fACCEL_Y_zhihu_pix,fACCEL_Z_zhihu_pix,&ACC_AVG );
			
			//if(att_cnt==2)
			//{
				//att_cnt = 0;
				//outterPid_cnt++;
				//LED2_ON;
				ButterWorthLPF_2order(&MPU_ACC_READ,&AccFilterOut);
				//LED2_OFF;
				getMpu6050Data = 0;
				//LED2_ON;
				//IMU_Quateration_Update((float)fGYRO_X , (float)fGYRO_Y , (float)fGYRO_Z , (float)ACC_AVG.x, (float)ACC_AVG.y, (float)ACC_AVG.z,&outAngle);
				//IMU_Quateration_Update((float)fGYRO_X , (float)fGYRO_Y , (float)fGYRO_Z , (float)fACCEL_X, (float)fACCEL_Y, (float)fACCEL_Z,&outAngle);
				//IMU_Quateration_Update((float)fGYRO_X , (float)fGYRO_Y , (float)fGYRO_Z , (float)AccFilterOut.x, (float)AccFilterOut.y, (float)AccFilterOut.z,&outAngle);
				//desireAngle.roll = (Rc_Data.aux1-1500)/100.0f+(Rc_Data.roll-1500)/13.0f;
				//desireAngle.pitch = (Rc_Data.aux2-1500)/100.0f+(Rc_Data.pitch-1500)/13.0f;
				
				//desireAngle.roll = 0;
				//desireAngle.pitch = 0;
				gyroControl(Rc_Data.throttle);
//				//4ms运行一次内环控制。
//				if(outterPid_cnt==2)//4ms
//				{
//					outterPid_cnt = 0;
//					angleControl(&outAngle,&desireAngle,Rc_Data.throttle);
//				}
				//ControlPID(Rc_Data.throttle);
				//calculateAngle = 0;
				//LED2_OFF;
			//}
		}
		if (sendData == 1)//1ms period
		{
				send_Senser_cnt++;
				send_Status_cnt++;
				send_desirePIDAngle_cnt++;
				send_RcData_cnt++;
				if(send_Senser_cnt==1)//5ms
				{
					send_Senser = 1;
					send_Senser_cnt = 0;
				}
				if(send_Status_cnt==5)
				{
					send_Status = 1;
					send_Status_cnt = 0;
				}
				//if(send_RcData_cnt==20)
				//{
				//	send_RcData = 1;
					//send_RcData_cnt = 0;
				//}
				//if(send_desirePIDAngle_cnt==25)//5ms
				//{
				//	send_desirePIDAngle = 1;
				//	send_desirePIDAngle_cnt = 0;
				//}
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
