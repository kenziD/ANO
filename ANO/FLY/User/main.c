#include "config.h"
#include "wave.h"
#include "Rc.h"

#include "HY_SRF05.h"

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
//超声波
float usound_height=0;
extern u8  TIM4CH4_CAPTURE_STA;		    				
extern u16	TIM4CH4_CAPTURE_VAL;	
int echo_start=0;
int main(void)
{
	static u8 led_on = 0;
	static u8 send_Senser_cnt = 0;
	static u8 send_Status_cnt = 0;
	static u8 send_RcData_cnt = 0;
	static u8 send_desirePIDAngle_cnt = 0;
	static u8 att_cnt = 0;
	static u8 outterPid_cnt = 0;
	u32 temp=0;
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
//		LED2_ON;
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
	//setCutOffFrequency(500,10);//T=0.02ms fs=1/T=500,Fcut=28Hz;
	//Ultrasound_Init
	HYSRF05_Init();
	Tim4_Init();//1Mhz caculate once(计数一次一个数就是1us)
	GPIO_ResetBits(GPIOB,GPIO_Pin_8);
	Ultrasound_Start();
	while (1)
	{
		//Ultrasound_Start();
		if(TIM4CH4_CAPTURE_STA&0X80)//
		{
			temp=TIM4CH4_CAPTURE_STA&0X3F;
			temp*=65536;//
			temp+=TIM4CH4_CAPTURE_VAL;//
			// from us change to (s).*340m/s change to m./2 double distance . /100 to cm.if here is cm,1000 是不是就不用乘了？in PID control end?
			usound_height=temp/1000.0/1000.0*340/2*100;
			TIM4CH4_CAPTURE_STA=0;//
			Ultrasound_Start();
		}
//		TIM4->CNT=0;//计数器清0
//		while(GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_9)==0);
//		TIM_Cmd(TIM4, ENABLE);
//		while(GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_9)==1&&(TIM4->CNT<TIM4->ARR-10));
//		TIM_Cmd(TIM4, DISABLE);
//		Uart1_send_custom_int16_V2(0xf2,TIM4->CNT);
//		send_wave(7);
//		usound_height=TIM4->CNT/58.8;
 
		if (getMpu6050Data == 1)//1ms period
		{
			att_cnt++;
			Read_Mpu6050(); 
			Mpu6050_Analyze();
			moveFilterAccData(fACCEL_X,fACCEL_Y,fACCEL_Z,&ACC_AVG );
			//moveFilterAccData(fACCEL_X_zhihu_pix,fACCEL_Y_zhihu_pix,fACCEL_Z_zhihu_pix,&ACC_AVG );
			IMU_Quateration_Update((float)fGYRO_X , (float)fGYRO_Y , (float)fGYRO_Z , (float)ACC_AVG.x, (float)ACC_AVG.y, (float)ACC_AVG.z,&outAngle);
			getMpu6050Data = 0;
			if(att_cnt==2)
			{
				att_cnt = 0;
				outterPid_cnt++;
				//ButterWorthLPF_2order(&MPU_ACC_READ,&AccFilterOut);
				//LED2_ON;
				
				//IMU_Quateration_Update((float)fGYRO_X , (float)fGYRO_Y , (float)fGYRO_Z , (float)fACCEL_X, (float)fACCEL_Y, (float)fACCEL_Z,&outAngle);
				//IMU_Quateration_Update((float)fGYRO_X , (float)fGYRO_Y , (float)fGYRO_Z , (float)AccFilterOut.x, (float)AccFilterOut.y, (float)AccFilterOut.z,&outAngle);
				desireAngle.roll = (Rc_Data.aux1-1500)/100.0f+(Rc_Data.roll-1500)/13.0f;
				desireAngle.pitch = (Rc_Data.aux2-1500)/100.0f+(Rc_Data.pitch-1500)/13.0f;
				
				//desireAngle.roll = 0;
				//desireAngle.pitch = 0;
				gyroControl(Rc_Data.throttle);
				//4ms运行一次内环控制。
				if(outterPid_cnt==2)//4ms
				{
					outterPid_cnt = 0;
					angleControl(&outAngle,&desireAngle,Rc_Data.throttle);
				}
				//ControlPID(Rc_Data.throttle);
				calculateAngle = 0;
				//LED2_OFF;
			}
		}

		if (sendData == 1)//1ms period
		{
				send_Senser_cnt++;
				send_Status_cnt++;
				send_desirePIDAngle_cnt++;
				send_RcData_cnt++;
				if(send_Senser_cnt==10)//5ms
				{
					send_Senser = 1;
					send_Senser_cnt = 0;
				}
				if(send_Status_cnt==15)
				{
					send_Status = 1;
					send_Status_cnt = 0;
				}
				if(send_RcData_cnt==20)
				{
					send_RcData = 1;
					send_RcData_cnt = 0;
				}
				if(send_desirePIDAngle_cnt==25)//5ms
				{
					send_desirePIDAngle = 1;
					send_desirePIDAngle_cnt = 0;
				}
			sendData = 0;
		}
	}
}
