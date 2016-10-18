#include "config.h"
#include "wave.h"

extern int STA;
extern int16_t motor0, motor1, motor2, motor3;
extern float surRoll, surPitch;
extern int expThro;
extern PID_ PID_ROLL, PID_PITCH;
extern uint8_t Res[32];
extern int p;
extern float expRoll;
extern float expPitch;
extern int16_t fGYRO_X, fGYRO_Y, fGYRO_Z;    //量化的陀螺仪数据 g(9.8m/s^2)
extern int16_t fACCEL_X, fACCEL_Y, fACCEL_Z; //量化的加速度计数据 °/s

int16_t AngleOut[3];

extern u8 getMpu6050Data;
extern u8 calculateAngle;
extern u8 sendData;
u8	send_senser_cnt = 0;
u8	send_status_cnt = 0;
int cnt = 0;


int main(void)
{
	float q[4];
	float ypr[3]; // yaw pitch roll
	u8 tmp_buf[32] = {0};
	float Receive_Data = 0;
	int throttle = 0;
	char status[] = "stop";
	float RC_get_Roll = 0;
	float RC_get_Pitch = 0;
	
	static u8 led_on = 0;
	
	RCC_HSE_Configuration();
	SysTick_Init();
	NVIC_Configuration();
	USART1_Config(115200);
	LED_Init();
	LED3_Flash(2,500000);
	ANO_TC_I2C2_INIT(0xA6, 400000, 1, 1, 3, 3);
	//硬实时
	Tim3_Init(500);//0.05s
	//软实时
	//Initial_Timer3();
	TIM2_Init(999, 0);

	Mpu6050init();
	//Mpu6050_Init_offset();
	MOT_GPIO_init();
	MOT_PWM_init();
	Set_PWM(0, 0, 0, 0);
	NRF24L01_Init();
	while (NRF24L01_Check())
	{
		LED2_Flash(2,500000);
	}
	//RX mode
	//NRF24L01_Mode_Config(1);

	//TX mode
	NRF24L01_Mode_Config(4);
	PID_Init();

	while (1)
	{
		if (getMpu6050Data == 1)//0.5s period,should be 10ms
		{
			//need 0.1ms
			Read_Mpu6050();
			Mpu6050_Analyze();
			getMpu6050Data = 0;
		}
		if (calculateAngle == 1)//0.5s period
		{
			 if(led_on)
			 {
			 	LED2_OFF;
			 	led_on = 0;
			 }
			 else
			 {
			 	LED2_ON;
			 	led_on = 1;
			 }
			
			IMU_getYawPitchRoll(ypr);
			calculateAngle = 0;
		}
		if (sendData == 1)//12ms
		{
			sendSenser((int16_t)fACCEL_X, (int16_t)fACCEL_Y, (int16_t)fACCEL_Z, (int16_t)fGYRO_X, (int16_t) fGYRO_Y, (int16_t)fGYRO_Z, (int16_t)(ypr[2] * 100), (signed short int)(ypr[1] * 100));
			send_wave(32);
			sendPwmVoltage((uint16_t)(motor0 / 1000.0 * 100), (uint16_t)(motor1 / 1000.0 * 100), (uint16_t)(motor2 / 1000.0 * 100), (uint16_t)(motor3 / 1000.0 * 100), 320);
			send_wave(32);
			sendData = 0;
		}
		//Read_Mpu6050();
		//Mpu6050_Analyze();
		////moveFilterAccData(fACCEL_X, fACCEL_Y, fACCEL_Z, AngleOut);
		//IMU_getYawPitchRoll(ypr);
		
		
		// if (NRF24L01_RxPacket(tmp_buf) == 0)
		// {
		// 	LED2_ON;
		// 	Receive_Data = (float)(tmp_buf[1] << 8 | tmp_buf[0]) / 1000.0;//
		// 	throttle = (int)(Receive_Data * 999.0);
		// 	RC_get_Roll = ((tmp_buf[3] << 8 | tmp_buf[2]) - 1970) / 70.0; //????1970  ??????????????30°,????70
		// 	RC_get_Pitch = ((tmp_buf[5] << 8 | tmp_buf[4]) - 2120) / 70.0;;
		// 	if (tmp_buf[6] == 'a')
		// 		strcpy(status, "start");
		// 	if (tmp_buf[6] == 'b')
		// 		strcpy(status, "stop");
		// }else{
		// 	LED2_OFF;
		// }

		// if (!strcmp(status, "stop"))
		// {
		// 	Set_PWM(0, 0, 0, 0);
		// }
		// else if (!strcmp(status, "start"))
		// {
		// 	expRoll = RC_get_Roll;
		// 	expPitch = RC_get_Pitch;
		// 	expThro = throttle;
		// 	surRoll = ypr[2];
		// 	surPitch = ypr[1];
		// 	PID_Set();
		// 	Set_PWM(motor0, motor1, motor2, motor3);
		// }
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
