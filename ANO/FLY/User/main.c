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
extern int16_t fGYRO_X, fGYRO_Y, fGYRO_Z;		 //量化的陀螺仪数据   rad/s   计算时要转换成度/s (弧度->度的转换)
extern int16_t fACCEL_X, fACCEL_Y, fACCEL_Z; //量化的加速度计数据  g(9.8m/s^2)
extern float Angle_accX, Angle_accY, Angle_accZ; //存储加速计的角度
int16_t AngleOut[3];
extern u8 getMpu6050Data;
extern u8 calculateAngle;
extern u8 sendPort;
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
	
	
	u8	send_rcdata_cnt = 0;
	
	int send_cnt = 0;

	RCC_HSE_Configuration();
	SysTick_Init();
	Tim3_Init(500);//0.5ms
	USART1_Config(115200);
	//LED_Init();
	ANO_TC_I2C2_INIT(0xA6, 400000, 1, 1, 3, 3);
	//TIM2_Init(999, 0);
	Mpu6050init();
	//Mpu6050_Init_offset();
	//MOT_GPIO_init();
	//MOT_PWM_init();
	//Set_PWM(0, 0, 0, 0);
	//NRF24L01_Init();
//	while (NRF24L01_Check())
//	{
//		LED_ON;
//	}
//	NRF24L01_RX_Mode();
//	PID_Init();

	while (1)
	{
		//moveFilterAccData(fACCEL_X, fACCEL_Y, fACCEL_Z, AngleOut);
		if (getMpu6050Data == 1)//12ms
		{
				//nedd 8ms
				Read_Mpu6050();
				Mpu6050_Analyze();
			
			getMpu6050Data = 0;
		
		}
		if (calculateAngle == 1)//12ms
		{
			ComplementaryFilter((float)fGYRO_X , (float)fGYRO_Y , (float)fGYRO_Z , Angle_accX, Angle_accY, Angle_accZ, ypr);
			calculateAngle = 0;
			
		}
		if (sendPort == 1)//12ms
		{
			Uart1_Send_AF((int16_t)(Angle_accX * 100), (int16_t)(Angle_accY * 100), (int16_t)(Angle_accZ * 100), (int16_t)(fGYRO_X * Gyro_Gr * 1000), (int16_t) (fGYRO_Y * Gyro_Gr * 1000), (int16_t)(fGYRO_Z * Gyro_Gr * 1000), (int16_t)(ypr[0] * 100), (signed short int)(ypr[1] * 100));
			send_wave(32);
			sendPort = 0;
			
		}

		// if (NRF24L01_RxPacket(tmp_buf) == 0)
		// {
		// 	Receive_Data = (float)(tmp_buf[1] << 8 | tmp_buf[0]) / 1000.0;//
		// 	throttle = (int)(Receive_Data * 999.0);
		// 	//printf("%d\n", throttle);
		// 	RC_get_Roll = ((tmp_buf[3] << 8 | tmp_buf[2]) - 1970) / 70.0; //????1970  ??????????????30°,????70
		// 	RC_get_Pitch = ((tmp_buf[5] << 8 | tmp_buf[4]) - 2120) / 70.0;;
		// 	if (tmp_buf[6] == 'a')
		// 		strcpy(status, "start");
		// 	if (tmp_buf[6] == 'b')
		// 		strcpy(status, "stop");
		// 	//printf("%s",status);
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
		// //Uart1_Send_PID(320,PID_ROLL.KI,PID_ROLL.KD,1,0,0);
		// //send_wave(32);
		// if (STA == 1)
		// {
		// 	receive_Data();
		// 	STA = 0;
		// 	p = 0;
		// }
		
		//Uart1_Send_AE((uint16_t)(motor0 / 1000.0 * 100), (uint16_t)(motor1 / 1000.0 * 100), (uint16_t)(motor2 / 1000.0 * 100), (uint16_t)(motor3 / 1000.0 * 100), 320);
		//send_wave(32);
	}
}
