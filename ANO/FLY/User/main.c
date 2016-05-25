#include "config.h"
#include "wave.h"

extern int STA;//´®¿Ú½ÓÊÕ32×Ö·ûÍê³ÉµÄ×´Ì¬
extern int16_t motor0, motor1, motor2, motor3;
extern float surRoll, surPitch;
extern int expThro;
extern PID_ PID_ROLL, PID_PITCH;
extern uint8_t Res[32];
extern int p;
extern float expRoll;
extern float expPitch;
extern float Angle_accX, Angle_accY, Angle_accZ; 
float AngleOut[3];
extern short ACCEL_X_last, ACCEL_Y_last, ACCEL_Z_last;
extern float fGYRO_X, fGYRO_Y, fGYRO_Z, T_T;		 //X,Y,ZÖá
extern float fACCEL_X, fACCEL_Y, fACCEL_Z; //量化的加速度计数据  °/s
extern short GYRO_X_last, GYRO_Y_last, GYRO_Z_last; 
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
	RCC_HSE_Configuration();
	SysTick_Init();
	USART1_Config(115200);
	LED_Init();
	IIC_Init();
	Initial_Timer3();
	TIM2_Init(999, 0);
	Mpu6050_Init_offset();
	Mpu6050init();
	MOT_GPIO_init();
	MOT_PWM_init();
	Set_PWM(0, 0, 0, 0);
	NRF24L01_Init();    			//³õÊ¼»¯NRF24L01
	while (NRF24L01_Check())	//¼ì²éNRF24L01ÊÇ·ñÔÚÎ».
	{
		LED_ON;
	}

	NRF24L01_RX_Mode();
	PID_Init();

	while (1)
	{
		Read_Mpu6050();
		IMU_getQ(q);
		//moveFilterAccData(fACCEL_X,fACCEL_Y,fACCEL_Z,AngleOut);
		IMU_getYawPitchRoll(ypr);

		if (NRF24L01_RxPacket(tmp_buf) == 0)
		{
			Receive_Data = (float)(tmp_buf[1] << 8 | tmp_buf[0]) / 1000.0;//
			throttle = (int)(Receive_Data * 999.0);
			RC_get_Roll = ((tmp_buf[3] << 8 | tmp_buf[2]) - 1970) / 70.0; //????1970  ??????????????30°,????70
			RC_get_Pitch = ((tmp_buf[5] << 8 | tmp_buf[4]) - 2120) / 70.0;;
			if (tmp_buf[6] == 'a')
				strcpy(status, "start");
			if (tmp_buf[6] == 'b')
				strcpy(status, "stop");
		}

		if (!strcmp(status, "stop"))
		{
			Set_PWM(0, 0, 0, 0);
		}
		else if (!strcmp(status, "start"))
		{
			expRoll = RC_get_Roll;
			expPitch = RC_get_Pitch;
			expThro = throttle;
			surRoll = ypr[2];
			surPitch = ypr[1];
			PID_Set();
			Set_PWM(motor0, motor1, motor2, motor3);
		}

		if (STA == 1)
		{
			receive_Data();
			STA = 0;
			p = 0;
		}
		Uart1_Send_AF( (signed short int)ACCEL_X_last,  (signed short int)ACCEL_Y_last, (signed short int) ACCEL_Z_last, (signed short int)GYRO_X_last,(signed short int) GYRO_Y_last, (signed short int)GYRO_Z_last, (signed short int)(ypr[2] * 100), (signed short int)(ypr[1] * 100));
	send_wave(32);
		Uart1_Send_AE((uint16_t)(motor0 / 1000.0 * 100), (uint16_t)(motor1 / 1000.0 * 100), (uint16_t)(motor2 / 1000.0 * 100), (uint16_t)(motor3 / 1000.0 * 100), 320);
		send_wave(32);
	}
}
