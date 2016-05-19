#include "config.h"
#include "wave.h"
#include <stdio.h>

float ypr[3]; // yaw pitch roll
float RC_get_Roll = 0;
float RC_get_Pitch = 0;
extern u8 FLAG_ATT;
int throttle = 0;
int main(void)
{

	u8 tmp_buf[32] = {0};
	float Receive_Data = 0;

	char status[] = "stop";

	u8	send_status_cnt = 0;
	u8	send_senser_cnt = 0;
	u8	send_rcdata_cnt = 0;
	int cnt = 0;
	int send_cnt = 0;
	RCC_HSE_Configuration();
	SysTick_Init();
	USART1_Config(115200);
	LED_Init();
	IIC_Init();
	Tim3_Init(500);//0.5ms
	TIM2_Init(999, 0);
	Mpu6050_Init_offset();
	Mpu6050init();
	MOT_GPIO_init();
	MOT_PWM_init();
	Set_PWM(0, 0, 0, 0);
	NRF24L01_Init();    			//
	while (NRF24L01_Check())	//
	{
		LED_ON;
	}

	NRF24L01_RX_Mode();
	PID_Init();


	while (1)
	{
		if (FLAG_ATT == 1)//1ms
		{
			FLAG_ATT = 0;
			cnt++;//1ms add one
			send_cnt++;
			send_status_cnt++;
			send_senser_cnt++;
			send_rcdata_cnt++;
			if (cnt == 1) //2ms
			{
				//nedd 30ms
				Read_Mpu6050();
			}
			else
			{
				IMU_getYawPitchRoll(ypr);
				cnt = 0;
			}
			if (send_status_cnt == 5)//10ms
			{
				send_status_flag = 1;
				send_status_cnt = 0;
			}
			if (send_senser_cnt == 5) {
				send_senser_flag = 1;
				send_senser_cnt = 0;
			}

			if (send_rcdata_cnt == 10) {
				send_rcdata_flag = 1;
				send_rcdata_cnt = 0;
			}
			//this code need 43ms
			//Uart1_Send_AF(0x00, 0x00, 0x00, 0x00, 0x00, 0x00, (signed short int)(ypr[2] * 100), (signed short int)(ypr[1] * 100));
			//send_wave(32);
			// Uart1_Send_AE(throttle,(uint16_t)(motor0 / 1000.0 * 100), (uint16_t)(motor1 / 1000.0 * 100), (uint16_t)(motor2 / 1000.0 * 100), (uint16_t)(motor3 / 1000.0 * 100), 320);
			// send_wave(32);


			// send_status((signed short int)(ypr[2] * 100), (signed short int)(ypr[1] * 100), 0x00, 0x00, 0x00, 0x01);
			// send_wave(18);
			// send_senser((signed short int)Angle_accX,(signed short int)Angle_accY,(signed short int)Angle_accZ,(signed short int)fGYRO_X,(signed short int)fGYRO_Y,(signed short int)fGYRO_Z,0x00,0x00,0x00);
			// send_wave(23);
			// send_rcdata(throttle,0x00,(signed short int)RC_get_Roll,(signed short int)RC_get_Pitch,0x00,0x00,0x00,0x00,0x00,0x00);
			// send_wave(25);
			// send_cnt = 0;

		}
		//
		// IMU_getQ(q);

		//	Uart1_send_custom_float(0xA1,ypr[1],ypr[2],ypr[0]);//·¢ËÍ×ËÌ¬½Ç ÓÃ×Ô¶¨ÒåÖ¡ floatÐÍ
		//	send_wave(16);

		if (NRF24L01_RxPacket(tmp_buf) == 0)
		{
			Receive_Data = (float)(tmp_buf[1] << 8 | tmp_buf[0]) / 1000.0;//
			throttle = (int)(Receive_Data * 999.0);
			// printf("%d\n", throttle);
			RC_get_Roll = ((tmp_buf[3] << 8 | tmp_buf[2]) - 1970) / 70.0; //????1970  ??????????????30°,????70
			RC_get_Pitch = ((tmp_buf[5] << 8 | tmp_buf[4]) - 2120) / 70.0;;
			if (tmp_buf[6] == 'a')
				strcpy(status, "start");
			if (tmp_buf[6] == 'b')
				strcpy(status, "stop");
			//printf("%s",status);
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
		//Uart1_Send_PID(320,PID_ROLL.KI,PID_ROLL.KD,1,0,0);
		//send_wave(32);
		if (STA == 1)
		{
			receive_Data();
			STA = 0;
			p = 0;
		}
	}
}
