#include "config.h"

extern int16_t Gx_offset, Gy_offset, Gz_offset;
extern int16_t fGYRO_X, fGYRO_Y, fGYRO_Z, T_T;		 //X,Y,ZÖá
extern int16_t fACCEL_X, fACCEL_Y, fACCEL_Z; //量化的加速度计数据  °/s
volatile uint32_t lastUpdate, now;

float q0 = 1, q1 = 0, q2 = 0, q3 = 0;        // quaternion elements representing the estimated orientation
float exInt = 0, eyInt = 0, ezInt = 0;        // scaled integral error

void Initial_Timer3(void)
{
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4 | RCC_APB1Periph_TIM3, ENABLE);
	/* TIM2 configuration*/
	/* Time Base configuration »ù±¾ÅäÖÃ ÅäÖÃ¶¨Ê±Æ÷µÄÊ±»ùµ¥Ôª*/
	TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
	TIM_TimeBaseStructure.TIM_Period = 0xffff; //×Ô¶¯ÖØ×°Öµ
	TIM_TimeBaseStructure.TIM_Prescaler = 0x0;
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);

	TIM_PrescalerConfig(TIM4, 0, TIM_PSCReloadMode_Update);
	/* Disable the TIM2 Update event */
	TIM_UpdateDisableConfig(TIM4, ENABLE);
	/* ----------------------TIM2 Configuration as slave for the TIM3 ----------*/
	/* Select the TIM2 Input Trigger: TIM3 TRGO used as Input Trigger for TIM2*/
	TIM_SelectInputTrigger(TIM4, TIM_TS_ITR2);
	/* Use the External Clock as TIM2 Slave Mode */
	TIM_SelectSlaveMode(TIM4, TIM_SlaveMode_External1);
	/* Enable the TIM2 Master Slave Mode */
	TIM_SelectMasterSlaveMode(TIM4, TIM_MasterSlaveMode_Enable);
	TIM_ARRPreloadConfig(TIM4, ENABLE);
	/* ¶¨Ê±Æ÷ÅäÖÃ:
	1.ÉèÖÃ¶¨Ê±Æ÷×î´ó¼ÆÊýÖµ 50000
	2.ÉèÖÃÊ±ÖÓ·ÖÆµÏµÊý£ºTIM_CKD_DIV1
	3. ÉèÖÃÔ¤·ÖÆµ£º  1Mhz/50000= 1hz
	4.¶¨Ê±Æ÷¼ÆÊýÄ£Ê½  ÏòÉÏ¼ÆÊýÄ£Ê½
	*/
	TIM_TimeBaseStructure.TIM_Period = 0xffff;
	TIM_TimeBaseStructure.TIM_Prescaler = 72;	 //1M µÄÊ±ÖÓ
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	//Ó¦ÓÃÅäÖÃµ½TIM3
	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);
	// Ê¹ÄÜTIM3ÖØÔØ¼Ä´æÆ÷ARR
	TIM_ARRPreloadConfig(TIM3, ENABLE);

	TIM_SelectSlaveMode(TIM3, TIM_SlaveMode_Reset);
	TIM_UpdateRequestConfig(TIM3, TIM_UpdateSource_Regular);
	/* ----------------------TIM3 Configuration as Master for the TIM2 -----------*/
	/* Use the TIM3 Update event  as TIM3 Trigger Output(TRGO) */
	TIM_SelectOutputTrigger(TIM3, TIM_TRGOSource_Update);
	/* Enable the TIM3 Master Slave Mode */
	TIM_SelectMasterSlaveMode(TIM3, TIM_MasterSlaveMode_Enable);

	//Æô¶¯¶¨Ê±Æ÷
	TIM_Cmd(TIM3, ENABLE);
	TIM_Cmd(TIM4, ENABLE);
}
uint32_t micros(void)
{
	uint32_t temp = 0 ;
	temp = TIM4->CNT; //¶Á¸ß16Î»Ê±¼ä
	temp = temp << 16;
	temp += TIM3->CNT; //¶ÁµÍ16Î»Ê±¼ä
	return temp;
}

//void prepare_data(void)
//{
//	float halfT;
//	static float Angle_gyroX_last, Angle_gyroY_last, Angle_gyroZ_last;
//	now = micros();  //¶ÁÈ¡Ê±¼ä
//	if (now < lastUpdate) { //¶¨Ê±Æ÷Òç³ö¹ýÁË¡£
//		halfT =  ((float)(now + (0xffff - lastUpdate)) / 1000000.0f);
//	}
//	else	{
//		halfT =  ((float)(now - lastUpdate) / 1000000.0f);
//	}
//	//printf("%f",halfT);
//	lastUpdate = now;	//¸üÐÂÊ±¼ä
//	//Read_Mpu6050();
//	Angle_gyroX = Angle_gyroX_last + fGYRO_X * halfT;
//	Angle_gyroY = Angle_gyroY_last - fGYRO_Y * halfT;
//	Angle_gyroZ = Angle_gyroZ_last - fGYRO_Z * halfT;
//	Angle_gyroX_last = Angle_gyroX;
//	Angle_gyroY_last = Angle_gyroY;
//	Angle_gyroZ_last = Angle_gyroZ;
//}



void IMU_Quateration_Update(float gx, float gy, float gz, float ax, float ay, float az)
{
	float norm;
	float vx, vy, vz;
	float ex, ey, ez;
	float halfT;

	now = micros();  //¶ÁÈ¡Ê±¼ä
	if (now < lastUpdate) { //¶¨Ê±Æ÷Òç³ö¹ýÁË¡£
		halfT =  ((float)(now + (0xffff - lastUpdate)) / 2000000.0f);
	}
	else	{
		halfT =  ((float)(now - lastUpdate) / 2000000.0f);
	}
	//printf("%f",halfT);
	lastUpdate = now;	//¸üÐÂÊ±¼ä
	gx *= Gyro_Gr;
	gy *= Gyro_Gr;
	gz *= Gyro_Gr;
	
	norm = sqrt(ax * ax + ay * ay + az * az);
	ax = ax / norm;
	ay = ay / norm;
	az = az / norm;

	vx = 2 * (q1 * q3 - q0 * q2);
	vy = 2 * (q0 * q1 + q2 * q3);
	vz = q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3;

	ex = (ay * vz - az * vy);
	ey = (az * vx - ax * vz);
	ez = (ax * vy - ay * vx);

	exInt = exInt + ex * Ki ;
	eyInt = eyInt + ey * Ki ;
	ezInt = ezInt + ez * Ki ;
	
	gx = gx + Kp * ex + exInt;
	gy = gy + Kp * ey + eyInt;
	gz = gz + Kp * ez + ezInt;

	q0 = q0 + (-q1 * gx - q2 * gy - q3 * gz) * halfT;
	q1 = q1 + (q0 * gx + q2 * gz - q3 * gy) * halfT;
	q2 = q2 + (q0 * gy - q1 * gz + q3 * gx) * halfT;
	q3 = q3 + (q0 * gz + q1 * gy - q2 * gx) * halfT;

	norm = sqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
	q0 = q0 / norm;
	q1 = q1 / norm;
	q2 = q2 / norm;
	q3 = q3 / norm;
}
extern float AngleOut[3];
void IMU_getQ(float * q)
{
	//IMU_Quateration_Update((float)fGYRO_X, (float)fGYRO_Y , (float)fGYRO_Z , (float)AngleOut[0], (float)AngleOut[1], (float)AngleOut[2]);
	IMU_Quateration_Update((float)fGYRO_X , (float)fGYRO_Y , (float)fGYRO_Z , (float)fACCEL_X, (float)fACCEL_Y, (float)fACCEL_Z);
	q[0] = q0; //·µ»Øµ±Ç°Öµ
	q[1] = q1;
	q[2] = q2;
	q[3] = q3;
}

u8 angle_offset_cnt = 0;
u8 angle_offset_OK = 0;
float pitch_offset = 0;
float roll_offset = 0;
float sum_roll = 0;
float sum_pitch = 0;
void IMU_getYawPitchRoll(float * angles) {
	float q[4];

	IMU_getQ(q);

//	if (angle_offset_OK)
//	{
		angles[0] = atan2(2 * q[1] * q[2] + 2 * q[0] * q[3], -2 * q[2] * q[2] - 2 * q[3] * q[3] + 1)*57.3; // yaw
		angles[1] = asin(-2 * q[1] * q[3] + 2 * q[0] * q[2]) *57.3 - pitch_offset; // pitch
		angles[2] = atan2(2 * q[2] * q[3] + 2 * q[0] * q[1], -2 * q[1] * q[1] - 2 * q[2] * q[2] + 1) *57.3 - roll_offset; // roll
//	}
//	else {
//		if (angle_offset_cnt < 200)
//		{
//			angles[0] = atan2(2 * q[1] * q[2] + 2 * q[0] * q[3], -2 * q[2] * q[2] - 2 * q[3] * q[3] + 1) *57.3; // yaw
//			angles[1] = asin(-2 * q[1] * q[3] + 2 * q[0] * q[2]) *57.3; // pitch
//			angles[2] = atan2(2 * q[2] * q[3] + 2 * q[0] * q[1], -2 * q[1] * q[1] - 2 * q[2] * q[2] + 1)*57.3; // roll
//			sum_roll += angles[2];
//			sum_pitch += angles[1];
//			angle_offset_cnt++;
//		}
//		else
//		{
//			roll_offset = sum_roll / 200.0;
//			pitch_offset = sum_pitch / 200.0;
//			angle_offset_OK = 1;
//		}

	//}
}
