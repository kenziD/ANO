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
	
	TIM_TimeBaseStructure.TIM_Period = 0xffff;
	TIM_TimeBaseStructure.TIM_Prescaler = 72;	 //1M µÄÊ±ÖÓ
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;

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
	uint32_t temp = 0;
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


float halfT = 0.001;
u8 angle_offset_cnt = 0;
u8 angle_offset_OK = 0;
float pitch_offset = 0;
float roll_offset = 0;
float sum_roll = 0;
float sum_pitch = 0;

float yawOffsetCache = 0,pitchOffsetCache = 0,rollOffsetCache = 0;

void IMU_Quateration_Update(float gx, float gy, float gz, float ax, float ay, float az,float * angles){
static u16 initCnt = 0;
	static float yawSum = 0,pitchSum = 0,rollSum = 0;
	float norm;
	float vx, vy, vz;
	float ex, ey, ez;
	// 脧脠掳脩脮芒脨漏脫脙碌脙碌陆碌脛脰碌脣茫潞脙
	float q0q0 = q0*q0;
	float q0q1 = q0*q1;
	float q0q2 = q0*q2;
//  float q0q3 = q0*q3;
	float q1q1 = q1*q1;
//  float q1q2 = q1*q2;
	float q1q3 = q1*q3;
	float q2q2 = q2*q2;
	float q2q3 = q2*q3;
	float q3q3 = q3*q3;
//	now = micros();  //露脕脠隆脢卤录盲
//	if (now < lastUpdate) { //露篓脢卤脝梅脪莽鲁枚鹿媒脕脣隆拢
//		halfT =  ((float)(now + (0xffff - lastUpdate)) / 2000000.0f);
//	}
//	else	{
//		halfT =  ((float)(now - lastUpdate) / 2000000.0f);
//	}
//lastUpdate = now;	//赂眉脨脗脢卤录盲
	if(ax*ay*az==0)
	return;
	
	gx *= Gyro_Gr;
	gy *= Gyro_Gr;
	gz *= Gyro_Gr;
	
	////////////////////////////
	/*1.3*0.05ms = 0.065ms;*/
	////////////////////////////
	
	////////////////////////////
	/*0.05ms*/
	norm = sqrt(ax * ax + ay * ay + az * az);
	///////////////////////////

	//////////////////////////
	/*0.015ms*/
	ax = ax / norm;
	ay = ay / norm;
	az = az / norm;
	//////////////////////////
	
	////////////////////////////
	/*0.8*10us= 0.008ms*/
	vx = 2 * (q1q3 - q0q2);
	vy = 2 * (q0q1 + q2q3);
	vz = q0q0 - q1q1 - q2q2 + q3q3;
	////////////////////////////
	
	///////////////////////////
	/*0.01ms*/
	ex = (ay * vz - az * vy);
	ey = (az * vx - ax * vz);
	ez = (ax * vy - ay * vx);
	
	////////////////////////////
/////////////////////////////
	/*0.01ms*/
	exInt = exInt + ex * Ki ;
	eyInt = eyInt + ey * Ki ;
	ezInt = ezInt + ez * Ki ;
	/////////////////////////////
/////////////////////////////
	/*0.01ms*/
	gx = gx + Kp * ex + exInt;
	gy = gy + Kp * ey + eyInt;
	gz = gz + Kp * ez + ezInt;
	//////////////////////////////
//////////////////////////////
	/*1.4*25us = 0.035ms*/
	q0 = q0 + (-q1 * gx - q2 * gy - q3 * gz) * halfT;
	q1 = q1 + (q0 * gx + q2 * gz - q3 * gy) * halfT;
	q2 = q2 + (q0 * gy - q1 * gz + q3 * gx) * halfT;
	q3 = q3 + (q0 * gz + q1 * gy - q2 * gx) * halfT;
	//////////////////////////////
	////////////////////////////////
	/*0.045ms*/
	//LED2_ON;
	norm = sqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
	//LED2_OFF;
	////////////////////////////////
	q0 = q0 / norm;
	q1 = q1 / norm;
	q2 = q2 / norm;
	q3 = q3 / norm;
	angles[0] += gz*Gyro_Gr*0.002;
	//angles[0] = atan2(2 * q1 * q2 + 2 * q0 * q3, -2 * q2 * q2 - 2 * q3 * q3 + 1)*57.3; // yaw
	angles[1] = asin(-2 * q1 * q3 + 2 * q0 * q2) *57.3 ; // pitch
	angles[1] -= pitch_offset;
	angles[2] = atan2(2 * q2 * q3 + 2 * q0 * q1, -2 * q1 * q1 - 2 * q2 * q2 + 1) *57.3 ; // roll
	angles[2] -=roll_offset;
	if(initCnt<=3000)
	{
		yawSum += angles[0];
		pitchSum += angles[1];
		rollSum += angles[2];
		if(initCnt==3000)
		{
			yawOffsetCache = yawSum/3000.0;
			pitchOffsetCache = pitchSum/3000.0;
			rollOffsetCache = rollSum/3000.0;
		}
		initCnt++;
	}
	}
extern float AngleOut[3];
