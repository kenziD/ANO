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

float P_00 = 0.0f,P_01 = 0.0f,P_10 = 0.0f,P_11 = 0.0f;
float dt = 0.002;//2ms T
float Qangle = 0.001;
float Qrate = 0.003;
float R_measure = 0.03;
float K0 = 0,K1 = 0;
float D = 0;

struct GyroBias {
	float x;
	float y;
	float z;
}gyroBias = {0,0,0};

struct AccAngle{
	float accRoll;
	float accPitch;
	float accYaw;
}accAngle = {0,0,0};
struct Angle{
	float roll;
	float pitch;
	float yaw;
}angle = {0,0,0};
void IMU_KalmanFilter(float gx, float gy, float gz, float ax, float ay, float az,float * angles)
{
	static float e=0;
	P_00 = P_00 + dt*(dt*P_11-P_10-P_01)+dt*Qangle;
	P_01 = P_01 - dt*P_11;
	P_10 = P_10 - dt*P_11;
	P_11 = P_11 + Qrate*dt;

	D = P_00+R_measure;

	K0 = P_00/D;
	K1 = P_10/D;
	
	accAngle.accRoll = atan2(ay,az)*180/3.14;
	angle.roll = angle.roll + dt*(gx-gyroBias.x);//roll
	e = accAngle.accRoll-angle.roll;
	angle.roll = angle.roll + K0*e;
	gyroBias.x = gyroBias.x + K1*e;
	
	//accAngle.accPitch = atan2(ax,az)*180/3.14;
	//angle.pitch = angle.pitch + dt*(gy-gyroBias.y);//pitch
	//angle.pitch = angle.pitch + K0*(accAngle.accPitch-angle.pitch);
	//gyroBias.y = gyroBias.y + K1*(accAngle.accPitch-angle.pitch);
	
	angles[0] =angle.roll;
	angles[1] = 0;
	angles[2] = 0;
	P_00 = P_00 - P_00*K0;
	P_01 = P_01 - P_01*K0;
	P_10 = P_10 - K1*P_00;
	P_11 = P_11 - K1*P_01;
	
}
extern float AngleOut[3];
