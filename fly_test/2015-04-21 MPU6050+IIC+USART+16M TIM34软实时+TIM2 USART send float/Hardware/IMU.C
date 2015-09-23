#include "config.h"

extern double Gx_offset, Gy_offset, Gz_offset;
extern volatile float Angle_gyroX, Angle_gyroY, Angle_gyroZ;
extern float fGYRO_X, fGYRO_Y, fGYRO_Z, T_T;		 //X,Y,ZÖá
volatile uint32_t lastUpdate, now;

float q0 = 1, q1 = 0, q2 = 0, q3 = 0;        // quaternion elements representing the estimated orientation
float exInt = 0, eyInt = 0, ezInt = 0;        // scaled integral error

extern float Angle_accX, Angle_accY, Angle_accZ; 
void Initial_Timer3(void)
{
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4 | RCC_APB1Periph_TIM3, ENABLE);// 开启时钟
	
    /* TIM4 configuration set TIM4Clk =72M/( Period +1) ( Prescaler +1)= */
	TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
	TIM_TimeBaseStructure.TIM_Period = 0xffff; // 计数次数为65535
	TIM_TimeBaseStructure.TIM_Prescaler = 0x0;
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);

	TIM_PrescalerConfig(TIM4, 0, TIM_PSCReloadMode_Update);
	/* Disable the TIM2 Update event */
	TIM_UpdateDisableConfig(TIM4, ENABLE);
	/* ----------------------TIM2 Configuration as slave for the TIM3 ----------*/
	/* Select the TIM2 Input Trigger: TIM3 TRGO used as Input Trigger for TIM2*/
	TIM_SelectInputTrigger(TIM4, TIM_TS_ITR2); // TIM4-SMCR-TS 010 内部触发2 ITR2 配置定时器4从定时器3获得输入触发
	/* Use the External Clock as TIM2 Slave Mode */
	TIM_SelectSlaveMode(TIM4, TIM_SlaveMode_External1);// TIM4-SMCR-SMS 111 外部时钟模式1 –选中的触发输入(TRGI)的上升沿驱动计数器。这里定时
//器3将作为4的预分频器。也就是定时器4的时钟和定时器3同步
	/* Enable the TIM2 Master Slave Mode */
	TIM_SelectMasterSlaveMode(TIM4, TIM_MasterSlaveMode_Enable); // TIM4-SMCR-MSM置1
	TIM_ARRPreloadConfig(TIM4, ENABLE);
	

	TIM_TimeBaseStructure.TIM_Period = 0xffff;
	TIM_TimeBaseStructure.TIM_Prescaler = 72;	 //1M 
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	
	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);
	
	TIM_ARRPreloadConfig(TIM3, ENABLE);

	TIM_SelectSlaveMode(TIM3, TIM_SlaveMode_Reset); // TIM3-SMCR-SMS 100 复位模
//式–选中的触发输入(TRGI)的上升沿重新初始化计数器，并且产生一个更新寄存器的信号。
	TIM_UpdateRequestConfig(TIM3, TIM_UpdateSource_Regular);
	/* ----------------------TIM3 Configuration as Master for the TIM2 -----------*/
	/* Use the TIM3 Update event  as TIM3 Trigger Output(TRGO) */
	TIM_SelectOutputTrigger(TIM3, TIM_TRGOSource_Update);
	/* Enable the TIM3 Master Slave Mode */
	TIM_SelectMasterSlaveMode(TIM3, TIM_MasterSlaveMode_Enable);

	TIM_Cmd(TIM3, ENABLE);
	TIM_Cmd(TIM4, ENABLE);
}
uint32_t micros(void)
{
	uint32_t temp = 0 ;
	temp = TIM4->CNT; // TIM4左移到高16位
	temp = temp << 16;
	temp += TIM3->CNT; // 再加上TIM3计数到低16位，组成32位计数器。以us位单位
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

void IMU_getQ(float * q)
{
	IMU_Quateration_Update(fGYRO_X* M_PI/180, fGYRO_Y* M_PI/180, fGYRO_Z* M_PI/180, Angle_accX, Angle_accY, Angle_accZ);
	q[0] = q0; //·µ»Øµ±Ç°Öµ
	q[1] = q1;
	q[2] = q2;
	q[3] = q3;
}
void IMU_getYawPitchRoll(float * angles) {
  float q[4]; //¡¡ËÄÔªÊý
  IMU_getQ(q); //¸üÐÂÈ«¾ÖËÄÔªÊý
  
  angles[0] = atan2(2 * q[1] * q[2] + 2 * q[0] * q[3], -2 * q[2]*q[2] - 2 * q[3] * q[3] + 1)* 180/M_PI; // yaw
  angles[1] = -asin(-2 * q[1] * q[3] + 2 * q[0] * q[2])* 180/M_PI; // pitch
  angles[2] = atan2(2 * q[2] * q[3] + 2 * q[0] * q[1], -2 * q[1] * q[1] - 2 * q[2] * q[2] + 1)* 180/M_PI; // roll
  //if(angles[0]<0)angles[0]+=360.0f;  //½« -+180¶È  ×ª³É0-360¶È
}
