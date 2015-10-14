#include "IIC.h"
#include <math.h>
#include "MPU6050.h"
#include "delay.h"

#define uchar unsigned char
unsigned char BUF[12];      
short GYRO_X_last, GYRO_Y_last, GYRO_Z_last; 
short ACCEL_X_last, ACCEL_Y_last, ACCEL_Z_last;

float fGYRO_X, fGYRO_Y, fGYRO_Z;		 //量化的陀螺仪数据     g(9.8m/s^2)
float fACCEL_X, fACCEL_Y, fACCEL_Z; //量化的加速度计数据  °/s

float Angle_accX, Angle_accY, Angle_accZ; //存储加速计的角度
volatile float Angle_gyroX, Angle_gyroY, Angle_gyroZ;

double Gx_offset = 0, Gy_offset = 0, Gz_offset = 0;
double Ax_offset = 0, Ay_offset = 0, Az_offset = 0;
int Mpu6050_init_offset_OK = 0;

/*****************************利用iic写一个字节***************************/
void Single_Write_Mpu6050(uchar Mpu6050_addr, uchar Register_addr, uchar Register_data)
{
	IIC_Start();
	IIC_Send_Byte(Mpu6050_addr);
	IIC_Send_Byte(Register_addr);
	IIC_Send_Byte(Register_data);
	IIC_Stop();
}

/************************利用iic读取一个字节*************************/
u8 Single_Read_Mpu6050(uchar Mpu6050_addr, uchar Register_addr)
{
	u8 Reg_data;
	IIC_Start();
	IIC_Send_Byte(Mpu6050_addr);
	IIC_Send_Byte(Register_addr);
	IIC_Start();
	IIC_Send_Byte(Mpu6050_addr + 1);
	Reg_data = IIC_Read_Byte(); //应答与不应答有啥区别？
	IIC_NAck();//主机（stm32）发送一个NAck表示读取结束
	IIC_Stop();
	return Reg_data;
}

/*****************************初始化设置*************************************/
void Mpu6050init()
{
	Single_Write_Mpu6050(Mpu6050_Address, PWR_MGMT_1, 0x00); //解除休眠
	delay_ms(200);
	Single_Write_Mpu6050(Mpu6050_Address, PWR_MGMT_1, 0x03); //选取陀螺仪x轴作为时钟
	Single_Write_Mpu6050(Mpu6050_Address, SMPLRT_DIV, 0x07);
	Single_Write_Mpu6050(Mpu6050_Address, CONFIG, 0x06);  //滤波频率：陀螺仪42hz,加速度计44hz?
	Single_Write_Mpu6050(Mpu6050_Address, GYRO_CONFIG, 0x08);
	Single_Write_Mpu6050(Mpu6050_Address, ACCEL_CONFIG, 0x09);
}


/******************初始化陀螺仪静止偏移*****************/
void Mpu6050_Init_offset()
{
	unsigned char i;
	float sum_gx = 0;
	float sum_gy = 0;
	float sum_gz = 0;
	float sum_ax = 0;
	float sum_ay = 0;
	float sum_az = 0;
	delay_us(200);//开机可能有脉冲，稍等一下再计算偏移量
	for (i = 0; i < 50; i ++)
	{

		Read_Mpu6050();
		sum_gx += fGYRO_X;
		sum_gy += fGYRO_Y;
		sum_gz += fGYRO_Z;

		sum_ax += Angle_accX;
		sum_ay += Angle_accY;
		sum_az += Angle_accZ;
	}
	Gx_offset = sum_gx / 50.0;
	Gy_offset = sum_gy / 50.0;
	Gz_offset = sum_gz / 50.0;
	
	Ax_offset = sum_ax / 50.0;
	Ay_offset = sum_ay / 50.0;
	Az_offset = sum_az / 50.0;
	Mpu6050_init_offset_OK = 1;

}
/***************范围矫正*****************/
//void compare(float real_time_vel, float* max, float* min)
//{
//	if (real_time_vel > *max)
//	{
//		*max = real_time_vel;
//		return;
//	}
//
//	if (real_time_vel < *min)
//	{
//		*min = real_time_vel;
//		return;
//	}
//}
//
//void Mpu6050_init_range(void)
//{
//	int i = 0;
//	for (i = 0; i < 5000; i ++)
//	{
//		Read_Mpu6050();
//		compare(Angle_accX, &Ax_max, &Ax_min);
//		compare(Angle_accY, &Ay_max, &Ay_min);
//		compare(Angle_accZ, &Az_max, &Az_min);
//	}
//	X_range = Ax_max - Ax_min;
//	Y_range = Ay_max - Ay_min;
//	Z_range = Az_max - Az_min;
//	Mpu6050_init_range_OK = 1;
//}
void Read_Mpu6050(void)
{
//	ID = Single_Read_Mpu6050(Mpu6050_Address, WHO_AM_I);
	BUF[0] = Single_Read_Mpu6050(Mpu6050_Address, GYRO_XOUT_L);
	BUF[1] = Single_Read_Mpu6050(Mpu6050_Address, GYRO_XOUT_H);
	GYRO_X_last =	(BUF[1] << 8) | BUF[0];
	fGYRO_X = GYRO_X_last / Gyro_Sen - Gx_offset;

	BUF[2] = Single_Read_Mpu6050(Mpu6050_Address, GYRO_YOUT_L);
	BUF[3] = Single_Read_Mpu6050(Mpu6050_Address, GYRO_YOUT_H);
	GYRO_Y_last =	(BUF[3] << 8) | BUF[2];
	fGYRO_Y = GYRO_Y_last / Gyro_Sen - Gy_offset;

	BUF[4] = Single_Read_Mpu6050(Mpu6050_Address, GYRO_ZOUT_L);
	BUF[5] = Single_Read_Mpu6050(Mpu6050_Address, GYRO_ZOUT_H);
	GYRO_Z_last =	(BUF[5] << 8) | BUF[4];
	fGYRO_Z = GYRO_Z_last / Gyro_Sen - Gz_offset;

	BUF[6] = Single_Read_Mpu6050(Mpu6050_Address, ACCEL_XOUT_L);
	BUF[7] = Single_Read_Mpu6050(Mpu6050_Address, ACCEL_XOUT_H);
	ACCEL_X_last =	(BUF[7] << 8) | BUF[6];
	fACCEL_X = ACCEL_X_last / Acc_Sen;

	BUF[8] = Single_Read_Mpu6050(Mpu6050_Address, ACCEL_YOUT_L);
	BUF[9] = Single_Read_Mpu6050(Mpu6050_Address, ACCEL_YOUT_H);
	ACCEL_Y_last =	(BUF[9] << 8) | BUF[8];
	fACCEL_Y = ACCEL_Y_last / Acc_Sen;

	BUF[10] = Single_Read_Mpu6050(Mpu6050_Address, ACCEL_ZOUT_L);
	BUF[11] = Single_Read_Mpu6050(Mpu6050_Address, ACCEL_ZOUT_H);
	ACCEL_Z_last =	(BUF[11] << 8) | BUF[10];
	fACCEL_Z = ACCEL_Z_last / Acc_Sen;

	//if (Mpu6050_init_range_OK)
	//{
	//	Angle_accX = (atan(fACCEL_X / sqrt(fACCEL_Z * fACCEL_Z + fACCEL_Y * fACCEL_Y)) * 180 / 3.14 - Ax_min) / X_range * 180 - 90;
	//	Angle_accY = (atan(fACCEL_Y / sqrt(fACCEL_X * fACCEL_X + fACCEL_Z * fACCEL_Z)) * 180 / 3.14 - Ay_min) / Y_range * 180 - 90;
	//	Angle_accZ = (atan(fACCEL_Z / sqrt(fACCEL_X * fACCEL_X + fACCEL_Y * fACCEL_Y)) * 180 / 3.14 - Az_min) / Z_range * 180 - 90;
	//}
	//Angle_accX = atan(fACCEL_X / sqrt(fACCEL_Z * fACCEL_Z + fACCEL_Y * fACCEL_Y)) * 180 / 3.14;
	//Angle_accY = atan(fACCEL_Y / sqrt(fACCEL_X * fACCEL_X + fACCEL_Z * fACCEL_Z)) * 180 / 3.14;
	//Angle_accZ = atan(fACCEL_Z / sqrt(fACCEL_X * fACCEL_X + fACCEL_Y * fACCEL_Y)) * 180 / 3.14;
	
	Angle_accX = (atan(fACCEL_X / sqrt(fACCEL_Z * fACCEL_Z + fACCEL_Y * fACCEL_Y)) * 180 / 3.14 - Ax_min) / X_range * 180 - 90;
	Angle_accY = (atan(fACCEL_Y / sqrt(fACCEL_X * fACCEL_X + fACCEL_Z * fACCEL_Z)) * 180 / 3.14 - Ay_min) / Y_range * 180 - 90;
	Angle_accZ = (atan(fACCEL_Z / sqrt(fACCEL_X * fACCEL_X + fACCEL_Y * fACCEL_Y)) * 180 / 3.14 - Az_min) / Z_range * 180 - 90;
}
