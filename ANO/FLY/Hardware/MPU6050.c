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
	Single_Write_Mpu6050(Mpu6050_Address, CONFIG, 0x06);  	//滤波频率：陀螺仪42hz,加速度计44hz?
	Single_Write_Mpu6050(Mpu6050_Address, GYRO_CONFIG, 0x18);
	Single_Write_Mpu6050(Mpu6050_Address, ACCEL_CONFIG, 0x08);
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
		sum_gx += GYRO_X_last;
		sum_gy += GYRO_Y_last;
		sum_gz += GYRO_Z_last;

		sum_ax += ACCEL_X_last;
		sum_ay += ACCEL_Y_last;
		sum_az += ACCEL_Z_last;
	}
	Gx_offset = sum_gx / 50.0;
	Gy_offset = sum_gy / 50.0;
	Gz_offset = sum_gz / 50.0;
	
	Ax_offset = sum_ax / 50.0;
	Ay_offset = sum_ay / 50.0;
	//az_offset 不能减去。因为水平时，az就表示了重力加速度在z轴的全部分量
	//Az_offset = sum_az / 50.0;
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
//修改成不要先换算成角度，直接把65536这个范围的数拿去和陀螺仪的弧度/s进行融合反倒特别准，甚至不用减去angleoffset就很准确率。
void Read_Mpu6050(void)
{
//	ID = Single_Read_Mpu6050(Mpu6050_Address, WHO_AM_I);
	BUF[0] = Single_Read_Mpu6050(Mpu6050_Address, GYRO_XOUT_L);
	BUF[1] = Single_Read_Mpu6050(Mpu6050_Address, GYRO_XOUT_H);
	GYRO_X_last =	(BUF[1] << 8) | BUF[0];
	fGYRO_X = (GYRO_X_last- Gx_offset) *Gyro_Gr ;

	BUF[2] = Single_Read_Mpu6050(Mpu6050_Address, GYRO_YOUT_L);
	BUF[3] = Single_Read_Mpu6050(Mpu6050_Address, GYRO_YOUT_H);
	GYRO_Y_last =	(BUF[3] << 8) | BUF[2];
	fGYRO_Y = (GYRO_Y_last - Gy_offset)*Gyro_Gr;

	BUF[4] = Single_Read_Mpu6050(Mpu6050_Address, GYRO_ZOUT_L);
	BUF[5] = Single_Read_Mpu6050(Mpu6050_Address, GYRO_ZOUT_H);
	GYRO_Z_last =	(BUF[5] << 8) | BUF[4];
	//要转换成弧度 要不然到四元数哪里也要/180*2pi 但为啥拿去四元数运算的要转成弧度
	fGYRO_Z = (GYRO_Z_last - Gz_offset) *Gyro_Gr;

	BUF[6] = Single_Read_Mpu6050(Mpu6050_Address, ACCEL_XOUT_L);
	BUF[7] = Single_Read_Mpu6050(Mpu6050_Address, ACCEL_XOUT_H);
	ACCEL_X_last =	(BUF[7] << 8) | BUF[6];
	fACCEL_X = ACCEL_X_last-Ax_offset;

	BUF[8] = Single_Read_Mpu6050(Mpu6050_Address, ACCEL_YOUT_L);
	BUF[9] = Single_Read_Mpu6050(Mpu6050_Address, ACCEL_YOUT_H);
	ACCEL_Y_last =	(BUF[9] << 8) | BUF[8];
	fACCEL_Y = ACCEL_Y_last-Ay_offset ;

	BUF[10] = Single_Read_Mpu6050(Mpu6050_Address, ACCEL_ZOUT_L);
	BUF[11] = Single_Read_Mpu6050(Mpu6050_Address, ACCEL_ZOUT_H);
	ACCEL_Z_last =	(BUF[11] << 8) | BUF[10];
	//这里注意不要减去offset
	fACCEL_Z = ACCEL_Z_last ;

}
void moveFilterAccData(float angle_accX,float angle_accY,float angle_accZ,float *angleOut){
	static uint8_t 	filter_cnt=0;
	static float	ACC_X_BUF[FILTER_NUM],ACC_Y_BUF[FILTER_NUM],ACC_Z_BUF[FILTER_NUM];
	int32_t temp1=0,temp2=0,temp3=0;
	uint8_t i;

	ACC_X_BUF[filter_cnt] = angle_accX;
	ACC_Y_BUF[filter_cnt] = angle_accY;
	ACC_Z_BUF[filter_cnt] = angle_accZ;
	for(i=0;i<FILTER_NUM;i++)
	{
		temp1 += ACC_X_BUF[i];
		temp2 += ACC_Y_BUF[i];
		temp3 += ACC_Z_BUF[i];
	}
	angleOut[0] = temp1 / FILTER_NUM;
	angleOut[1] = temp2 / FILTER_NUM;
	angleOut[2] = temp3 / FILTER_NUM;
	filter_cnt++;
	if(filter_cnt==FILTER_NUM)	filter_cnt=0;
}
