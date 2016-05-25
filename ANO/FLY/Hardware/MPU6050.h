#ifndef __mpu6050_H
#define __mpu6050_H
#include "sys.h"
#define Mpu6050_Address 0xD0

#define	SMPLRT_DIV		0x19	//�����ǲ����ʣ�����ֵ��0x07(125Hz)(1000/(7+1))
#define	CONFIG				0x1A	//��ͨ�˲�Ƶ�ʣ�����ֵ��0x06(5Hz) 
#define	GYRO_CONFIG		0x1B	//�������Լ켰������Χ������ֵ��0x18(���Լ죬2000deg/s)
#define	ACCEL_CONFIG	0x1C	//���ټ��Լ졢������Χ����ͨ�˲�Ƶ�ʣ�����ֵ��0x01(���Լ죬2G��5Hz)

#define ACCEL_XOUT_H 0x3B
#define ACCEL_XOUT_L 0x3C
#define ACCEL_YOUT_H 0x3D
#define ACCEL_YOUT_L 0x3E
#define ACCEL_ZOUT_H 0x3F
#define ACCEL_ZOUT_L 0x40

#define	GYRO_XOUT_H		0x43
#define	GYRO_XOUT_L		0x44	
#define	GYRO_YOUT_H		0x45
#define	GYRO_YOUT_L		0x46
#define	GYRO_ZOUT_H		0x47
#define	GYRO_ZOUT_L		0x48

#define	PWR_MGMT_1		0x6B
#define	WHO_AM_I		  0x75

#define Gyro_Sen 16.4 //����GYRO_CONFIG������Χ����
#define Gyro_G 0.0610314f
#define Gyro_Gr		0.0010653f		//���ٶȱ�ɻ��� �˲�����Ӧ����2000��ÿ��	1/16.4/57.3
#define Acc_Sen 8192.0;
#define Acc_G 0.0001220f;

#define Ax_min -88.045357
#define Ay_min -89.067940
#define Az_min -89.826790
#define X_range 176.5
#define Y_range 178.54
#define Z_range 179.7

#define FILTER_NUM 20
u8 Single_Read_Mpu6050(unsigned char Mpu6050_addr,unsigned char Register_addr);
void Mpu6050init(void);
void Mpu6050_Init_offset(void);
void Mpu6050_init_range(void);
//void Read_Mpu6050(float* gx,float* gy,float* gz,float* ax,float* ay,float* az);
void Read_Mpu6050(void);
void moveFilterAccData(float angle_accX,float angle_accY,float angle_accZ,float *angleOut);
#endif

