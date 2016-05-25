#include "MPU6050.h"
#include <math.h>
#include "delay.h"

static u8	mpu6050_buffer[14];		//iic读取后存放数据

#define uchar unsigned char
unsigned char BUF[12];

short GYRO_X_last, GYRO_Y_last, GYRO_Z_last;
short ACCEL_X_last, ACCEL_Y_last, ACCEL_Z_last;

float fGYRO_X, fGYRO_Y, fGYRO_Z;		 //量化的陀螺仪数据     g(9.8m/s^2)
float fACCEL_X, fACCEL_Y, fACCEL_Z; //量化的加速度计数据  °/s

float Angle_accX, Angle_accY, Angle_accZ; //存储加速计的角度
volatile float Angle_gyroX, Angle_gyroY, Angle_gyroZ;

float Gx_offset = 0, Gy_offset = 0, Gz_offset = 0;
float Ax_offset = 0, Ay_offset = 0, Az_offset = 0;
int Mpu6050_init_offset_OK = 0;

uint8_t *id = 0;
void Delay_ms_mpu(u16 nms)
{
	uint16_t i, j;
	for (i = 0; i < nms; i++)
		for (j = 0; j < 8500; j++);
}

/******************初始化陀螺仪静止偏移*****************/
void Mpu6050_Init_offset(void)
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
		Mpu6050_Analyze();

		sum_gx += fGYRO_X;
		sum_gy += fGYRO_Y;
		sum_gz += fGYRO_Z;

		sum_ax += fACCEL_X;
		sum_ay += fACCEL_X;
		sum_az += fACCEL_X;
	}
	Gx_offset = sum_gx / 50.0;
	Gy_offset = sum_gy / 50.0;
	Gz_offset = sum_gz / 50.0;

	Ax_offset = sum_ax / 50.0;
	Ay_offset = sum_ay / 50.0;
	Az_offset = sum_az / 50.0;
	Mpu6050_init_offset_OK = 1;

}
void Mpu6050_Analyze(void)
{
//	ID = Single_Read_Mpu6050(Mpu6050_Address, WHO_AM_I);

	fGYRO_X =	(mpu6050_buffer[8] << 8) | mpu6050_buffer[9];
	fGYRO_X = fGYRO_X * Gyro_G - Gx_offset;
	
	fGYRO_Y =	(mpu6050_buffer[10] << 8) | mpu6050_buffer[11];
	fGYRO_Y = fGYRO_Y * Gyro_G - Gy_offset;

	fGYRO_Z =	(mpu6050_buffer[12] << 8) | mpu6050_buffer[13];
	fGYRO_Z = fGYRO_Z * Gyro_G - Gz_offset;

	fACCEL_X =	(mpu6050_buffer[0] << 8) | mpu6050_buffer[1];
	fACCEL_X = fACCEL_X  - Ax_offset;

	fACCEL_Y =	(mpu6050_buffer[2] << 8) | mpu6050_buffer[3];
	fACCEL_Y = fACCEL_Y  - Ay_offset;

	fACCEL_Z =	(mpu6050_buffer[4] << 8) | mpu6050_buffer[5];
	fACCEL_Z = fACCEL_Z  - Az_offset;

	//if (Mpu6050_init_range_OK)
	//{
	//	Angle_accX = (atan(fACCEL_X / sqrt(fACCEL_Z * fACCEL_Z + fACCEL_Y * fACCEL_Y)) * 180 / 3.14 - Ax_min) / X_range * 180 - 90;
	//	Angle_accY = (atan(fACCEL_Y / sqrt(fACCEL_X * fACCEL_X + fACCEL_Z * fACCEL_Z)) * 180 / 3.14 - Ay_min) / Y_range * 180 - 90;
	//	Angle_accZ = (atan(fACCEL_Z / sqrt(fACCEL_X * fACCEL_X + fACCEL_Y * fACCEL_Y)) * 180 / 3.14 - Az_min) / Z_range * 180 - 90;
	//}
	//Angle_accX = atan(fACCEL_X / sqrt(fACCEL_Z * fACCEL_Z + fACCEL_Y * fACCEL_Y)) * 180 / 3.14;
	//Angle_accY = atan(fACCEL_Y / sqrt(fACCEL_X * fACCEL_X + fACCEL_Z * fACCEL_Z)) * 180 / 3.14;
	//Angle_accZ = atan(fACCEL_Z / sqrt(fACCEL_X * fACCEL_X + fACCEL_Y * fACCEL_Y)) * 180 / 3.14;

	//Angle_accX = (atan(fACCEL_X / sqrt(fACCEL_Z * fACCEL_Z + fACCEL_Y * fACCEL_Y)) * 180 / 3.14 - Ax_min) / X_range * 180 - 90;
	//Angle_accY = (atan(fACCEL_Y / sqrt(fACCEL_X * fACCEL_X + fACCEL_Z * fACCEL_Z)) * 180 / 3.14 - Ay_min) / Y_range * 180 - 90;
	//Angle_accZ = (atan(fACCEL_Z / sqrt(fACCEL_X * fACCEL_X + fACCEL_Y * fACCEL_Y)) * 180 / 3.14 - Az_min) / Z_range * 180 - 90;
}
void moveFilterAccData(float angle_accX, float angle_accY, float angle_accZ, float *angleOut) {
	static uint8_t 	filter_cnt = 0;
	static float	ACC_X_BUF[FILTER_NUM], ACC_Y_BUF[FILTER_NUM], ACC_Z_BUF[FILTER_NUM];
	int32_t temp1 = 0, temp2 = 0, temp3 = 0;
	uint8_t i;

	ACC_X_BUF[filter_cnt] = angle_accX;
	ACC_Y_BUF[filter_cnt] = angle_accY;
	ACC_Z_BUF[filter_cnt] = angle_accZ;
	for (i = 0; i < FILTER_NUM; i++)
	{
		temp1 += ACC_X_BUF[i];
		temp2 += ACC_Y_BUF[i];
		temp3 += ACC_Z_BUF[i];
	}
	angleOut[0] = temp1 / FILTER_NUM;
	angleOut[1] = temp2 / FILTER_NUM;
	angleOut[2] = temp3 / FILTER_NUM;
	filter_cnt++;
	if (filter_cnt == FILTER_NUM)	filter_cnt = 0;
}

/**************************实现函数********************************************
//将iic读取到得数据分拆,放入相应寄存器
*******************************************************************************/
#define 	MPU6050_MAX		32767
#define		MPU6050_MIN		-32768


/**************************实现函数********************************************
//将iic读取到得数据分拆,放入相应寄存器,更新MPU6050_Last
*******************************************************************************/
void Read_Mpu6050(void)
{
	ANO_TC_I2C2_Read_Int(devAddr, MPU6050_RA_ACCEL_XOUT_H, 14, mpu6050_buffer);

}
/**************************实现函数********************************************
*函数原型:		u8 IICwriteBit(u8 dev, u8 reg, u8 bitNum, u8 data)
*功　　能:	    读 修改 写 指定设备 指定寄存器一个字节 中的1个位
输入	dev  目标设备地址
reg	   寄存器地址
bitNum  要修改目标字节的bitNum位
data  为0 时，目标位将被清0 否则将被置位
返回   成功 为1
失败为0
*******************************************************************************/
void IICwriteBit(u8 dev, u8 reg, u8 bitNum, u8 data) {
	u8 b;
	ANO_TC_I2C2_Read_Buf(dev, reg, 1, &b);
	b = (data != 0) ? (b | (1 << bitNum)) : (b & ~(1 << bitNum));
	ANO_TC_I2C2_Write_Buf(dev, reg, 1, &b);
}
/**************************实现函数********************************************
*函数原型:		u8 IICwriteBits(u8 dev,u8 reg,u8 bitStart,u8 length,u8 data)
*功　　能:	    读 修改 写 指定设备 指定寄存器一个字节 中的多个位
输入	dev  目标设备地址
reg	   寄存器地址
bitStart  目标字节的起始位
length   位长度
data    存放改变目标字节位的值
返回   成功 为1
失败为0
*******************************************************************************/
void IICwriteBits(u8 dev, u8 reg, u8 bitStart, u8 length, u8 data)
{

	u8 b, mask;
	ANO_TC_I2C2_Read_Buf(dev, reg, 1, &b);
	mask = (0xFF << (bitStart + 1)) | 0xFF >> ((8 - bitStart) + length - 1);
	data <<= (8 - length);
	data >>= (7 - bitStart);
	b &= mask;
	b |= data;
	ANO_TC_I2C2_Write_Buf(dev, reg, 1, &b);
}
/**************************实现函数********************************************
*函数原型:		void MPU6050_setClockSource(uint8_t source)
*功　　能:	    设置  MPU6050 的时钟源
* CLK_SEL | Clock Source
* --------+--------------------------------------
* 0       | Internal oscillator
* 1       | PLL with X Gyro reference
* 2       | PLL with Y Gyro reference
* 3       | PLL with Z Gyro reference
* 4       | PLL with external 32.768kHz reference
* 5       | PLL with external 19.2MHz reference
* 6       | Reserved
* 7       | Stops the clock and keeps the timing generator in reset
*******************************************************************************/
void MPU6050_setClockSource(uint8_t source) {
	IICwriteBits(devAddr, MPU6050_RA_PWR_MGMT_1, MPU6050_PWR1_CLKSEL_BIT, MPU6050_PWR1_CLKSEL_LENGTH, source);

}
/** Set full-scale gyroscope range.
* @param range New full-scale gyroscope range value
* @see getFullScaleRange()
* @see MPU6050_GYRO_FS_250
* @see MPU6050_RA_GYRO_CONFIG
* @see MPU6050_GCONFIG_FS_SEL_BIT
* @see MPU6050_GCONFIG_FS_SEL_LENGTH
*/
void MPU6050_setFullScaleGyroRange(uint8_t range) {
	IICwriteBits(devAddr, MPU6050_RA_GYRO_CONFIG, MPU6050_GCONFIG_FS_SEL_BIT, MPU6050_GCONFIG_FS_SEL_LENGTH, range);
}

/**************************实现函数********************************************
*函数原型:		void MPU6050_setFullScaleAccelRange(uint8_t range)
*功　　能:	    设置  MPU6050 加速度计的最大量程
*******************************************************************************/
void MPU6050_setFullScaleAccelRange(uint8_t range) {
	IICwriteBits(devAddr, MPU6050_RA_ACCEL_CONFIG, MPU6050_ACONFIG_AFS_SEL_BIT, MPU6050_ACONFIG_AFS_SEL_LENGTH, range);
}
/**************************实现函数********************************************
*函数原型:		void MPU6050_setSleepEnabled(uint8_t enabled)
*功　　能:	    设置  MPU6050 是否进入睡眠模式
enabled =1   睡觉
enabled =0   工作
*******************************************************************************/
void MPU6050_setSleepEnabled(uint8_t enabled) {
	IICwriteBit(devAddr, MPU6050_RA_PWR_MGMT_1, MPU6050_PWR1_SLEEP_BIT, enabled);
}

/**************************实现函数********************************************
*函数原型:		void MPU6050_setI2CMasterModeEnabled(uint8_t enabled)
*功　　能:	    设置 MPU6050 是否为AUX I2C线的主机
*******************************************************************************/
void MPU6050_setI2CMasterModeEnabled(uint8_t enabled) {
	IICwriteBit(devAddr, MPU6050_RA_USER_CTRL, MPU6050_USERCTRL_I2C_MST_EN_BIT, enabled);
}

/**************************实现函数********************************************
*函数原型:		void MPU6050_setI2CBypassEnabled(uint8_t enabled)
*功　　能:	    设置 MPU6050 是否为AUX I2C线的主机
*******************************************************************************/
void MPU6050_setI2CBypassEnabled(uint8_t enabled) {
	IICwriteBit(devAddr, MPU6050_RA_INT_PIN_CFG, MPU6050_INTCFG_I2C_BYPASS_EN_BIT, enabled);
}

void MPU6050_setDLPF(uint8_t mode)
{
	IICwriteBits(devAddr, MPU6050_RA_CONFIG, MPU6050_CFG_DLPF_CFG_BIT, MPU6050_CFG_DLPF_CFG_LENGTH, mode);
}
/**************************实现函数********************************************
*函数原型:		void MPU6050_initialize(void)
*功　　能:	    初始化 	MPU6050 以进入可用状态。
*******************************************************************************/
void Mpu6050init(void)
{
	MPU6050_setSleepEnabled(0); //进入工作状态
	Delay_ms_mpu(200);
	MPU6050_setClockSource(MPU6050_CLOCK_PLL_XGYRO); //设置时钟  0x6b   0x01
	Delay_ms_mpu(50);
	MPU6050_setFullScaleGyroRange(MPU6050_GYRO_FS_2000);//陀螺仪最大量程 +-2000度每秒
	Delay_ms_mpu(50);
	MPU6050_setFullScaleAccelRange(MPU6050_ACCEL_FS_4);	//加速度度最大量程 +-4G
	Delay_ms_mpu(50);
	MPU6050_setDLPF(MPU6050_DLPF_BW_42);
	Delay_ms_mpu(50);
	MPU6050_setI2CMasterModeEnabled(0);	 //不让MPU6050 控制AUXI2C
	Delay_ms_mpu(50);
	MPU6050_setI2CBypassEnabled(1);	 //主控制器的I2C与	MPU6050的AUXI2C	直通。控制器可以直接访问HMC5883L
	Delay_ms_mpu(50);
}
