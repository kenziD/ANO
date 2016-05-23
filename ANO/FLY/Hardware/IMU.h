#ifndef __IMU_H
#define __IMU_H
#include "stm32f10x.h"

#define Kp 1.6f                        // proportional gain governs rate of convergence to accelerometer/magnetometer
#define Ki 0.008f                // integral gain governs rate of convergence of gyroscope biases
#define M_PI 3.14159

extern u8 angle_offset_OK;
void Initial_Timer3(void);
uint32_t micros(void);
void prepare_data(void);
void IMU_Quateration_Update(float gx, float gy, float gz, float ax, float ay, float az);
void IMU_getQ(float * q);
void IMU_getYawPitchRoll(float * angles);
#endif
