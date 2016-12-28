#ifndef __PID_H
#define __PID_H

#include "sys.h"
#include "IMU.h"
#define CHANGE_P 0
#define CHANGE_I 1
#define CHANGE_D 2

#define PWM_Max 999
typedef struct PID
{
    float KP, KI, KD, Imax,intergral,output;
}PID_;
extern PID_ PID;
//void change_PID();
//void changePIDNum(float* pid_num);
void gyroControl(int16_t expThro);
void angleControl(floatEurlaAngle *outAngle, floatEurlaAngle *desireAngle, int16_t expThro);
void ControlPID(int16_t expThro);
void PID_Init(void);
void PID_Analyze(void);
#endif
