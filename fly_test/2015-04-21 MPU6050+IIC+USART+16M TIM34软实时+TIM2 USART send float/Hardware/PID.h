#ifndef __PID_H
#define __PID_H

#include "sys.h"

#define CHANGE_P 0
#define CHANGE_I 1
#define CHANGE_D 2

#define PWM_Max 999
typedef struct PID
{
    float KP, KI, KD, Imax;
}_PID;
extern _PID PID;
void change_PID();
void changePIDNum(float* pid_num);
void PID_Set(void);
void PID_Init(void);
void Set_PWM(int16_t m0, int16_t m1,int16_t m2,int16_t m3);

#endif
