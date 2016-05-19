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
}PID_;
extern PID_ PID;
extern PID_ PID_ROLL, PID_PITCH;
extern float expRoll,expPitch;
extern int expThro;
extern float surYaw, surRoll, surPitch;
extern int16_t motor0, motor1, motor2, motor3;
//void change_PID();
//void changePIDNum(float* pid_num);
void PID_Set(void);
void PID_Init(void);
void Set_PWM(int16_t m0, int16_t m1,int16_t m2,int16_t m3);

#endif
