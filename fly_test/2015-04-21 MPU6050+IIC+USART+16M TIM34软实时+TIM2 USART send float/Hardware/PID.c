#include "config.h"
#include <string.h>
uint8_t expRoll = 0;
uint8_t expPitch = 0;
extern float fGYRO_X, fGYRO_Y, fGYRO_Z;
extern unsigned char ID;
extern uint16_t  Res;
extern uint8_t Data[4];
extern int end;
extern int state;

_PID PID;

float surYaw, surRoll, surPitch;
float diffYaw, diffRoll, diffPitch;
int16_t motor0, motor1, motor2, motor3;
int expThro = 200;

void changePIDNum(float* pid_num)
{
	float f;
	memcpy(&f, &Data[0], 4);
	*pid_num = f;
	
	end = 0;
}
void change_PID()
{
	if (state == CHANGE_P) 		changePIDNum(&PID.KP);
	else if (state == CHANGE_I) changePIDNum(&PID.KI);
	else if (state == CHANGE_D) changePIDNum(&PID.KD);
	printf("%f,%f,%f\n",PID.KP,PID.KI,PID.KD);
}

void PID_Init(void)
{
    PID.KP = 1;
    PID.KI = 0;
    PID.KD = 0;

    PID.Imax = 300;
		printf("%f,%f,%f",PID.KP,PID.KI,PID.KD);
}
void PID_Set(void)
{
    static float thro_Out = 0, roll_Out = 0, pitch_Out = 0, yaw_Out = 0;
    static float roll_i = 0, pitch_i = 0, yaw_i = 0;

    /*Roll*****************************************************/
    diffRoll = expRoll - surRoll;
    if(surRoll > -0.1 && surRoll < 0.1)
    {
        roll_i = 0;
    }

    roll_i -= PID.KI * surRoll;
    PID.Imax = diffRoll * 10;
    if(PID.Imax < 0)
    {
        PID.Imax = (-PID.Imax) + 100;
    }
    else
    {
        PID.Imax += 100;
    }

    if(roll_i > PID.Imax) roll_i = PID.Imax;
    if(roll_i > PID.Imax) roll_i = -PID.Imax;

    roll_Out = PID.KP * diffRoll + roll_i - PID.KD * fGYRO_X;
		
		//printf("%f\t%f\t%f\t",diffRoll,roll_i,roll_Out);
    /*Pitch*****************************************************/
    //diffPitch = expPitch - surPitch;
    //if(surPitch > -0.1 && surPitch < 0.1)
    //{
    //    pitch_i = 0;
    //}
//
    //pitch_i -= PID.KI * surPitch;
//    PID.Imax = diffPitch * 10;
//    if(PID.Imax < 0)
//    {
//        PID.Imax = (-PID.Imax) + 100;
//    }
//    else
//    {
//        PID.Imax += 100;
//    }

//    if(pitch_i > PID.Imax) pitch_i = PID.Imax;
//    if(pitch_i > PID.Imax) pitch_i = -PID.Imax;

  //  pitch_Out = PID.KP * diffPitch + pitch_i - PID.KD * fGYRO_Y;

//printf("%f\t%f\t%f\t",diffPitch,pitch_i,pitch_Out);
    /***************************/
   // motor0 = (int16_t)(expThro            + pitch_Out + yaw_Out);
   // motor1 = (int16_t)(expThro - roll_Out             - yaw_Out);
   // motor2 = (int16_t)(expThro            - pitch_Out + yaw_Out);
   // motor3 = (int16_t)(expThro + roll_Out             - yaw_Out);
    motor0 = (int16_t)(expThro - roll_Out );
    motor1 = (int16_t)(expThro + roll_Out );
    motor2 = (int16_t)(expThro + roll_Out );
    motor3 = (int16_t)(expThro - roll_Out );
}

void Set_PWM(int16_t m0, int16_t m1,int16_t m2,int16_t m3)
{
    if(m0 > PWM_Max)  m0 = PWM_Max;
	  if(m1 > PWM_Max)	m1 = PWM_Max;
    if(m2 > PWM_Max)	m2 = PWM_Max;
    if(m3 > PWM_Max)	m3 = PWM_Max;

    if(m0 < 0)  m0 = 0;
    if(m1 < 0)	m1 = 0;
    if(m2 < 0)	m2 = 0;
    if(m3 < 0)	m3 = 0;
    

    TIM_SetCompare1(TIM2,m1);//M1
    TIM_SetCompare2(TIM2,m2);//M2
    TIM_SetCompare3(TIM2,m0);//M0
    TIM_SetCompare4(TIM2,m3);//M3
}



