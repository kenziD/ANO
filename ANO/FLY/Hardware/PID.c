#include "config.h"
#include <string.h>

float expRoll = 0;
float expPitch = 0;
int expThro = 0;
// extern float fGYRO_X, fGYRO_Y, fGYRO_Z;


PID_ PID_ROLL,PID_PITCH,PID_YAW;
float surYaw, surRoll, surPitch;
float diffYaw, diffRoll, diffPitch;
int16_t motor0, motor1, motor2, motor3;


void PID_Init(void)
{
    PID_ROLL.KP = 1;
    PID_ROLL.KI = 0;
    PID_ROLL.KD = 0.5;

    PID_ROLL.Imax = 300;

    PID_PITCH.KP = 1;
    PID_PITCH.KI = 0;
    PID_PITCH.KD = 0.5;

    PID_PITCH.Imax = 300;
		//printf("%f,%f,%f",PID_ROLL.KP,PID_ROLL.KI,PID_ROLL.KD);
}
void PID_Set(void)
{
    static float roll_Out = 0, pitch_Out = 0, yaw_Out = 0;
    static float roll_i = 0, pitch_i = 0, yaw_i = 0;

    /*Roll*****************************************************/
    diffRoll = expRoll - surRoll;
    if(surRoll > -0.1 && surRoll < 0.1)
    {
        roll_i = 0;
    }

    roll_i -= PID_ROLL.KI * surRoll;
    PID_ROLL.Imax = diffRoll * 10;
    if(PID_ROLL.Imax < 0)
    {
        PID_ROLL.Imax = (-PID_ROLL.Imax) + 100;
    }
    else
    {
        PID_ROLL.Imax += 100;
    }

    if(roll_i > PID_ROLL.Imax) roll_i = PID_ROLL.Imax;
    if(roll_i > PID_ROLL.Imax) roll_i = -PID_ROLL.Imax;

    roll_Out = PID_ROLL.KP * diffRoll + roll_i - PID_ROLL.KD * fGYRO_X;
		
		
		/*Pitch*****************************************************/
    diffPitch = expPitch - surPitch;
    if(surPitch > -0.1 && surPitch < 0.1)
    {
        pitch_i = 0;
    }

    pitch_i -= PID_PITCH.KI * surPitch;
    PID_PITCH.Imax = diffPitch * 10;
    if(PID_PITCH.Imax < 0)
    {
        PID_PITCH.Imax = (-PID_PITCH.Imax) + 100;
    }
    else
    {
        PID_PITCH.Imax += 100;
    }

    if(pitch_i > PID_PITCH.Imax) pitch_i = PID_PITCH.Imax;
    if(pitch_i > PID_PITCH.Imax) pitch_i = -PID_PITCH.Imax;

    pitch_Out = PID_PITCH.KP * diffPitch + pitch_i - PID_PITCH.KD * fGYRO_Y;
		//printf("%f,%f,Pitch_out:%f\r\n",surPitch,diffPitch,pitch_Out);

    /***************************/
    motor0 = (int16_t)(expThro - roll_Out + pitch_Out );
    motor1 = (int16_t)(expThro + roll_Out + pitch_Out );
    motor2 = (int16_t)(expThro + roll_Out - pitch_Out );
    motor3 = (int16_t)(expThro - roll_Out - pitch_Out );
		
    //motor0 = (int16_t)(expThro - roll_Out );
    //motor1 = (int16_t)(expThro + roll_Out );
    //motor2 = (int16_t)(expThro + roll_Out );
    //motor3 = (int16_t)(expThro - roll_Out );
		
		//motor0 = (int16_t)(expThro + pitch_Out );
    //motor1 = (int16_t)(expThro + pitch_Out );
    //motor2 = (int16_t)(expThro - pitch_Out );
    //motor3 = (int16_t)(expThro - pitch_Out );
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



