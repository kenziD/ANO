#include "config.h"
#include <string.h>

float expRoll = 0;
float expPitch = 0;
float expYaw = 0;
extern int16_t fGYRO_X, fGYRO_Y, fGYRO_Z;


PID_ PID_ROLL,PID_PITCH,PID_YAW;
float surYaw, surRoll, surPitch;
float diffYaw, diffRoll, diffPitch;
int16_t motor0, motor1, motor2, motor3;


void PID_Init(void)
{
    PID_ROLL.KP = 5;
    PID_ROLL.KI = 0.05;
    PID_ROLL.KD = 0.1;

    PID_ROLL.Imax = 300;

    PID_PITCH.KP = 5;
    PID_PITCH.KI = 0.05;
    PID_PITCH.KD = 0.1;

    PID_PITCH.Imax = 300;

    PID_YAW.KP = 3.5;
    PID_YAW.KI = 0;
    PID_YAW.KD = 0.5;

    PID_YAW.Imax = 300;
}
extern Define_Rc_Data Rc_Data;
void ControlPID(int16_t expThro)
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
    if(roll_i < -PID_ROLL.Imax) roll_i = -PID_ROLL.Imax;

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
    if(pitch_i < -PID_PITCH.Imax) pitch_i = -PID_PITCH.Imax;

    pitch_Out = PID_PITCH.KP * diffPitch + pitch_i - PID_PITCH.KD * fGYRO_Y;
		//printf("%f,%f,Pitch_out:%f\r\n",surPitch,diffPitch,pitch_Out);

		
	/*Yaw*****************************************************/
    diffYaw = expYaw - surYaw;
    if(surYaw > -0.1 && surYaw < 0.1)
    {
        yaw_i = 0;
    }

    yaw_i -= PID_YAW.KI * surYaw;
    PID_YAW.Imax = diffYaw * 10;
    if(PID_YAW.Imax < 0)
    {
        PID_YAW.Imax = (-PID_YAW.Imax) + 100;
    }
    else
    {
        PID_YAW.Imax += 100;
    }

    if(yaw_i > PID_YAW.Imax) yaw_i = PID_YAW.Imax;
    if(yaw_i < -PID_YAW.Imax) yaw_i = -PID_YAW.Imax;

    yaw_Out = PID_YAW.KP * diffYaw + yaw_i - PID_YAW.KD * fGYRO_Z;
		
		if(Rc_Data.start==1)
		{
			motor0 = (int16_t)(expThro - roll_Out + pitch_Out+yaw_Out );
			motor1 = (int16_t)(expThro + roll_Out + pitch_Out-yaw_Out );
			motor2 = (int16_t)(expThro + roll_Out - pitch_Out+yaw_Out );
			motor3 = (int16_t)(expThro - roll_Out - pitch_Out-yaw_Out );
			if(motor0 > PWM_Max)  motor0 = PWM_Max;
			if(motor1 > PWM_Max)	motor1 = PWM_Max;
			if(motor2 > PWM_Max)	motor2 = PWM_Max;
			if(motor3 > PWM_Max)	motor3 = PWM_Max;

			if(motor0 < 0)  motor0 = 0;
			if(motor1 < 0)	motor1 = 0;
			if(motor2 < 0)	motor2 = 0;
			if(motor3 < 0)	motor3 = 0;
		}
		if(Rc_Data.start==0 || expThro==0){
			motor0 = 0;
			motor1 = 0;
			motor2 = 0;
			motor3 = 0;
		}
   
    
    TIM_SetCompare1(TIM2,motor1);//M1
    TIM_SetCompare2(TIM2,motor2);//M2
    TIM_SetCompare3(TIM2,motor0);//M0
    TIM_SetCompare4(TIM2,motor3);//M3
   
};


