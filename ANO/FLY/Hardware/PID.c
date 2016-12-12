#include "config.h"
#include <string.h>

float expRoll = 0;
float expPitch = 0;
float expYaw = 0;
extern int16_t fGYRO_X, fGYRO_Y, fGYRO_Z;


PID_ PID_ROLL, PID_PITCH, PID_YAW, PID_GYRO_ROLL, PID_GYRO_PITCH, PID_GYRO_YAW;
float surYaw, surRoll, surPitch;
float diffYaw, diffRoll, diffPitch;
int16_t motor0, motor1, motor2, motor3;


void PID_Init(void)
{
//    PID_ROLL.KP = 5;
//    PID_ROLL.KI = 0.05;
//    PID_ROLL.KD = 0.1;
//    PID_ROLL.Imax = 300;
//    PID_ROLL.intergral = 0;
//    PID_ROLL.output = 0;

//    PID_PITCH.KP = 5;
//    PID_PITCH.KI = 0.05;
//    PID_PITCH.KD = 0.1;
//    PID_PITCH.Imax = 309;
//    PID_PITCH.intergral = 0;
//    PID_PITCH.output = 0;

//    PID_YAW.KP = 3.5;
//    PID_YAW.KI = 0;
//    PID_YAW.KD = 0.5;
//    PID_YAW.Imax = 300;
//    PID_YAW.intergral = 0;
//    PID_YAW.output = 0;

    PID_ROLL.KP = 5;
    PID_ROLL.KI = 0.02;
    PID_ROLL.KD = 8.0;
    PID_ROLL.Imax = 1000;
    PID_ROLL.intergral = 0;
    PID_ROLL.output = 0;

    PID_PITCH.KP = 5;
    PID_PITCH.KI = 0.02;
    PID_PITCH.KD = 8.0;
    PID_PITCH.Imax = 1000;
    PID_PITCH.intergral = 0;
    PID_PITCH.output = 0;

    PID_YAW.KP = 3.5;
    PID_YAW.KI = 0;
    PID_YAW.KD = 0.5;
    PID_YAW.Imax = 300;
    PID_YAW.intergral = 0;
    PID_YAW.output = 0;

    PID_GYRO_ROLL.KP = 1.4;
    PID_GYRO_ROLL.KI = 0.0;
    PID_GYRO_ROLL.KD = 2.0;
    PID_GYRO_ROLL.Imax = 5000;
    PID_GYRO_ROLL.intergral = 0;
    PID_GYRO_ROLL.output = 0;

    PID_GYRO_PITCH.KP = 1.4;
    PID_GYRO_PITCH.KI = 0.0;
    PID_GYRO_PITCH.KD = 2.0;
    PID_GYRO_PITCH.Imax = 5000;
    PID_GYRO_PITCH.intergral = 0;
    PID_GYRO_PITCH.output = 0;

    PID_GYRO_YAW.KP = 8.0;
    PID_GYRO_YAW.KI = 0.1;
    PID_GYRO_YAW.KD = 0.1;
    PID_GYRO_YAW.Imax = 5000;
    PID_GYRO_YAW.intergral = 0;
    PID_GYRO_YAW.output = 0;
}
extern Define_Rc_Data Rc_Data;


float tmp_roll_error = 0;
float tmp_pitch_error = 0;
/*
outter loop.control angle.
*/
void angleControl(floatEurlaAngle *outAngle, floatEurlaAngle *desireAngle, int16_t expThro)
{
    static floatEurlaAngle controlAngle;
    static floatEurlaAngle lastAngle;
    //////////////////////////////////roll///////////////////////////////////////
    controlAngle.roll = desireAngle->roll - outAngle->roll;

    PID_ROLL.intergral += controlAngle.roll;

    if (PID_ROLL.intergral > PID_ROLL.Imax) PID_ROLL.intergral = PID_ROLL.Imax;
    if (PID_ROLL.intergral < -PID_ROLL.Imax) PID_ROLL.intergral = -PID_ROLL.Imax;

    // I guess it is afraid intergraling when quadcoptor not take off will cause large error.
    if (expThro < 200)
    {
        PID_ROLL.intergral  = 0;
    }
    PID_ROLL.output =  PID_ROLL.KP * controlAngle.roll + PID_ROLL.KI * PID_ROLL.intergral + PID_ROLL.KD * (controlAngle.roll - lastAngle.roll);
    lastAngle.roll = controlAngle.roll;

    //////////////////////////////////pitch/////////////////////////////////////
    controlAngle.pitch = desireAngle->pitch - outAngle->pitch;

    PID_PITCH.intergral += controlAngle.pitch;

    if (PID_PITCH.intergral > PID_PITCH.Imax) PID_PITCH.intergral = PID_PITCH.Imax;
    if (PID_PITCH.intergral < -PID_PITCH.Imax) PID_PITCH.intergral = -PID_PITCH.Imax;
    if (expThro < 200)
    {
        PID_PITCH.intergral  = 0;
    }
    PID_PITCH.output =  PID_PITCH.KP * controlAngle.pitch + PID_PITCH.KI * PID_ROLL.intergral + PID_PITCH.KD * (controlAngle.pitch - lastAngle.pitch);
    lastAngle.pitch = controlAngle.pitch;

}
void gyroControl(int16_t expThro)
{
    static floatEurlaAngle controlGyro;
    static floatEurlaAngle lastGyro;
    /////////////////////////roll//////////////////////////////////////////////////////
    controlGyro.roll = PID_ROLL.output - fGYRO_X*Gyro_G;
    PID_GYRO_ROLL.intergral += controlGyro.roll;
    if (PID_GYRO_ROLL.intergral > PID_GYRO_ROLL.Imax) PID_GYRO_ROLL.intergral = PID_GYRO_ROLL.Imax;
    if (PID_GYRO_ROLL.intergral < -PID_GYRO_ROLL.Imax) PID_GYRO_ROLL.intergral = -PID_GYRO_ROLL.Imax;
    PID_GYRO_ROLL.output = PID_GYRO_ROLL.KP * controlGyro.roll + PID_GYRO_ROLL.KI * PID_GYRO_ROLL.intergral + PID_GYRO_ROLL.KD * (controlGyro.roll - lastGyro.roll);
    lastGyro.roll = controlGyro.roll;
    /////////////////////////pitch//////////////////////////////////////////////////////
    controlGyro.pitch = PID_PITCH.output - fGYRO_Y*Gyro_G;
    PID_GYRO_PITCH.intergral += controlGyro.pitch;
    if (PID_GYRO_PITCH.intergral > PID_GYRO_PITCH.Imax) PID_GYRO_PITCH.intergral = PID_GYRO_PITCH.Imax;
    if (PID_GYRO_PITCH.intergral < -PID_GYRO_PITCH.Imax) PID_GYRO_PITCH.intergral = -PID_GYRO_PITCH.Imax;
    PID_GYRO_PITCH.output = PID_GYRO_PITCH.KP * controlGyro.pitch + PID_GYRO_PITCH.KI * PID_GYRO_PITCH.intergral + PID_GYRO_PITCH.KD * (controlGyro.pitch - lastGyro.pitch);
    lastGyro.pitch = controlGyro.pitch;
	/////////////////////////////yaw///////////////////////////////////////////////////////
		controlGyro.yaw = -fGYRO_Z*Gyro_G;
    PID_GYRO_YAW.intergral += controlGyro.yaw;
    if (PID_GYRO_YAW.intergral > PID_GYRO_YAW.Imax) PID_GYRO_YAW.intergral = PID_GYRO_YAW.Imax;
    if (PID_GYRO_YAW.intergral < -PID_GYRO_YAW.Imax) PID_GYRO_YAW.intergral = -PID_GYRO_YAW.Imax;
    PID_GYRO_YAW.output = PID_GYRO_YAW.KP * controlGyro.yaw + PID_GYRO_YAW.KI * PID_GYRO_YAW.intergral + PID_GYRO_YAW.KD * (controlGyro.yaw - lastGyro.yaw);
    lastGyro.yaw = controlGyro.yaw;

    if (Rc_Data.start == 1)
    { 
//        motor0 = (int16_t)(expThro -  PID_GYRO_ROLL.output + PID_GYRO_PITCH.output +PID_GYRO_YAW.output);
//        motor1 = (int16_t)(expThro +  PID_GYRO_ROLL.output + PID_GYRO_PITCH.output -PID_GYRO_YAW.output);
//        motor2 = (int16_t)(expThro +  PID_GYRO_ROLL.output - PID_GYRO_PITCH.output +PID_GYRO_YAW.output);
//        motor3 = (int16_t)(expThro -  PID_GYRO_ROLL.output - PID_GYRO_PITCH.output -PID_GYRO_YAW.output);
			
			        motor0 = (int16_t)(expThro -  PID_GYRO_ROLL.output + PID_GYRO_PITCH.output );
        motor1 = (int16_t)(expThro +  PID_GYRO_ROLL.output + PID_GYRO_PITCH.output);
        motor2 = (int16_t)(expThro +  PID_GYRO_ROLL.output - PID_GYRO_PITCH.output);
        motor3 = (int16_t)(expThro -  PID_GYRO_ROLL.output - PID_GYRO_PITCH.output);
			
			
//			
//				motor0 = (int16_t)(expThro);
//        motor1 = (int16_t)(expThro);
//        motor2 = (int16_t)(expThro);
//        motor3 = (int16_t)(expThro);
			
			
        if (motor0 > PWM_Max)    motor0 = PWM_Max;
        if (motor1 > PWM_Max)    motor1 = PWM_Max;
        if (motor2 > PWM_Max)    motor2 = PWM_Max;
        if (motor3 > PWM_Max)    motor3 = PWM_Max;
				//仿真中这个限制幅度是多么的重要！
        if (motor0 < 0)  motor0 = 0;
        if (motor1 < 0)  motor1 = 0;
        if (motor2 < 0)  motor2 = 0;
        if (motor3 < 0)  motor3 = 0;
    }
    if (Rc_Data.start == 0 || expThro == 0) 
		{
        motor0 = 0;
        motor1 = 0;
        motor2 = 0;
        motor3 = 0;
    }

    TIM_SetCompare1(TIM2, motor1); //M1
    TIM_SetCompare2(TIM2, motor2); //M2
    TIM_SetCompare3(TIM2, motor0); //M0
    TIM_SetCompare4(TIM2, motor3); //M3
}
void ControlPID(int16_t expThro)
{
    static float roll_Out = 0, pitch_Out = 0, yaw_Out = 0;
    static float roll_i = 0, pitch_i = 0, yaw_i = 0;

    /*Roll*****************************************************/
    diffRoll = expRoll - surRoll;
    if (surRoll > -0.1 && surRoll < 0.1)
    {
        roll_i = 0;
    }
    //注意这里是surRoll!不是diffRoll所以是减号-=.这样保证当roll为正时(0-roll)为负,gyro也为正,所以也是减号。保证pidout每一项都是负的。
    roll_i -= PID_ROLL.KI * surRoll;
    PID_ROLL.Imax = diffRoll * 10;
    //why?
    if (PID_ROLL.Imax < 0)
    {
        PID_ROLL.Imax = (-PID_ROLL.Imax) + 100;
    }
    else
    {
        PID_ROLL.Imax += 100;
    }

    if (roll_i > PID_ROLL.Imax) roll_i = PID_ROLL.Imax;
    if (roll_i < -PID_ROLL.Imax) roll_i = -PID_ROLL.Imax;

    roll_Out = PID_ROLL.KP * diffRoll + roll_i - PID_ROLL.KD * fGYRO_X;


    /*Pitch*****************************************************/
    diffPitch = expPitch - surPitch;
    if (surPitch > -0.1 && surPitch < 0.1)
    {
        pitch_i = 0;
    }

    pitch_i -= PID_PITCH.KI * surPitch;
    PID_PITCH.Imax = diffPitch * 10;
    if (PID_PITCH.Imax < 0)
    {
        PID_PITCH.Imax = (-PID_PITCH.Imax) + 100;
    }
    else
    {
        PID_PITCH.Imax += 100;
    }

    if (pitch_i > PID_PITCH.Imax) pitch_i = PID_PITCH.Imax;
    if (pitch_i < -PID_PITCH.Imax) pitch_i = -PID_PITCH.Imax;

    pitch_Out = PID_PITCH.KP * diffPitch + pitch_i - PID_PITCH.KD * fGYRO_Y;
    //printf("%f,%f,Pitch_out:%f\r\n",surPitch,diffPitch,pitch_Out);


    /*Yaw*****************************************************/
    diffYaw = expYaw - surYaw;
    if (surYaw > -0.1 && surYaw < 0.1)
    {
        yaw_i = 0;
    }

    yaw_i -= PID_YAW.KI * surYaw;
    PID_YAW.Imax = diffYaw * 10;
    if (PID_YAW.Imax < 0)
    {
        PID_YAW.Imax = (-PID_YAW.Imax) + 100;
    }
    else
    {
        PID_YAW.Imax += 100;
    }

    if (yaw_i > PID_YAW.Imax) yaw_i = PID_YAW.Imax;
    if (yaw_i < -PID_YAW.Imax) yaw_i = -PID_YAW.Imax;

    yaw_Out = PID_YAW.KP * diffYaw + yaw_i - PID_YAW.KD * fGYRO_Z;

    if (Rc_Data.start == 1)
    {
        motor0 = (int16_t)(expThro - roll_Out + pitch_Out + yaw_Out );
        motor1 = (int16_t)(expThro + roll_Out + pitch_Out - yaw_Out );
        motor2 = (int16_t)(expThro + roll_Out - pitch_Out + yaw_Out );
        motor3 = (int16_t)(expThro - roll_Out - pitch_Out - yaw_Out );
        if (motor0 > PWM_Max)  motor0 = PWM_Max;
        if (motor1 > PWM_Max)    motor1 = PWM_Max;
        if (motor2 > PWM_Max)    motor2 = PWM_Max;
        if (motor3 > PWM_Max)    motor3 = PWM_Max;

        if (motor0 < 0)  motor0 = 0;
        if (motor1 < 0)  motor1 = 0;
        if (motor2 < 0)  motor2 = 0;
        if (motor3 < 0)  motor3 = 0;
    }
    if (Rc_Data.start == 0 || expThro == 0) {
        motor0 = 0;
        motor1 = 0;
        motor2 = 0;
        motor3 = 0;
    }

    TIM_SetCompare1(TIM2, motor1); //M1
    TIM_SetCompare2(TIM2, motor2); //M2
    TIM_SetCompare3(TIM2, motor0); //M0
    TIM_SetCompare4(TIM2, motor3); //M3

};


