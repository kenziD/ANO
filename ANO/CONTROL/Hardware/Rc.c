#include "adc.h"
#include "Key.h"
#include "Rc.h"
extern u8 rc_buf[32];
extern u8 key;
#define upRange 2086 //2010-4096
#define downRange 2010 //0-2010
#define inStaticRange(throttle) (throttle<2015 && throttle>2005) ? true:false //中值为2010
#define BYTE0(dwTemp)       (*(char *)(&dwTemp))
#define BYTE1(dwTemp)       (*((char *)(&dwTemp) + 1))
#define BYTE2(dwTemp)       (*((char *)(&dwTemp) + 2))
#define BYTE3(dwTemp)       (*((char *)(&dwTemp) + 3))

u8 g_LoadRcReadyFlag = 0;

u16 throttleMiddle = 0;
u16 YawMiddle = 0;
u16 RollMiddle = 0;
u16 PitchMiddle = 0;
u16 aux1Middle = 0;
u16 aux2Middle = 0;
u16 aux3Middle = 0;

u16 sum_throttleMiddle = 0;
u16 sum_YawMiddle = 0;
u16 sum_RollMiddle = 0;
u16 sum_PitchMiddle = 0;
u16 sum_aux1Middle = 0;
u16 sum_aux2Middle = 0;
u16 sum_aux3Middle = 0;


// void autoMiddle(void)
// {
// 	throttleMiddle = voltage1();
// 	YawMiddle = voltage2();
// 	RollMiddle = voltage3();
// 	PitchMiddle = voltage4();
// 	aux1Middle = voltage7();
// 	aux2Middle = voltage5();
// 	aux3Middle = voltage6();
// }

void loadRcData()
{
	static u8 cnt = 0;
	static u8 autoMiddleFlag = 0;
	static u16 throttle_vol = 0;
	static u16 throttle = 0;
	static u16 rc_throttle = 0;
	static u16 gap;
	static u16 Roll = 0;
	static u16 rc_roll = 0;
	static u16 Pitch = 0;
	static u16 rc_pitch = 0;
	static u16 Yaw = 0;
	static u16 aux1 = 0;
	static u16 rc_aux1 = 0;
	static u16 aux2 = 0;
	static u16 rc_aux2 = 0;
	static u16 aux3 = 0;
	static u16 rc_aux3 = 0;
	static u8 middle = 1;
	throttle_vol = voltage1();
/* for motor test*/
//	if(throttle_vol>3000 )//左-上
//	{
//		if(middle==1)
//		{
//			middle=0;
//			if(throttle>999) throttle=999;
//			else if(throttle<(999-50) )
//			{
//				throttle = throttle + 50;
//			}
//		}
//	}
//	else if (throttle_vol<1800) //左-下
//	{
//		if(middle==1)
//		{
//			middle = 0;
//			if(throttle<=0)
//			{
//				throttle = 0;
//			}
//			else if(throttle>=50 )
//			{
//				throttle = throttle - 50;
//			}
//		}
//	}
//	else
//	{
//		middle =1;
//	}
	Yaw = voltage2();
	Roll = voltage3();
	Pitch = voltage4();
	aux1 = voltage7();
	aux2 = voltage5();
	aux3 = voltage6();

	if (cnt < 15)
	{
		cnt++;
		sum_RollMiddle += Roll;
		sum_PitchMiddle += Pitch;

		sum_aux1Middle += aux1;
		sum_aux2Middle += aux2;
		sum_aux3Middle += aux3;

	}
	if (cnt == 15)
	{
		RollMiddle = (u16)(sum_RollMiddle / 15.0);
		PitchMiddle = (u16)(sum_PitchMiddle / 15.0);
		aux1Middle = (u16)(sum_aux1Middle / 15.0);
		aux2Middle = (u16)(sum_aux2Middle / 15.0);
		aux3Middle = (u16)(sum_aux3Middle / 15.0);
		autoMiddleFlag = 1;
	}


	if (autoMiddleFlag == 1)
	{
/*auto middle doesn't change pwm*/
//		if (throttle_vol > 2000) //左-上
//		{
//			gap = (uint16_t)(((float)(throttle_vol - 1966) / upRange) * 20.0);
//			throttle = throttle + (uint16_t)(((float)(throttle_vol - 2010) / upRange) * 20.0);
//			if (throttle > 999) throttle = 999;
//		}
//		else if (throttle_vol < 1900) //左-下
//		{
//			gap = (uint16_t)((float)(1966 - throttle_vol) / downRange * 25.0);
//			if (throttle < gap) throttle = 0;
//			else if (throttle > gap)
//			{
//				throttle = throttle - gap;
//			}
//		}
		throttle = (u16)(throttle_vol/4096.0*1000);
		rc_buf[0] = BYTE0( throttle);
		rc_buf[1] = BYTE1( throttle);

		if (Roll == RollMiddle)
		{
			rc_roll = 1500;
		}
		else if (Roll > RollMiddle)
		{
			rc_roll = (int16_t)(Roll - RollMiddle) * 1.0 / (4093 - RollMiddle) * 500.0 + 1500;
		}
		else if (Roll < RollMiddle)
		{
			rc_roll = (int16_t)(Roll * 1.0 / RollMiddle * 500.0) + 1000;
		}
		rc_buf[2] = BYTE0(rc_roll);
		rc_buf[3] = BYTE1(rc_roll);

		if (Pitch == PitchMiddle)
		{
			rc_pitch = 1500;
		}
		else if (Pitch > PitchMiddle)
		{
			rc_pitch = (int16_t)(Pitch - PitchMiddle) * 1.0 / (4093 - PitchMiddle) * 500.0 + 1500;
		}
		else if (Pitch < PitchMiddle)
		{
			rc_pitch = (int16_t)(Pitch * 1.0 / PitchMiddle * 500.0) + 1000;
		}
		rc_buf[4] = BYTE0(rc_pitch);
		rc_buf[5] = BYTE1(rc_pitch);


		rc_buf[6] = BYTE0(Yaw);
		rc_buf[7] = BYTE1(Yaw);
		
		rc_buf[8] = 'c';
		if (key == MODE_KEY_DOWN)
			rc_buf[8] = 'a';

		if (key == FUN_KEY_DOWN)
			rc_buf[8] = 'b';
		//aux1:start 1000 to 2000 middle is 1500.
		if (aux1 == aux1Middle)
		{
			rc_aux1 = 1500;
		}
		else if (aux1 > aux1Middle)
		{
			rc_aux1 = (int16_t)(aux1 - aux1Middle) * 1.0 / (4093 - aux1Middle) * 500.0 + 1500;
		}
		else if (aux1 < aux1Middle)
		{
			rc_aux1 = (int16_t)(aux1 * 1.0 / aux1Middle * 500.0) + 1000;
		}
		//aux2:start 1000 to 2000 middle is 1500.
		if (aux2 == aux2Middle)
		{
			rc_aux2 = 1500;
		}
		else if (aux2 > aux2Middle)
		{
			rc_aux2 = (int16_t)(aux2 - aux2Middle) * 1.0 / (4093 - aux2Middle) * 500.0 + 1500;
		}
		else if (aux2 < aux2Middle)
		{
			rc_aux2 = (int16_t)(aux2 * 1.0 / aux2Middle * 500.0) + 1000;
		}
		//aux3:start 1000 to 2000 middle is 1500.
		if (aux3 == aux3Middle)
		{
			rc_aux3 = 1500;
		}
		else if (aux3 > aux3Middle)
		{
			rc_aux3 = (int16_t)(aux3 - aux3Middle) * 1.0 / (4093 - aux3Middle) * 500.0 + 1500;
		}
		else if (aux3 < aux3Middle)
		{
			rc_aux3 = (int16_t)(aux3 * 1.0 / aux3Middle * 500.0) + 1000;
		}
		rc_buf[9] = BYTE0(rc_aux1);
		rc_buf[10] = BYTE1(rc_aux1);
		rc_buf[11] = BYTE0(rc_aux2);
		rc_buf[12] = BYTE1(rc_aux2);
		rc_buf[13] = BYTE0(rc_aux3);
		rc_buf[14] = BYTE1(rc_aux3);
		g_LoadRcReadyFlag = 1;
	}
}

