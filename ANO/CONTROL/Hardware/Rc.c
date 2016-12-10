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
void autoMiddle(void)
{
	throttleMiddle = voltage1();
}
void loadRcData()
{
	static u16 throttle_vol =0;
	static u16 throttle=0;
	static u16 gap;
	static u16 Roll = 0;
	static u16 Pitch = 0;
	static u16 Yaw = 0;
	static u16 aux1 = 0;
	static u16 aux2 = 0;
	static u16 aux3 = 0;
	static u8 middle = 1;
	throttle_vol = voltage1();
		if(throttle_vol>2000)//左-上
		{
			gap = (uint16_t)(((float)(throttle_vol-1966)/upRange)*20.0);
			throttle = throttle + (uint16_t)(((float)(throttle_vol-2010)/upRange)*20.0);
			if(throttle>999) throttle=999;
		}
		else if (throttle_vol<1900) //左-下
		{
			gap = (uint16_t)((float)(1966-throttle_vol)/downRange*25.0);
			if(throttle<gap) throttle = 0;
			else if(throttle>gap) 
			{
				throttle = throttle - gap;
			}
		}
//		if(throttle_vol>3000 )//左-上
//		{
//			if(middle==1)
//			{
//				middle=0;
//				if(throttle>999) throttle=999;
//			else if(throttle<(999-50) )
//			{
//					throttle = throttle + 50;
//				
//			}
//			}
//			
//		
//		}
//		else if (throttle_vol<1800) //左-下
//		{
//			if(middle==1)
//			{
//							middle = 0;
//	
//				if(throttle<=0)
//				{
//					throttle = 0;
//				}
//				else if(throttle>50 )
//				{
//					throttle = throttle - 50;
//					
//				}
//			}

//				
//				
//		}else
//		{middle =1;}
			Yaw = voltage2();
			Roll = voltage3();
			Pitch = voltage4();
			aux1 = voltage7();
			aux2 = voltage5();
			aux3 = voltage6();
      rc_buf[0] = BYTE0( throttle);
      rc_buf[1] = BYTE1( throttle);
			rc_buf[2] = BYTE0(Roll);
      rc_buf[3] = BYTE1(Roll);
			rc_buf[4] = BYTE0(Pitch);
			rc_buf[5] = BYTE1(Pitch);
			rc_buf[6] = BYTE0(Yaw);
      rc_buf[7] = BYTE1(Yaw);
			rc_buf[8] = 'c';
			if(key == MODE_KEY_DOWN)
				rc_buf[8] = 'a';
				
			if(key == FUN_KEY_DOWN)
				rc_buf[8] = 'b';
			rc_buf[9] = BYTE0(aux1);
      rc_buf[10] = BYTE1(aux1);
			rc_buf[11] = BYTE0(aux2);
      rc_buf[12] = BYTE1(aux2);
			rc_buf[13] = BYTE0(aux3);
			rc_buf[14] = BYTE1(aux3);
			g_LoadRcReadyFlag = 1;
}
