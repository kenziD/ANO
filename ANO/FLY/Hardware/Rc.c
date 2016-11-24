#include "Rc.h"
#include "led.h"
////////////////////////////////
//Control Receiver Analyzer
////////////////////////////////
extern float pitch_offset;
extern float roll_offset;
extern float yawOffsetCache,pitchOffsetCache,rollOffsetCache;
void Rc_Data_Analyze(u8 *rcDataBuf,Define_Rc_Data *rc_data)
{
	static u16 startCNT = 0;
	static u16 endCNT = 0;	
	static u8 led_on = 0;
		 
//	if(led_on)
//       {
//        LED2_OFF;
//        led_on = 0;
//       }
//       else
//       {
//        LED2_ON;
//       led_on = 1;
//       }
	//rc_data->throttle = (int)((rcDataBuf[1] << 8 | rcDataBuf[0])/1000.0*999);//0~999
	rc_data->throttle = (int)(rcDataBuf[1] << 8 | rcDataBuf[0]);//0~999
	rc_data->roll = ((rcDataBuf[3] << 8 | rcDataBuf[2]) - 1970) / 70.0;
	rc_data->pitch = ((rcDataBuf[5] << 8 | rcDataBuf[4]) - 2022) / 70.0;
	rc_data->yaw = rcDataBuf[7] << 8 | rcDataBuf[6];
	if(rc_data->throttle<200 && rc_data->yaw<1000){
		
		if(startCNT==500)
		{
			strcpy(rc_data->status, "start");
			LED3_ON;
			startCNT = 0;
		}
		else
		{
			startCNT++;
		}
		
	}
	else
	{
		startCNT = 0;
	}
	if(rc_data->throttle<200 && rc_data->yaw>3000)
	{
		if(endCNT==500)
		{
			strcpy(rc_data->status, "stop");
			LED3_OFF;
			endCNT = 0;
		}
		else
		{
			endCNT++;
		}
	}
	else
	{
		endCNT = 0;
	}
	if (rcDataBuf[8] == 'a')
	{
		pitch_offset = pitchOffsetCache;
		roll_offset = rollOffsetCache;
	}
	if (rcDataBuf[8] == 'b')
	{
		pitch_offset = 0;
		roll_offset = 0;
	}
}
