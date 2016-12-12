#include "Rc.h"
#include "led.h"
////////////////////////////////
//Control Receiver Analyzer
////////////////////////////////
extern float pitch_offset;
extern float roll_offset;
extern float yawOffsetCache,pitchOffsetCache,rollOffsetCache;
extern float gCalibrate;

void Rc_Data_Analyze(u8 *rcDataBuf,Define_Rc_Data *rc_data)
{
	u8 num  =32;
	u8 sum = 0;
	static u16 startCNT = 0;
	static u16 endCNT = 0;	
	//for(u8 i=0;i<(num-1);i++)
		//sum += *(rcDataBuf+i);
	//if(!(sum==*(rcDataBuf+num-1)))		return;		//ÅÐ¶Ïsum
	//if(!(*(rcDataBuf)==0xAA && *(rcDataBuf+1)==0xAF))		return;		//ÅÐ¶ÏÖ¡Í·
//	static u8 led_on = 0;
	
	rc_data->throttle = (int)(rcDataBuf[1] << 8 | rcDataBuf[0]);//0~999
	rc_data->roll = rcDataBuf[3] << 8 | rcDataBuf[2];
	rc_data->pitch = rcDataBuf[5] << 8 | rcDataBuf[4];
	rc_data->yaw = rcDataBuf[7] << 8 | rcDataBuf[6];
	rc_data->aux1 = rcDataBuf[10]<<8|rcDataBuf[9];
	rc_data->aux2 = rcDataBuf[12]<<8|rcDataBuf[11];
	rc_data->aux3 = rcDataBuf[14]<<8|rcDataBuf[13];
	/*for motor test*/
	//if(rc_data->throttle<200 && rc_data->yaw<500){
	if(rc_data->start == 0&&rc_data->throttle<5){
		
		if(startCNT==100)
		{
			
			rc_data->start = 1;
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
	/*for motor test*/
	//if(rc_data->throttle<200 && rc_data->yaw>3000)
	if(rc_data->start == 1&&rc_data->throttle<5)
	{
		if(endCNT==100)
		{
			rc_data->start = 0;
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
		gCalibrate = 1;
	}
	if (rcDataBuf[8] == 'b')
	{
		pitch_offset = 0;
		roll_offset = 0;
	}
}

