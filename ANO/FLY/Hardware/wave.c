#include "wave.h"
#include "usart.h"
#include "config.h"
#include "pid.h"
#include "MPU6050.h"
#include "24l01.h"
#include "Rc.h"
//#define tx_num	32
unsigned char temp[6];
unsigned char TxBuffer[32];//一共发送的字节数 记得改
unsigned char count=0; 
extern PID_ PID_ROLL,PID_PITCH;
extern uint8_t Res[32];
extern float ypr[3];
#define BYTE0(dwTemp)       (*(char *)(&dwTemp))
#define BYTE1(dwTemp)       (*((char *)(&dwTemp) + 1))
#define BYTE2(dwTemp)       (*((char *)(&dwTemp) + 2))
#define BYTE3(dwTemp)       (*((char *)(&dwTemp) + 3))
Define_Rc_Data Rc_Data = {0,0,0,0, 0 ,0,0,0};
extern floatEurlaAngle outAngle;;
u8 rc_tmp[32] = {0};
extern u8 send_Senser;
extern u8 send_Status;
extern u8 send_RcData;
extern u8 send_PwmWave;
extern u8 send_desirePIDAngle;
extern float expRoll;
extern float expPitch;
extern float expYaw;
extern floatEurlaAngle desireAngle;
void NRF_Check()
{
	static u8 led_on = 0;
	u8 sta = 0;
	if (NRF24L01_RxPacket(rc_tmp) == 0)
	{
//		if(led_on)
//    {
//       LED2_OFF;
//       led_on = 0;
//    }
//    else
//    {
//       LED2_ON;
//       led_on = 1;
//    }
		//10us
		//LED2_ON;
		Rc_Data_Analyze(rc_tmp,&Rc_Data);
	}
	else
	{
		//LED2_OFF;
	}
}
extern int16_t motor0, motor1, motor2, motor3;
extern Int16xyz ACC_AVG;
extern Int16xyz AccFilterOut;
extern float _a0,_a1,_a2,_b0,_b1,_b2;
extern int16_t fACCEL_X_6Cali,fACCEL_Y_6Cali , fACCEL_Z_6Cali; //use 6 position static.use bias,Sx,Sy,Sz.
extern int16_t fACCEL_X_noOffset , fACCEL_Y_noOffset , fACCEL_Z_noOffset ;//raw Mpu6050 register data
extern int16_t fACCEL_X_zhihu , fACCEL_Y_zhihu , fACCEL_Z_zhihu;//use matlab  lsqcurvefit
extern int16_t fACCEL_X_zhihu_pix , fACCEL_Y_zhihu_pix , fACCEL_Z_zhihu_pix ; //seem as six position bias and lsqcurvefit combination.but not the same.they consider the orthangal

extern floatEurlaAngle accOutAngle_offset;
extern floatEurlaAngle gyroOutAngle;
u8 getTX_FIFO_status()
{
	SPI1_SetSpeed(SPI_BaudRatePrescaler_8);
	return NRF24L01_Read_Reg(NRF_FIFO_STATUS);
}
void Data_Transfer()
{
	u8 FIFOstatus = 0x10;
	NRF_Check();
	//FIFOstatus = getTX_FIFO_status();
	//if TX_FIFO is not empty,the last time transfer didn't receive an acknowlege.return.
	//if((FIFOstatus & (1<<4))==0)
	//{
	//	return;
	//}
	if(send_Senser)
	{
		send_Senser = 0;
		//send acc data after average filter
		//sendSenser(ACC_AVG.x, ACC_AVG.y,ACC_AVG.z, fGYRO_X,  fGYRO_Y,fGYRO_Z, (int16_t)(ypr[2] * 100), (int16_t)(ypr[1] * 100),(int16_t)(ypr[0] * 10));
		
		//sendSenser(fACCEL_X, fACCEL_Y,fACCEL_Z, fGYRO_X,  fGYRO_Y,fGYRO_Z, (int16_t)(outAngle.roll* 100), (int16_t)(outAngle.pitch* 100),(int16_t)(outAngle.yaw* 10));
		//send_wave(32);

		//Version2
		//send_senserV2(fACCEL_X, fACCEL_Y,fACCEL_Z,fACCEL_X_6Cali,fACCEL_Y_6Cali,fACCEL_Z_6Cali,fACCEL_X_zhihu_pix, fACCEL_Y_zhihu_pix,fACCEL_Z_zhihu_pix);
		//send_wave(23);
		
		send_senserV2(fACCEL_X,fACCEL_Y,fACCEL_Z,fGYRO_X, fGYRO_Y,fGYRO_Z, 0x00,0x00,0x00);
		send_wave(23);
		
	}
	else if(send_Status)
	{
		send_Status = 0;
		//sendPwmVoltage(&Rc_Data,(uint16_t)(motor0 / 1000.0 * 100), (uint16_t)(motor1 / 1000.0 * 100), (uint16_t)(motor2 / 1000.0 * 100), (uint16_t)(motor3 / 1000.0 * 100));//0.00003974s
		//send_wave(32);
		send_statusV2((int16_t)(outAngle.roll* 100),(int16_t)(outAngle.pitch* 100),(int16_t)(outAngle.yaw* 100),0x00,0x00,1);
		send_wave(18);
		//send_statusV2((int16_t)(accOutAngle_offset.roll* 100),(int16_t)(accOutAngle_offset.pitch* 100),0x00,0x00,0x00,1);
		//send_wave(18);
		
	}
	else if(send_RcData)
	{
		send_RcData = 0;
		//Uart1_send_custom_int16_V2(0xf6,fACCEL_X_noOffset,fACCEL_Y_noOffset,fACCEL_Z_noOffset);
		//send_wave(11);
		//Uart1_send_custom_int16_V2(0xf7,fACCEL_X,fACCEL_Y,fACCEL_Z);
		//send_wave(11);
	}
	else if(send_PwmWave)
	{
		//Data_Send_PwmWave();
		
		
	}
	else if(send_desirePIDAngle)
	{
		send_desirePIDAngle = 0;
    //Uart1_send_custom_float_V2_2(0xf1,accOutAngle_NOoffset.roll,accOutAngle_NOoffset.pitch);
    //send_wave(13);
		Uart1_send_custom_int16_V2_4(0xf1,(int16_t)(accOutAngle_offset.roll*100),(int16_t)(gyroOutAngle.roll*100),(int16_t)(outAngle.roll* 100),0x00);
		send_wave(13);
		//Uart1_send_custom_float_V2_2(0xf2,accOutAngle_offset.roll,accOutAngle_offset.pitch);
		//send_wave(13);
		//Uart1_send_custom_float_V2_2(0xf3,accOutAngle_bias.roll,accOutAngle_bias.pitch);
		//send_wave(13);
		//Uart1_send_custom_float_V2_2(0xf4,accOutAngle_zhihu.roll,accOutAngle_zhihu.pitch);
		//send_wave(13);
		//Uart1_send_custom_float_V2_2(0xf5,accOutAngle_pix.roll,accOutAngle_pix.pitch);
		//send_wave(13);
	}
}
/**************************向物理串口发一个字节***************************************
*******************************************************************************/
__inline unsigned char UART_Putc(unsigned char data)			//
{
	USART_SendData(USART1,  (uint8_t)data);
	while (USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET);
	return data;
}


unsigned char putChar(unsigned char DataToSend)
{
	TxBuffer[count++] = DataToSend;  
	return DataToSend;
}

unsigned char Uart1_Put_UInt16(uint16_t DataToSend)
{
	unsigned char sum = 0;
	TxBuffer[count++] = BYTE1(DataToSend);
	TxBuffer[count++] = BYTE0(DataToSend);
	sum += BYTE1(DataToSend);
	sum += BYTE0(DataToSend);
	return sum;
}
unsigned char Uart1_Put_Int16(int16_t DataToSend)
{
	unsigned char sum = 0;
	TxBuffer[count++] = BYTE1(DataToSend);
	TxBuffer[count++] = BYTE0(DataToSend);
	sum += BYTE1(DataToSend);
	sum += BYTE0(DataToSend);
	return sum;
}
//arg type defination is important 0.^
unsigned char Uart1_Put_Int32(int32_t DataToSend)
{
	unsigned char sum = 0;
	TxBuffer[count++] = BYTE3(DataToSend);
	TxBuffer[count++] = BYTE2(DataToSend);
	TxBuffer[count++] = BYTE1(DataToSend);
	TxBuffer[count++] = BYTE0(DataToSend);
	sum += BYTE1(DataToSend);
	sum += BYTE0(DataToSend);
	sum += BYTE2(DataToSend);
	sum += BYTE3(DataToSend);
	return sum;
}
unsigned char Uart1_Put_float(float DataToSend)
{
	unsigned char sum = 0;
	TxBuffer[count++] = BYTE3(DataToSend);
	TxBuffer[count++] = BYTE2(DataToSend);
	TxBuffer[count++] = BYTE1(DataToSend);
	TxBuffer[count++] = BYTE0(DataToSend);
	
	sum += BYTE3(DataToSend);
	sum += BYTE2(DataToSend);
	sum += BYTE1(DataToSend);
	sum += BYTE0(DataToSend);
	
	return sum;
}


void Uart1_send_custom_int16_V2(unsigned char fun,int16_t aa,int16_t bb,int16_t cc)
{
	unsigned char sum = 0;
	count=0;

	sum +=putChar(0xAA);
	sum +=putChar(0xAA);
	sum +=putChar(fun);
	
	sum +=putChar(0x06);//发送的数据的长度 记得改
	sum +=Uart1_Put_Int16(aa);//发送16位数据
	sum +=Uart1_Put_Int16(bb);//发送16位数据  
	sum +=Uart1_Put_Int16(cc);//发送16位数据  
	putChar(sum);
}
void Uart1_send_custom_int16_V2_4(unsigned char fun,int16_t aa,int16_t bb,int16_t cc,int16_t dd)
{
	unsigned char sum = 0;
	count=0;

	sum +=putChar(0xAA);
	sum +=putChar(0xAA);
	sum +=putChar(fun);
	
	sum +=putChar(0x08);//发送的数据的长度 记得改
	sum +=Uart1_Put_Int16(aa);//发送16位数据
	sum +=Uart1_Put_Int16(bb);//发送16位数据  
	sum +=Uart1_Put_Int16(cc);//发送16位数据 
	sum +=Uart1_Put_Int16(dd);//发送16位数据  	
	putChar(sum);
}
void Uart1_send_custom_int16_V2_6(unsigned char fun,int16_t aa,int16_t bb,int16_t cc,int16_t dd,int16_t ee,int16_t ff)
{
	unsigned char sum = 0;
	count=0;

	sum +=putChar(0xAA);
	sum +=putChar(0xAA);
	sum +=putChar(fun);
	
	sum +=putChar(0x12);//发送的数据的长度 记得改
	sum +=Uart1_Put_Int16(aa);//发送16位数据
	sum +=Uart1_Put_Int16(bb);//发送16位数据  
	sum +=Uart1_Put_Int16(cc);//发送16位数据  
	sum +=Uart1_Put_Int16(dd);//发送16位数据
	sum +=Uart1_Put_Int16(ee);//发送16位数据  
	sum +=Uart1_Put_Int16(ff);//发送16位数据  
	putChar(sum);
}

void Uart1_send_custom_float_V2_2(unsigned char fun,float aa,float bb)
{
	unsigned char sum = 0;
	count=0;

	sum +=putChar(0xAA);
	sum +=putChar(0xAA);
	sum +=putChar(fun);
	
	sum +=putChar(0x08);//2个float占8个字节
	
	sum +=Uart1_Put_float(aa);//发送16位数据 
	sum +=Uart1_Put_float(bb);
	
	putChar(sum);
}

void send_custom_float_V2_6(unsigned char fun,float aa,float bb,float cc,float dd,float ee,float ff)
{
	unsigned char sum = 0;
	count=0;

	sum +=putChar(0xAA);
	sum +=putChar(0xAA);
	sum +=putChar(fun);
	
	sum +=putChar(0x18);//6个float占24个字节
	
	sum +=Uart1_Put_float(aa);//发送16位数据 
	sum +=Uart1_Put_float(bb);
	sum +=Uart1_Put_float(cc);
	sum +=Uart1_Put_float(dd);
	sum +=Uart1_Put_float(ee);
	sum +=Uart1_Put_float(ff);

	putChar(sum);
}

void send_custom_float_V2_4(unsigned char fun,float aa,float bb,float cc,float dd)
{
	unsigned char sum = 0;
	count=0;

	sum +=putChar(0xAA);
	sum +=putChar(0xAA);
	sum +=putChar(fun);
	
	sum +=putChar(0x10);//4个float共16个字节
	
	sum +=Uart1_Put_float(aa);//发送16位数据 
	sum +=Uart1_Put_float(bb);
	sum +=Uart1_Put_float(cc);
	sum +=Uart1_Put_float(dd);

	putChar(sum);
}


void send_wave(int tx_num)//一共发送几个字节,如果是传送到NRF24L01会有应答ack。将应答结果写入rxbuf。这里是利用了应答形成了双向通讯。主函数里会对应答数据进行分析。主要是遥控数据。
{	
	char count_1=0;
	#ifdef DATA_TRANSFER_USE_SPI_NRF
		int send_result = NRF24L01_TxPacket(TxBuffer,tx_num);
		if(send_result == RX_OK)
		{
			//LED2_ON;
		}
		else
		{
			//LED2_OFF;
		}
	#endif
		
	#ifdef DATA_TRANSFER_USE_USART
		while(count_1<tx_num)
	      UART_Putc(TxBuffer[count_1++]);
	#endif
}


/////////////////////////////////////////////////////////////version2////////////////////////////////////////////////
void send_statusV2(int16_t rol, int16_t pitch, int16_t yaw, int16_t alt_cbs, int32_t alt_prs, u8 armed) {
	unsigned char sum = 0;
	count=0;
	sum += putChar(0xAA);
	sum += putChar(0xAA);
	sum += putChar(0x01);
	
	sum += putChar(0x0D);
	//13 BYTE not including 0xAA 0xAA 0X01 self and sum.
	
	sum += Uart1_Put_Int16(rol);
	sum += Uart1_Put_Int16(pitch);
	sum += Uart1_Put_Int16(yaw);
	sum += Uart1_Put_Int16(alt_cbs);
	sum += Uart1_Put_Int32(alt_prs);

	if (armed) {
		sum += putChar(0xA0);
	}
	else {
		sum += putChar(0xA1);
	}
	putChar(sum);
}
void send_senserV2(int16_t acc_x, int16_t acc_y, int16_t acc_z, int16_t gyro_x, int16_t gyro_y, int16_t gyro_z, int16_t mag_x, int16_t mag_y, int16_t mag_z) {
	unsigned char sum = 0;
	count=0;
	sum += putChar(0xAA);
	sum += putChar(0xAA);
	sum += putChar(0x02);
	//18 BYTE
	sum += putChar(0x12);
	sum += Uart1_Put_Int16(acc_x);
	sum += Uart1_Put_Int16(acc_y);
	sum += Uart1_Put_Int16(acc_z);
	sum += Uart1_Put_Int16(gyro_x);
	sum += Uart1_Put_Int16(gyro_y);
	sum += Uart1_Put_Int16(gyro_z);
	sum += Uart1_Put_Int16(mag_x);
	sum += Uart1_Put_Int16(mag_y);
	sum += Uart1_Put_Int16(mag_z);

	putChar(sum);
}
void send_rcdataV2(Define_Rc_Data *rc_data) 
{
	unsigned char sum = 0;

	float voltage_temp = ADC_ConvertedValue*2*3.3/4096.0;//????
	int16_t v_100 = voltage_temp*100;
//	u16 pwm_vol = voltage_temp*100*rc_data->throttle/999.0;
	count=0;
	sum += putChar(0xAA);
	sum += putChar(0xAA);
	sum += putChar(0x03);
	//20 BYTE
	sum += putChar(0x14);
	 sum += Uart1_Put_Int16((int16_t)rc_data->throttle);
	//sum += Uart1_Put_Int16(1500);
	 sum += Uart1_Put_Int16((int16_t)rc_data->yaw);
	//sum += Uart1_Put_Int16(1500);
	 sum += Uart1_Put_Int16((int16_t)rc_data->roll);
	//sum += Uart1_Put_Int16(1500);
	 sum += Uart1_Put_Int16((int16_t)rc_data->pitch);
	//sum += Uart1_Put_Int16(1500);
	 sum += Uart1_Put_Int16((int16_t)rc_data->aux1);
	//sum += Uart1_Put_Int16(1500);
	 sum += Uart1_Put_Int16((int16_t)rc_data->aux2);
	//sum += Uart1_Put_Int16(1500);
	 sum += Uart1_Put_Int16((int16_t)rc_data->aux3);
	//sum += Uart1_Put_Int16(1500);
	 sum += Uart1_Put_Int16(v_100);//aux4
	//sum += Uart1_Put_Int16(1500);
	sum += Uart1_Put_Int16(0);//aux5
	sum += Uart1_Put_Int16(0);//aux6
	
	putChar(sum);
}
