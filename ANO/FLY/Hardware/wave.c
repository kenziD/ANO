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
//	SPI1_SetSpeed(SPI_BaudRatePrescaler_8); //spiËÙ¶ÈÎª9Mhz£¨24L01µÄ×î´óSPIÊ±ÖÓÎª10Mhz£©
//	sta = NRF24L01_Read_Reg(STATUS); 
//	if (sta & RX_OK) 
//	{
//		//LED2_ON;
//		NRF24L01_Read_Buf(RD_RX_PLOAD, rc_tmp, RX_PLOAD_WIDTH); 
//		Rc_Data_Analyze(rc_tmp,&Rc_Data);
//		NRF24L01_Write_Reg(FLUSH_RX, 0xff); 
//	}
//	else
//	{
//		//LED2_OFF;
//	}
//	if(sta & MAX_TX)
//	{
//		NRF24L01_Write_Reg(FLUSH_TX, 0xff); 
//	}
//	NRF24L01_Write_Reg(WRITE_REG_NRF + STATUS, sta); 
}
extern int16_t motor0, motor1, motor2, motor3;
extern Int16xyz ACC_AVG;
extern Int16xyz AccFilterOut;
extern float _a0,_a1,_a2,_b0,_b1,_b2;
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
		send_senserV2(fACCEL_X, fACCEL_Y,fACCEL_Z, fGYRO_X, fGYRO_Y,fGYRO_Z, 0x00,0x00,0x00);
		send_wave(23);
		
		//send_senserV2(AccFilterOut.x, AccFilterOut.y,AccFilterOut.z, fGYRO_X, fGYRO_Y,fGYRO_Z, fACCEL_X,fACCEL_Y,fACCEL_Z);
		//send_wave(23);
		
	}
	else if(send_Status)
	{
		send_Status = 0;
		//sendPwmVoltage(&Rc_Data,(uint16_t)(motor0 / 1000.0 * 100), (uint16_t)(motor1 / 1000.0 * 100), (uint16_t)(motor2 / 1000.0 * 100), (uint16_t)(motor3 / 1000.0 * 100));//0.00003974s
		//send_wave(32);
		
		send_statusV2((int16_t)(outAngle.roll* 100),(int16_t)(outAngle.pitch* 100),(int16_t)(outAngle.yaw* 100),0x00,0x00,1);
		send_wave(18);
	}
	else if(send_RcData)
	{
		send_RcData = 0;
		send_rcdataV2(&Rc_Data);
		send_wave(25);
	}
	else if(send_PwmWave)
	{
		//Data_Send_PwmWave();
	}
	if(send_desirePIDAngle)
	{
		send_desirePIDAngle = 0;
		//Uart1_send_custom_three_int16((int16_t)(ypr[2]),(int16_t)(ypr[1]),(int16_t)(ypr[0]));
		//send_wave(10);
		//给第3帧 第1,2,3位 发送float数据
		//Uart1_send_custom_float(0xA3,expRoll,expPitch,(Rc_Data.aux3-2046)/1024.0);

		//Uart1_send_custom_float(0xA3,desireAngle.roll,desireAngle.pitch,(Rc_Data.aux3-2046)/1024.0);
		//send_wave(16);
		
		//send three float
		//Uart1_send_custom_float_V2(0xf1,desireAngle.roll,desireAngle.pitch,0);
		//send_wave(17);
		
		//send six float check filter param
		//send_custom_float_V2_6(0xf2,_a0,_a1,_a2,_b0,_b1,_b2);
		//send_wave(29);
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

/****************给第一帧 第一位 发送uint16_t数据*************/
void Uart1_send_custom_uint16(uint16_t aa)
{
	
	unsigned char sum = 0;
	count=0;

	sum +=putChar(0x88);
	sum +=putChar(0xA1);
	
	sum +=putChar(0x02);//发送的数据的长度 记得改
	sum +=Uart1_Put_UInt16(aa);//发送16位数据 
 
	putChar(sum);
}
/****************给第一帧 第一位 发送int16_t数据*************/
void Uart1_send_custom_int16(int16_t aa)
{
	unsigned char sum = 0;
	count=0;

	sum +=putChar(0x88);
	sum +=putChar(0xA1);
	
	sum +=putChar(0x02);//发送的数据的长度 记得改
	sum +=Uart1_Put_Int16(aa);//发送16位数据  
	putChar(sum);
}
/****************给第一帧 第1,2,3位 发送int16_t数据*************/
void Uart1_send_custom_three_int16(int16_t aa,int16_t bb,int16_t cc)
{
	unsigned char sum = 0;
	count=0;

	sum +=putChar(0x88);
	sum +=putChar(0xA1);
	
	sum +=putChar(0x06);//发送的数据的长度 记得改
	sum +=Uart1_Put_Int16(aa);//发送16位数据
	sum +=Uart1_Put_Int16(bb);//发送16位数据  
	sum +=Uart1_Put_Int16(cc);//发送16位数据    
	putChar(sum);
}
/****************给第2帧 第1,2,3,4位 发送int16_t数据*************/
void zhen2_send_custom_four_int16(int16_t aa,int16_t bb,int16_t cc,int16_t dd)
{
	unsigned char sum = 0;
	count=0;

	sum +=putChar(0x88);
	sum +=putChar(0xA2);
	
	sum +=putChar(0x08);//发送的数据的长度 记得改
	sum +=Uart1_Put_Int16(aa);//发送16位数据
	sum +=Uart1_Put_Int16(bb);//发送16位数据  
	sum +=Uart1_Put_Int16(cc);//发送16位数据  
	sum +=Uart1_Put_Int16(dd);//发送16位数据      
	putChar(sum);
}
/****************给第X帧 第X位 发送float数据*************/

void Uart1_send_custom_float(unsigned char fun,float aa,float bb,float cc)
{
	unsigned char sum = 0;
	count=0;

	sum +=putChar(0x88);
	sum +=putChar(fun);
	
	sum +=putChar(0x0c);//3个float占12个字节
	
	sum +=Uart1_Put_float(aa);//发送16位数据 
	sum +=Uart1_Put_float(bb);
	sum +=Uart1_Put_float(cc);

	putChar(sum);
}

void Uart1_send_custom_float_V2(unsigned char fun,float aa,float bb,float cc)
{
	unsigned char sum = 0;
	count=0;

	sum +=putChar(0xAA);
	sum +=putChar(0xAA);
	sum +=putChar(fun);
	
	sum +=putChar(0x0c);//3个float占12个字节
	
	sum +=Uart1_Put_float(aa);//发送16位数据 
	sum +=Uart1_Put_float(bb);
	sum +=Uart1_Put_float(cc);

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
void Uart1_send_custom_PID(uint8_t aa)
{
	unsigned char sum = 0;
	count=0;

	sum +=putChar(0x88);
	sum +=putChar(0xA1);
	
	sum +=putChar(0x01);//PID数据占32个字节 分32次发送每次发送一个字节。循环32次喽。
	
	sum +=putChar(aa);

	putChar(sum);
}
void sendSenser(int16_t aa,int16_t bb,int16_t cc,int16_t dd,int16_t ee,int16_t ff,int16_t roll,int16_t pitch,int16_t yaw)
{
	unsigned char sum = 0;
	count=0;
	sum += putChar(0x88);
	sum += putChar(0xAF);
	sum += putChar(0x1C);
	sum += putChar(BYTE1(aa));//1
	sum += putChar(BYTE0(aa));
	sum += putChar(BYTE1(bb));//2
	sum += putChar(BYTE0(bb));
	sum += putChar(BYTE1(cc));//3 ACC DATA
	sum += putChar(BYTE0(cc));
	sum += putChar(BYTE1(dd));//4
	sum += putChar(BYTE0(dd));
	sum += putChar(BYTE1(ee));//5
	sum += putChar(BYTE0(ee));
	sum += putChar(BYTE1(ff));//6 GYRO DATA
	sum += putChar(BYTE0(ff));
	putChar(0);
	putChar(0);
	putChar(0);
	putChar(0);
	putChar(0);
	putChar(0);//磁力计
	sum += putChar(BYTE1(roll));//7 ANGLE DATA ROLL
	sum += putChar(BYTE0(roll));
	sum += putChar(BYTE1(pitch));//8 PITCH
	sum += putChar(BYTE0(pitch));
	sum += putChar(BYTE1(yaw));//9 YAW
	sum += putChar(BYTE0(yaw));
	
	putChar(0);
	putChar(0);
	putChar(0);
	putChar(0);
	putChar(sum);
}
void sendPwmVoltage(Define_Rc_Data *rc_data,uint16_t aa,uint16_t bb,uint16_t cc,uint16_t dd)
{
	unsigned char sum = 0;
	float voltage_temp = ADC_ConvertedValue*2*3.3/4096.0;//四轴电压

	u16 v_100 = voltage_temp*100;
	u16 pwm_vol = voltage_temp*100*rc_data->throttle/999.0;
	count=0;

	sum += putChar(0x88);
	sum += putChar(0xAE);
	sum += putChar(0x1C);
	
	sum += putChar(BYTE1(rc_data->throttle));
	sum += putChar(BYTE0(rc_data->throttle));//throttle
	sum += putChar(BYTE1(rc_data->yaw));
	sum += putChar(BYTE0(rc_data->yaw));//yaw
	sum += putChar(BYTE1(rc_data->roll));
	sum += putChar(BYTE0(rc_data->roll));//roll
	sum += putChar(BYTE1(rc_data->pitch));
	sum += putChar(BYTE0(rc_data->pitch));//pitch
	sum += putChar(BYTE1(rc_data->aux1));
	sum += putChar(BYTE0(rc_data->aux1));//aux1
	sum += putChar(BYTE1(rc_data->aux2));
	sum += putChar(BYTE0(rc_data->aux2));//aux2
	sum += putChar(BYTE1(rc_data->aux3));
	sum += putChar(BYTE0(rc_data->aux3));//aux3
	sum += putChar(BYTE1(v_100));	//电池
	sum += putChar(BYTE0(v_100));//aux 4
	putChar(0);
	putChar(0);//aux 5
	
	sum += putChar(BYTE1(aa));//PWM 1
	sum += putChar(BYTE0(aa));
	sum += putChar(BYTE1(bb));//PWM 2
	sum += putChar(BYTE0(bb));
	sum += putChar(BYTE1(cc));//PWM 3
	sum += putChar(BYTE0(cc));
	sum += putChar(BYTE1(dd));//PWM 4
	sum += putChar(BYTE0(dd));
	sum += putChar(BYTE1(pwm_vol));//VOLTAGE
	sum += putChar(BYTE0(pwm_vol));
	putChar(sum);
}


void Uart1_Send_PID(uint16_t rol_p,uint16_t rol_i,uint16_t rol_d,uint16_t pit_p,uint16_t pit_i,uint16_t pit_d)
{
	unsigned char sum = 0;
	count=0;
	sum += putChar(0x88);
	sum += putChar(0xAC);
	sum += putChar(0x1C);
	sum += putChar(0xAD);
	
	sum += putChar(BYTE1(rol_p));//
	sum += putChar(BYTE0(rol_p));
	sum += putChar(BYTE1(rol_i));//
	sum += putChar(BYTE0(rol_i));
	sum += putChar(BYTE1(rol_d));//
	sum += putChar(BYTE0(rol_d));

	sum += putChar(BYTE1(pit_p));//
	sum += putChar(BYTE0(pit_p));
	sum += putChar(BYTE1(pit_i));//
	sum += putChar(BYTE0(pit_i));
	sum += putChar(BYTE1(pit_d));//
	sum += putChar(BYTE0(pit_d));

  putChar(0);
	putChar(0);//yaw_p
	putChar(0);
	putChar(0);//yaw_i
	putChar(0);
	putChar(0);//yaw_d
	
	putChar(0);
	putChar(0);
	putChar(0);
	putChar(0);
	putChar(0);
	putChar(0);
	putChar(0);
	putChar(0);
	putChar(0);
	
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

/*************************接收********************/

void receive_Data(void)
{
	u8 sum = 0;
	u8 i=0;
	for(i=0;i<31;i++)
		sum += Res[i];
	if(!(sum==Res[31]))		return;		//ÅÐ¶Ïsum
	if(!(Res[0]==0x8A))		return;		//ÅÐ¶ÏÖ¡Í·
	if(Res[1]==0X8B&&Res[2]==0x1C)								//判断功能字,=0x8B,为控制数据
	{
		if(Res[3]==0XAD)//发送PID
		{
			Uart1_Send_PID((uint16_t)(PID_ROLL.KP*100),(uint16_t)(PID_ROLL.KI*100),(uint16_t)(PID_ROLL.KD*100),(uint16_t)(PID_PITCH.KP*100),(uint16_t)(PID_PITCH.KI*100),(uint16_t)(PID_PITCH.KD*100));
			//send_wave(32);
		}
		if(Res[3]==0XAE)//接收PID
		{
			PID_ROLL.KP=(float)(Res[4]<<8|Res[5])/100.0;
			PID_ROLL.KI=(float)(Res[6]<<8|Res[7])/100.0;
			PID_ROLL.KD=(float)(Res[8]<<8|Res[9])/100.0;
				
			PID_PITCH.KP=(float)(Res[10]<<8|Res[11])/100.0;
			PID_PITCH.KI=(float)(Res[12]<<8|Res[13])/100.0;
			PID_PITCH.KD=(float)(Res[14]<<8|Res[15])/100.0;
		}
	}
}

/*************************sendData********************/

/////////////////////////////////////////////////////////////version2////////////////////////////////////////////////
void send_statusV2(int16_t rol, int16_t pitch, int16_t yaw, int16_t alt_cbs, int32_t alt_prs, u8 armed) {
	unsigned char sum = 0;
	count=0;
	sum += putChar(0xAA);
	sum += putChar(0xAA);
	sum += putChar(0x01);
	
	sum += putChar(0x0D);
	//13 BYTE not including 0xAA 0xAA 0X01 self and sum.
	
//	sum += putChar(BYTE1(rol));
//	sum += putChar(BYTE0(rol));
//	sum += putChar(BYTE1(pitch));
//	sum += putChar(BYTE0(pitch));
//	sum += putChar(BYTE1(yaw));
//	sum += putChar(BYTE0(yaw));
//	sum += putChar(BYTE1(alt_cbs));
//	sum += putChar(BYTE0(alt_cbs));
//	sum += putChar(BYTE3(alt_prs));
//	sum += putChar(BYTE2(alt_prs));
//	sum += putChar(BYTE1(alt_prs));
//	sum += putChar(BYTE0(alt_prs));
	
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
//	sum += putChar(BYTE1(acc_x));//PWM 1
//	sum += putChar(BYTE0(acc_x));
//	sum += putChar(BYTE1(acc_y));//PWM 1
//	sum += putChar(BYTE0(acc_y));
//	sum += putChar(BYTE1(acc_z));//PWM 1
//	sum += putChar(BYTE0(acc_z));
//	sum += putChar(BYTE1(gyro_x));
//	sum += putChar(BYTE0(gyro_x));//PWM 1
//	sum += putChar(BYTE1(gyro_y));
//	sum += putChar(BYTE0(gyro_y));//PWM 1
//	sum += putChar(BYTE1(gyro_z));
//	sum += putChar(BYTE0(gyro_z));//PWM 1
//	sum += putChar(BYTE1(mag_x));
//	sum += putChar(BYTE0(mag_x));//PWM 1
//	sum += putChar(BYTE1(mag_y));
//	sum += putChar(BYTE0(mag_y));//PWM 1
//	sum += putChar(BYTE1(mag_z));
//	sum += putChar(BYTE0(mag_z));
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
