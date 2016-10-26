#include "wave.h"
#include "usart.h"
#include "config.h"
#include "pid.h"
#include "MPU6050.h"
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

unsigned char Uart1_Put_temp(unsigned char DataToSend)
{
	temp[count++] = DataToSend;  
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


void Uart1_send_temp(uint16_t aa)
{
	unsigned char sum = 0;
	count=0;
	sum +=Uart1_Put_temp(0x88);
	sum +=Uart1_Put_temp(0xA1);
	sum +=Uart1_Put_temp(0x02);//PID数据占32个字节 分32次发送每次发送一个字节。循环32次喽。
  sum +=Uart1_Put_temp(BYTE1(aa));
	sum +=Uart1_Put_temp(BYTE0(aa));
	Uart1_Put_temp(sum);
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
/****************给第一帧 第一位 发送float数据*************/

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
void sendSenser(int16_t aa,int16_t bb,int16_t cc,int16_t dd,int16_t ee,int16_t ff,int16_t gg,int16_t hh)
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
	sum += putChar(BYTE1(gg));//7 ANGLE DATA ROLL
	sum += putChar(BYTE0(gg));
	sum += putChar(BYTE1(hh));//8 PITCH
	sum += putChar(BYTE0(hh));
	putChar(0);//YAW
	putChar(0);
	
	putChar(0);
	putChar(0);
	putChar(0);
	putChar(0);
	putChar(sum);
}
void sendPwmVoltage(uint16_t aa,uint16_t bb,uint16_t cc,uint16_t dd,uint16_t ee)
{
	unsigned char sum = 0;
	count=0;
	sum += putChar(0x88);
	sum += putChar(0xAE);
	sum += putChar(0x1C);
	
	putChar(0);
	putChar(0);//throttle
	putChar(0);
	putChar(0);//yaw
	putChar(0);
	putChar(0);//roll
	putChar(0);
	putChar(0);//pitch
	putChar(0);
	putChar(0);//aux 1
	putChar(0);
	putChar(0);//aux 2
	putChar(0);
	putChar(0);//aux 3
	putChar(0);
	putChar(0);//aux 4
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
	sum += putChar(BYTE1(ee));//VOLTAGE
	sum += putChar(BYTE0(ee));


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
void send_wave(int tx_num)//一共发送几个字节
{	
	char count_1=0;
	#ifdef DATA_TRANSFER_USE_SPI_NRF
		if(NRF24L01_TxPacket(TxBuffer) == TX_OK)
		{
			 LED2_ON;
		}
		else
		{
			LED2_OFF;
		}
	#endif
	#ifdef DATA_TRANSFER_USE_USART
		while(count_1<tx_num)
	      UART_Putc(TxBuffer[count_1++]);
	#endif
	
	
}

void sendSenserPackage(void)
{
//		static u8 led_on = 0;
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
	sendSenser((int16_t)fACCEL_X, (int16_t)fACCEL_Y, (int16_t)fACCEL_Z, (int16_t)fGYRO_X, (int16_t) fGYRO_Y, (int16_t)fGYRO_Z, (int16_t)(ypr[2] * 100), (signed short int)(ypr[1] * 100));//0.00002419s
	send_wave(32);//0.00013279s
}
void send_temp(int tx_num)//一共发送几个字节
{
	char count_1=0;
	while(count_1<tx_num)
      UART_Putc(temp[count_1++]);
}
///////////////////////////////////////////////////////////////////////

void printhh(void)
  {
	UART_Putc(0x0d);       			 //output'CR'
	UART_Putc(0x0A);       			 //output'CR'
  } 
  
void print5n(unsigned int x)
  {	  
	   UART_Putc((x/10000)+0x30);           //计算万位数字
	   UART_Putc(((x%10000)/1000)+0x30);    //计算千位数字
	   UART_Putc(((x%1000)/100)+0x30);      //计算百位数字
	   UART_Putc(((x%100)/10)+0x30);        //计算十位数字
	   UART_Putc((x%10)+0x30);              //计算个位数字
  }
  
void print4n(unsigned int x)
  {	  
	  UART_Putc((x/1000)+0x30);    //计算千位数字
	  UART_Putc(((x%1000)/100)+0x30);      //计算百位数字
	  UART_Putc(((x%100)/10)+0x30);        //计算十位数字
	  UART_Putc((x%10)+0x30);              //计算个位数字
  }
void print3n(unsigned int x)
{
	UART_Putc((x/100)+0x30);      //计算百位数字
	UART_Putc(((x%100)/10)+0x30);        //计算十位数字
	UART_Putc((x%10)+0x30);   
}
void print2n(unsigned int x)
{
  
	UART_Putc((x/10)+0x30);        //计算十位数字
	UART_Putc((x%10)+0x30);   
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
			send_wave(32);
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


