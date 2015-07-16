#include "wave.h"
#include "usart.h"

//#define tx_num	32

unsigned char TxBuffer[32];//一共发送的字节数 记得改
unsigned char count=0; 

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

unsigned char Uart1_Put_Char(unsigned char DataToSend)
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

	sum +=Uart1_Put_Char(0x88);
	sum +=Uart1_Put_Char(0xA1);
	
	sum +=Uart1_Put_Char(0x02);//发送的数据的长度 记得改
	sum +=Uart1_Put_UInt16(aa);//发送16位数据 
 
	Uart1_Put_Char(sum);
}
/****************给第一帧 第一位 发送int16_t数据*************/
void Uart1_send_custom_int16(int16_t aa)
{
	unsigned char sum = 0;
	count=0;

	sum +=Uart1_Put_Char(0x88);
	sum +=Uart1_Put_Char(0xA1);
	
	sum +=Uart1_Put_Char(0x02);//发送的数据的长度 记得改
	sum +=Uart1_Put_Int16(aa);//发送16位数据 
 
	Uart1_Put_Char(sum);
}
/****************给第一帧 第一位 发送float数据*************/

void Uart1_send_custom_float(unsigned char fun,float aa,float bb,float cc)
{
	unsigned char sum = 0;
	count=0;

	sum +=Uart1_Put_Char(0x88);
	sum +=Uart1_Put_Char(fun);
	
	sum +=Uart1_Put_Char(0x0c);//3个float占12个字节
	
	sum +=Uart1_Put_float(aa);//发送16位数据 
	sum +=Uart1_Put_float(bb);
	sum +=Uart1_Put_float(cc);

	Uart1_Put_Char(sum);
}

void Uart1_Send_AF(int16_t aa,int16_t bb,int16_t cc,int16_t dd,int16_t ee,int16_t ff,int16_t gg,int16_t hh)
{
	unsigned char sum = 0;
	count=0;
	sum += Uart1_Put_Char(0x88);
	sum += Uart1_Put_Char(0xAF);
	sum += Uart1_Put_Char(0x1C);
	sum += Uart1_Put_Char(BYTE1(aa));//1
	sum += Uart1_Put_Char(BYTE0(aa));
	sum += Uart1_Put_Char(BYTE1(bb));//2
	sum += Uart1_Put_Char(BYTE0(bb));
	sum += Uart1_Put_Char(BYTE1(cc));//3 ACC DATA
	sum += Uart1_Put_Char(BYTE0(cc));
	sum += Uart1_Put_Char(BYTE1(dd));//4
	sum += Uart1_Put_Char(BYTE0(dd));
	sum += Uart1_Put_Char(BYTE1(ee));//5
	sum += Uart1_Put_Char(BYTE0(ee));
	sum += Uart1_Put_Char(BYTE1(ff));//6 GYRO DATA
	sum += Uart1_Put_Char(BYTE0(ff));
	Uart1_Put_Char(0);
	Uart1_Put_Char(0);
	Uart1_Put_Char(0);
	Uart1_Put_Char(0);
	Uart1_Put_Char(0);
	Uart1_Put_Char(0);//磁力计
	sum += Uart1_Put_Char(BYTE1(gg));//7 ANGLE DATA ROLL
	sum += Uart1_Put_Char(BYTE0(gg));
	sum += Uart1_Put_Char(BYTE1(hh));//8 PITCH
	sum += Uart1_Put_Char(BYTE0(hh));
	Uart1_Put_Char(0);//YAW
	Uart1_Put_Char(0);
	
	Uart1_Put_Char(0);
	Uart1_Put_Char(0);
	Uart1_Put_Char(0);
	Uart1_Put_Char(0);
	Uart1_Put_Char(sum);
}
void Uart1_Send_AE(uint16_t aa,uint16_t bb,uint16_t cc,uint16_t dd,uint16_t ee)
{
	unsigned char sum = 0;
	count=0;
	sum += Uart1_Put_Char(0x88);
	sum += Uart1_Put_Char(0xAE);
	sum += Uart1_Put_Char(0x1C);
	
	Uart1_Put_Char(0);
	Uart1_Put_Char(0);//throttle
	Uart1_Put_Char(0);
	Uart1_Put_Char(0);//yaw
	Uart1_Put_Char(0);
	Uart1_Put_Char(0);//roll
	Uart1_Put_Char(0);
	Uart1_Put_Char(0);//pitch
	Uart1_Put_Char(0);
	Uart1_Put_Char(0);//aux 1
	Uart1_Put_Char(0);
	Uart1_Put_Char(0);//aux 2
	Uart1_Put_Char(0);
	Uart1_Put_Char(0);//aux 3
	Uart1_Put_Char(0);
	Uart1_Put_Char(0);//aux 4
	Uart1_Put_Char(0);
	Uart1_Put_Char(0);//aux 5
	
	sum += Uart1_Put_Char(BYTE1(aa));//PWM 1
	sum += Uart1_Put_Char(BYTE0(aa));
	sum += Uart1_Put_Char(BYTE1(bb));//PWM 2
	sum += Uart1_Put_Char(BYTE0(bb));
	sum += Uart1_Put_Char(BYTE1(cc));//PWM 3
	sum += Uart1_Put_Char(BYTE0(cc));
	sum += Uart1_Put_Char(BYTE1(dd));//PWM 4
	sum += Uart1_Put_Char(BYTE0(dd));
	sum += Uart1_Put_Char(BYTE1(ee));//VOLTAGE
	sum += Uart1_Put_Char(BYTE0(ee));


	Uart1_Put_Char(sum);
}

void send_wave(int tx_num)//一共发送几个字节
{
	char count_1=0;
	while(count_1<tx_num)
      UART_Putc(TxBuffer[count_1++]);
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
