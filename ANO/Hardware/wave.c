#include "wave.h"
#include "usart.h"

//#define tx_num	32

unsigned char TxBuffer[16];//一共发送的字节数 记得改
unsigned char count=0; 

#define BYTE0(dwTemp)       (*(char *)(&dwTemp))
#define BYTE1(dwTemp)       (*((char *)(&dwTemp) + 1))
#define BYTE2(dwTemp)       (*((char *)(&dwTemp) + 2))
#define BYTE3(dwTemp)       (*((char *)(&dwTemp) + 3))

/**************************向物理串口发一个字节***************************************
*******************************************************************************/
__inline unsigned char UART_Putc(unsigned char data)			//
{
	USART_SendData(USART1, (uint8_t) data);
	while (USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET);
	return data;
}

unsigned char Uart1_Put_Char(unsigned char DataToSend)
{
	TxBuffer[count++] = DataToSend;  
	return DataToSend;
}

unsigned char Uart1_Put_Int16(uint16_t DataToSend)
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
//void Uart1_send_custom(uint16_t aa)
//{
//	
//	unsigned char sum = 0;
//	count=0;

//	sum +=Uart1_Put_Char(0x88);
//	sum +=Uart1_Put_Char(0xA1);
//	
//	sum +=Uart1_Put_Char(0x02);//发送的数据的长度 记得改
//	sum +=Uart1_Put_Int16(aa);//发送16位数据 
// 
//	Uart1_Put_Char(sum);
//}
/****************给第一帧 第一位 发送float数据*************/

void Uart1_send_custom(unsigned char fun,float aa,float bb,float cc)
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
void Uart1_Send_AF(signed int aa,signed int bb,signed int cc,signed int dd,signed int ee,signed int ff,signed int gg,signed int hh)
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
	sum += Uart1_Put_Char(BYTE1(cc));//3
	sum += Uart1_Put_Char(BYTE0(cc));
	sum += Uart1_Put_Char(BYTE1(dd));//4
	sum += Uart1_Put_Char(BYTE0(dd));
	sum += Uart1_Put_Char(BYTE1(ee));//5
	sum += Uart1_Put_Char(BYTE0(ee));
	sum += Uart1_Put_Char(BYTE1(ff));//6
	sum += Uart1_Put_Char(BYTE0(ff));
	Uart1_Put_Char(0);
	Uart1_Put_Char(0);
	Uart1_Put_Char(0);
	Uart1_Put_Char(0);
	Uart1_Put_Char(0);
	Uart1_Put_Char(0);
	sum += Uart1_Put_Char(BYTE1(gg));//7,4500->45'//这是姿态!!!
	sum += Uart1_Put_Char(BYTE0(gg));
	sum += Uart1_Put_Char(BYTE1(hh));//8
	sum += Uart1_Put_Char(BYTE0(hh));
	Uart1_Put_Char(0);
	Uart1_Put_Char(0);
	Uart1_Put_Char(0);
	Uart1_Put_Char(0);
	Uart1_Put_Char(0);
	Uart1_Put_Char(0);
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
