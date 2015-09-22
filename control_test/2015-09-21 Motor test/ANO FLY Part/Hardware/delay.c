#include "delay.h"
#include "sys.h"
#include "misc.h"

static u8  fac_us = 0; //us延时倍乘数
static u16 fac_ms = 0; //ms延时倍乘数

//初始化延迟函数
//SYSTICK的时钟固定为HCLK时钟的1/8
//SYSCLK:系统时钟
/**************************实现函数********************************************
*函数原型: void delay_init(u8 SYSCLK)
*功　　能: 初始化延迟系统，使延时程序进入可用状态
*******************************************************************************/
void SysTick_Init(void)
{
    SysTick->CTRL &= 0xfffffffb; //bit2清空,选择外部时钟  HCLK/8
    fac_us = 72 / 8;
    fac_ms = (u16)fac_us * 1000;
}

//延时nms
//注意nms的范围
//SysTick->LOAD为24位寄存器,所以,最大延时为:
//nms<=0xffffff*8*1000/SYSCLK
//SYSCLK单位为Hz,nms单位为ms
/**************************实现函数********************************************
*函数原型: void delay_ms(u16 nms)
*功　　能: 毫秒级延时  延时nms  nms<=1864
*******************************************************************************/
void delay_ms(u16 nms)
{
    u32 temp;
    SysTick->LOAD = (u32)nms * fac_ms; //时间加载(SysTick->LOAD为24bit)
    SysTick->VAL = 0x00;          //清空计数器
    SysTick->CTRL = 0x01 ;        //开始倒数
    do
    {
        temp = SysTick->CTRL;
    }
    while(temp & 0x01 && !(temp & (1 << 16))); //等待时间到达
    SysTick->CTRL = 0x00;     //关闭计数器
    SysTick->VAL = 0X00;      //清空计数器
}

//延时nus
//nus为要延时的us数.
/**************************实现函数********************************************
*函数原型: void delay_us(u32 nus)
*功　　能: 微秒级延时  延时nus  nms<=1864
*******************************************************************************/
void delay_us(u32 nus)
{
    u32 temp;
    SysTick->LOAD = nus * fac_us; //时间加载
    SysTick->VAL = 0x00;      //清空计数器
    SysTick->CTRL = 0x01 ;    //开始倒数
    do
    {
        temp = SysTick->CTRL;
    }
    while(temp & 0x01 && !(temp & (1 << 16))); //等待时间到达
    SysTick->CTRL = 0x00;     //关闭计数器
    SysTick->VAL = 0X00;      //清空计数器
}

/***************可以使用HCLK没有8分频*****************/

//void delay_us(uint32_t n)  ////////延时多少微秒，n就输入多少！ 
//{
//  SysTick->LOAD=72*n;	//装载计数值，因为时钟72M，72次在1μs   
//	SysTick->CTRL=0x00000005;//时钟来源设为为HCLK(72M)，打开定时器  
//  while(!(SysTick->CTRL&0x00010000)); //等待计数到0   
//	SysTick->CTRL=0x00000004;//关闭定时器 
//}

//void delay_ms(uint16_t n)  //延时多少毫秒，n就输入多少！ 
//{
//  SysTick->LOAD=72000*n;	//装载计数值，因为时钟72M，72000次在1ms   
//	SysTick->CTRL=0x00000005;//时钟来源设为为HCLK(72M)，打开定时器  
//  while(!(SysTick->CTRL&0x00010000)); //等待计数到0   
//	SysTick->CTRL=0x00000004;//关闭定时器 
//}




























