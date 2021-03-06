/**
  ******************************************************************************
  * @file    Project/STM32F10x_StdPeriph_Template/stm32f10x_it.c
  * @author  MCD Application Team
  * @version V3.5.0
  * @date    08-April-2011
  * @brief   Main Interrupt Service Routines.
  *          This file provides template for all exceptions handler and
  *          peripherals interrupt service routine.
  ******************************************************************************
  * @attention
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * <h2><center>&copy; COPYRIGHT 2011 STMicroelectronics</center></h2>
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x_it.h"
#include "usart.h"
#include "adc.h"
#include "stdint.h"
#include "24l01.h"
#include "Key.h"
#include "string.h"
#include "LED.h"
/** @addtogroup STM32F10x_StdPeriph_Template
  * @{
  */
#define BYTE0(dwTemp)       (*(char *)(&dwTemp))
#define BYTE1(dwTemp)       (*((char *)(&dwTemp) + 1))
#define BYTE2(dwTemp)       (*((char *)(&dwTemp) + 2))
#define BYTE3(dwTemp)       (*((char *)(&dwTemp) + 3))
	
#define ON 1;
#define OFF 0;
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/******************************************************************************/
/*            Cortex-M3 Processor Exceptions Handlers                         */
/******************************************************************************/

/**
  * @brief  This function handles NMI exception.
  * @param  None
  * @retval None
  */
void NMI_Handler(void)
{
}

/**
  * @brief  This function handles Hard Fault exception.
  * @param  None
  * @retval None
  */
void HardFault_Handler(void)
{
  /* Go to infinite loop when Hard Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Memory Manage exception.
  * @param  None
  * @retval None
  */
void MemManage_Handler(void)
{
  /* Go to infinite loop when Memory Manage exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Bus Fault exception.
  * @param  None
  * @retval None
  */
void BusFault_Handler(void)
{
  /* Go to infinite loop when Bus Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Usage Fault exception.
  * @param  None
  * @retval None
  */
void UsageFault_Handler(void)
{
  /* Go to infinite loop when Usage Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles SVCall exception.
  * @param  None
  * @retval None
  */
void SVC_Handler(void)
{
}

/**
  * @brief  This function handles Debug Monitor exception.
  * @param  None
  * @retval None
  */
void DebugMon_Handler(void)
{
}

/**
  * @brief  This function handles PendSVC exception.
  * @param  None
  * @retval None
  */
void PendSV_Handler(void)
{
}

/**
  * @brief  This function handles SysTick Handler.
  * @param  None
  * @retval None
  */
void SysTick_Handler(void)
{
}
extern u8 rc_buf[32];
extern u8 key;
u16 throttle;
#define upRange 2086 //2010-4096
#define downRange 2010 //0-2010
#define inStaticRange(throttle) (throttle<2015 && throttle>2005) ? true:false //中值为2010
void TIM3_IRQHandler(void)    //0.5ms中断一次
{
  static u8 ms1 = 0, ms2 = 0, ms5 = 0, ms10 = 0;    //中断次数计数器
	static u16 throttle_vol =0;
  static float percent = 0;
	static u16 gap;
  uint16_t per_1000;
	static u16 Roll = 0;
	static u16 Pitch = 0;
	static u16 Yaw = 0;
  if (TIM3->SR & TIM_IT_Update)   //if ( TIM_GetITStatus(TIM3 , TIM_IT_Update) != RESET )
  {
    TIM3->SR = ~TIM_FLAG_Update;//TIM_ClearITPendingBit(TIM3 , TIM_FLAG_Update);   //清除中断标志
    //每次中断都执行,0.5ms
    ms1++;
    ms2++;
    ms5++;
    ms10++;
    //Nrf_Check_Event();
    if (ms1 == 2)     //每两次中断执行一次,1ms
    {
      ms1 = 0;
    }
    if (ms2 == 4)     //每四次中断执行一次,2ms
    {
      ms2 = 0;
    }
    if (ms5 == 10)
    {
      ms5 = 0;        //每十次中断执行一次,5ms
    }
    if (ms10 == 20)
    {
      ms10 = 0;       //每二十次中断执行一次,10ms
      //percent = (float)voltage1() / 4093.0;
      //per_1000 = (uint16_t)(percent * 1000);
			
			
			throttle_vol = voltage1();
			if(throttle_vol>2015)//左-上
			{
				gap = (uint16_t)(((float)(throttle_vol-2010)/upRange)*20.0);
				throttle = throttle + (uint16_t)(((float)(throttle_vol-2010)/upRange)*20.0);
				if(throttle>999) throttle=999;
			}
			else if (throttle_vol<2005) //左-下
			{
				gap = (uint16_t)((float)(2010-throttle_vol)/downRange*25.0);
				if(throttle<gap) throttle = 0;
				else if(throttle>gap) {
					throttle = throttle - gap;
				}
			}
			
			Yaw = voltage2();
			Roll = voltage3();
			Pitch = voltage4();
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
				rc_buf[8] = 'a';//接收端判断如果为a，则判为start
				
			if(key == FUN_KEY_DOWN)
				rc_buf[8] = 'b';//接收端判断如果为b,则判为stop
    }
  }
}
/******************************************************************************/
/*                 STM32F10x Peripherals Interrupt Handlers                   */
/*  Add here the Interrupt Handler for the used peripheral(s) (PPP), for the  */
/*  available peripheral interrupt handler's name please refer to the startup */
/*  file (startup_stm32f10x_xx.s).                                            */
/******************************************************************************/

/**
  * @brief  This function handles PPP interrupt request.
  * @param  None
  * @retval None
  */
/*void PPP_IRQHandler(void)
{
}*/

/**
  * @}
  */


/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/
