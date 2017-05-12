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

/** @addtogroup STM32F10x_StdPeriph_Template
  * @{
  */

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
#include "ANO_TC_STM32F1_I2C.h"
void I2C2_EV_IRQHandler( void )
{
	ANO_TC_I2C2_EV_IRQ();
}
void I2C2_ER_IRQHandler( void )
{
	ANO_TC_I2C2_ER_IRQ();
}
#include "LED.h"
#include "usart.h"
#include "wave.h"

#include "delay.h"

u8 getMpu6050Data = 0;
u8 calculateAngle = 0;
u8 sendData = 0;
u8 ms1_cnt = 0;
u8 ms2_cnt = 0;
u8 led_on=0;
int usound_ms1_cnt = 0;
void TIM3_IRQHandler(void)    //0.5ms�ж�һ��
{
//	static u8 led_on = 0;
	
	//0.5ms
  if (TIM3->SR & TIM_IT_Update)   //if ( TIM_GetITStatus(TIM3 , TIM_IT_Update) != RESET )
  {
    TIM3->SR = ~TIM_FLAG_Update;//TIM_ClearITPendingBit(TIM3 , TIM_FLAG_Update);   //����жϱ�־
    ms1_cnt++;
		ms2_cnt++;
		
		if (ms1_cnt == 2) //1ms
    {
      getMpu6050Data=1;
			sendData=1;
			ms1_cnt = 0;
    }
		
//		if(ms2_cnt == 4)
//		{
//			calculateAngle=1;
//			ms2_cnt = 0;
//		}
		Data_Transfer();
		//if(ms1_cnt==4)
		//{
			//sendData=1;
			//ms1_cnt = 0;
		//}
  }
}

u8  TIM4CH4_CAPTURE_STA=0;	//���벶��״̬		    				
u16	TIM4CH4_CAPTURE_VAL;	//���벶��ֵ
//��ʱ��5�жϷ������
void TIM4_IRQHandler(void)
{

    if ((TIM4CH4_CAPTURE_STA & 0X80) == 0) //��δ�ɹ����񣨳ɹ������־λΪ0��
    {
        if (TIM_GetITStatus(TIM4, TIM_IT_Update) != RESET)
        {
            if (TIM4CH4_CAPTURE_STA & 0X40) //�Ѿ�������ߵ�ƽ�� �ȴ��½��صĵ����Ĺ����У���������̫����������Χ->ǿ����Ϊ������ɡ�
                                                                                            //���û��̫������ߵ�ƽ����������ۼ�
            {
                if ((TIM4CH4_CAPTURE_STA & 0X3F) == 0X3F) //ǿ�Ʊ�ǲ������
                {
                    TIM4CH4_CAPTURE_STA |= 0X80; //��ǳɹ�������һ��
                    TIM4CH4_CAPTURE_VAL = 0XFFFF;
                } else TIM4CH4_CAPTURE_STA++;
            }
        }
        if (TIM_GetITStatus(TIM4, TIM_IT_CC4) != RESET)//����1���������¼��������½�������������ˣ���û�����½�����������������û�ɹ�����������
        {
            if (TIM4CH4_CAPTURE_STA & 0X40)     //�����������(��־λ��1)����û�����½�����β���һ���½���
            {
                TIM4CH4_CAPTURE_STA |= 0X80;    //��ǳɹ�����һ�������أ�����λ���ֲ�������ɲ����־λ��1��
                TIM4CH4_CAPTURE_VAL = TIM_GetCapture4(TIM4);
                TIM_OC4PolarityConfig(TIM4, TIM_ICPolarity_Rising); //CC1P=0 ����Ϊ�����ز���
            } else                               //��û�ɹ�������������β�����������еĵ�һ��������
            {
                TIM4CH4_CAPTURE_STA = 0;        //���
                TIM4CH4_CAPTURE_VAL = 0;
                TIM_SetCounter(TIM4, 0);
                TIM4CH4_CAPTURE_STA |= 0X40;    //��ǲ�����������
                TIM_OC4PolarityConfig(TIM4, TIM_ICPolarity_Falling);    //CC1P=1 ����Ϊ�½��ز���
            }
        }
    }

    TIM_ClearITPendingBit(TIM4, TIM_IT_CC4 | TIM_IT_Update); //����жϱ�־λ

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
