/********************************************************************************
  * @file    TIM/TimeBase/stm32f10x_it.c 
  * @author  MCD Application Team
  * @version V3.5.0
  * @date    11-March-2011
  * @brief   Main Interrupt Service Routines.
  *          This file provides template for all exceptions handler and peripherals
  *          interrupt service routine.
  ******************************************************************************
  * @copy
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * <h2><center>&copy; COPYRIGHT 2011 STMicroelectronics</center></h2>
  */

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x_it.h"
#include "user.h"

/** @addtogroup STM32F10x_StdPeriph_Examples
  * @{
  */

/** @addtogroup TIM_TimeBase
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
  * @brief  This function handles Debug Monitor exception.
  * @param  None
  * @retval None
  */
void DebugMon_Handler(void)
{
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
  * @brief  This function handles PendSV_Handler exception.
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
  u8SchdTicksCnt++;
}


void TIM1_UP_IRQHandler(void)
{
  if (TIM_GetITStatus(TIM1, TIM_IT_Update) != RESET)
  {
    GPIO_WriteBit(GPIOC, GPIO_Pin_13, (BitAction)(1 - GPIO_ReadOutputDataBit(GPIOC, GPIO_Pin_13)));

    TIM_ClearITPendingBit(TIM1, TIM_IT_Update);
  }
}

/******************************************************************************/
/*            STM32F10x Peripherals Interrupt Handlers                        */
/******************************************************************************/

/**
  * @brief  This function handles TIM2 global interrupt request.
  * @param  None
  * @retval None
  */
void TIM2_IRQHandler(void)
{

  if (TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET)
  {
    //	GPIO_WriteBit(GPIOA, GPIO_Pin_2, (BitAction)(1 - GPIO_ReadOutputDataBit(GPIOA, GPIO_Pin_2)));

    TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
  }

  //  if (TIM_GetITStatus(TIM2, TIM_IT_CC1) != RESET)
  //  {
  //    TIM_ClearITPendingBit(TIM2, TIM_IT_CC1);
  //
  //    /* Pin PC.06 toggling with frequency = 73.24 Hz */
  //    GPIO_WriteBit(GPIOA, GPIO_Pin_6, (BitAction)(1 - GPIO_ReadOutputDataBit(GPIOA, GPIO_Pin_6)));
  //    capture = TIM_GetCapture1(TIM2);
  //    TIM_SetCompare1(TIM2, capture + CCR1_Val);
  //  }
  //  else if (TIM_GetITStatus(TIM2, TIM_IT_CC2) != RESET)
  //  {
  //    TIM_ClearITPendingBit(TIM2, TIM_IT_CC2);
  //
  //    /* Pin PC.07 toggling with frequency = 109.8 Hz */
  //    GPIO_WriteBit(GPIOA, GPIO_Pin_7, (BitAction)(1 - GPIO_ReadOutputDataBit(GPIOA, GPIO_Pin_7)));
  //    capture = TIM_GetCapture2(TIM2);
  //    TIM_SetCompare2(TIM2, capture + CCR2_Val);
  //  }
  //  else if (TIM_GetITStatus(TIM2, TIM_IT_CC3) != RESET)
  //  {
  //    TIM_ClearITPendingBit(TIM2, TIM_IT_CC3);
  //
  //    /* Pin PC.08 toggling with frequency = 219.7 Hz */
  //    GPIO_WriteBit(GPIOA, GPIO_Pin_8, (BitAction)(1 - GPIO_ReadOutputDataBit(GPIOA, GPIO_Pin_8)));
  //    capture = TIM_GetCapture3(TIM2);
  //    TIM_SetCompare3(TIM2, capture + CCR3_Val);
  //  }
  //  else
  //  {
  //    TIM_ClearITPendingBit(TIM2, TIM_IT_CC4);
  //
  //    /* Pin PC.09 toggling with frequency = 439.4 Hz */
  //    GPIO_WriteBit(GPIOA, GPIO_Pin_9, (BitAction)(1 - GPIO_ReadOutputDataBit(GPIOA, GPIO_Pin_9)));
  //    capture = TIM_GetCapture4(TIM2);
  //    TIM_SetCompare4(TIM2, capture + CCR4_Val);
  //  }
}

extern uint16_t u16SciSendBuffer[20];
extern uint16_t u16SciSendPointer;
extern uint16_t u16SciReceiveBuffer[20];
extern uint16_t u16SciReceivePointer;


void USART1_IRQHandler(void)
{

  static u8 s_u8IsrRxBuffer[UART_ISR_RX_BUF_SIZE];
  static u8 s_u8IsrRxCounter;
  static u8 retStat;
  if (USART_GetITStatus(USART1, USART_IT_TC) != RESET)
{
//if ( u16SciSendPointer < 20 )
//{
//  USART1->DR = u16SciSendBuffer[u16SciSendPointer ++];
//}

  USART_ClearITPendingBit(USART1, USART_IT_TC);
}

  if (USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)    //接收中断
  {
    s_u8IsrRxBuffer[s_u8IsrRxCounter] = (u8)USART_ReceiveData(USART1);

    if (s_u8IsrRxBuffer[s_u8IsrRxCounter] == 0xaa) //for test
    {
      s_u8IsrRxCounter++;
	  SCHTaskGetQFree(UartRxTcb,retStat);
      if (retStat == SCH_Q_FREE)
      {
        SCHTaskQpost(UartRxTcb,
                     s_u8IsrRxBuffer,
                     s_u8IsrRxCounter);        
      }
      else
      {
        //出错处理
      }

      s_u8IsrRxCounter = 0;
    }
    else
    {
      s_u8IsrRxCounter++;
    }

    if (s_u8IsrRxCounter > UART_RX_BUF_SIZE)
    {
      s_u8IsrRxCounter = 0;
      //溢出出错处理
    }

    USART_ClearITPendingBit(USART1, USART_IT_RXNE);
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

/**
  * @}
  */

/******************* (C) COPYRIGHT 2009 STMicroelectronics *****END OF FILE****/
