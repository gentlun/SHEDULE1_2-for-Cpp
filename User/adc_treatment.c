/**********************Copyright (c)  2010-2020 ********************************
**                  Electrical Technology Co.,Ltd..
**                                 
**                  http://www.***.com
**
**----------------------------File Info-----------------------------------------
** File name:      adc_treatment.c
** Created by:      
** Created date:   2013-03-05
** Version:        V1.0
** Descriptions:   the original version
**
**------------------------------------------------------------------------------
** Modified by: 
** Modified date: 
** Version:	
** Descriptions: 
**
*******************************************************************************/

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x.h"
#include "user.h"
/* Private defines -----------------------------------------------------------*/
u16 ADC_ConvertedValueBuffer[16];

/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
/*-----------------------------------------------------------------------------*/
FAULT_SYS_UNION  g_u8SystemStatus;

u16 g_u16DcVAdcResult;

u16 g_u16DcCurAdcResult;


//u16 rAdc2Trans(u8 ADC2_Channel)
//{
//  WORD_UNION tAdcResualtRead;
//
//  /* Clear the ADC2 channels */
//  ADC2->CSR &= (u8)(~ADC2_CSR_CH);
//  /* Select the ADC2 channel */
//  ADC2->CSR |= (u8)(ADC2_Channel);
//
//  /* Clear EOC  flag status */
//  ADC2->CSR &= (u8)(~ADC2_CSR_EOC);
//  // Set ADON again to start AD convert.
//  ADC2->CR1 |= ADC2_CR1_ADON;
//
//  while (!((u8)(ADC2->CSR & ADC2_CSR_EOC)));  // Waiting for AD convert finished (EOC=1).
//                                                   //return (ADC2_GetConversionValue());
//  tAdcResualtRead.byte.L = ADC2->DRL;   /* Read LSB first */
//  tAdcResualtRead.byte.H = ADC2->DRH;   /* Then read MSB  */
//  return (tAdcResualtRead.all);
//}

void vVoltageProtection(void)
{
  g_u16DcVAdcResult = ADC_ConvertedValueBuffer[VDC_OUT_CH];
  if (g_u16DcVAdcResult > 512)
  {
    asm("nop");
    asm("nop");
  }
  //.......
}

void vCurrentProtection(void)
{
  g_u16DcCurAdcResult = ADC_ConvertedValueBuffer[IDC_OUT_CH];
  if (g_u16DcCurAdcResult > 512)
  {
    asm("nop");
    asm("nop");
  }
  //......
}


void vAdcResultSend(void)
{
  static u8 s_u8AdcBuffer[ADC_BUF_SIZE];
  static u8 i;
  static u8 *pDataBuffer, u8DataSize;
  static u8		 u8RetStatus;
  SCHTaskBegin();

  while (1)
  {    
    SCHCurTaskDly(1000 / SCH_SYS_TICKS_MS);   //delay 1000ms

    SCHTaskQpend();           //任务等待消息

    pDataBuffer = (u8 *)AdcResultSendTcb.pData;
    u8DataSize  = AdcResultSendTcb.Size;

    if (u8DataSize > ADC_BUF_SIZE)
    {
      u8DataSize = ADC_BUF_SIZE;
      //出错处理
    }

    for (i = 0; i < u8DataSize; i++)
    {
      s_u8AdcBuffer[i] = *pDataBuffer; //copy to s_u8AdcBuffer
      pDataBuffer++;
    }

    //使用如下类似方法便可与其它任务共享UART_TX资源,即任务要使用UART_TX前先检查其队列是否可用,不可用则等待
    for (u8DataSize = 0; u8DataSize < 255; u8DataSize++)  //借用u8DataSize
    {
	  SCHTaskGetQFree(UartTxTcb,u8RetStatus);
      if (u8RetStatus == SCH_Q_FREE)      //检查UART_TX发送任务队列是否可用
      {
        SCHTaskQpost(UartTxTcb,
                     s_u8AdcBuffer,
                     u8DataSize);
        break;
      }
      else
      {
        SCHCurTaskDly(1 / SCH_SYS_TICKS_MS);  //delay 1ms
      }
    }

    if (u8DataSize >= 100)
    {
      //出错处理
    }    
  }

  SCHTaskEnd();
}


void vAdcTreatment(void)
{
  static u8 s_u8Adc[ADC_BUF_SIZE];
  static u8	retStat;

  SCHTaskBegin();

  while (1)
  {

    vVoltageProtection();

    vCurrentProtection();

	SCHTaskGetQFree(AdcResultSendTcb,retStat)
    if (retStat == SCH_Q_FREE)
    {
      s_u8Adc[0] = (u8)(g_u16DcVAdcResult >> 8);
      s_u8Adc[1] = (u8)g_u16DcVAdcResult;
      s_u8Adc[2] = (u8)(g_u16DcCurAdcResult>>8);
      s_u8Adc[3] = (u8)g_u16DcCurAdcResult;

      SCHTaskQpost(AdcResultSendTcb,
                   s_u8Adc,
                   ADC_BUF_SIZE);
    }

		ADC_Cmd(ADC1, ENABLE);	//启动转换
		
    SCHCurTaskDly(10 / SCH_SYS_TICKS_MS);
  }

  SCHTaskEnd();
}

/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/
