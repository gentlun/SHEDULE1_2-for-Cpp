
/* Includes ------------------------------------------------------------------*/
#include "stm32f10x.h"
#include "user.h"
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
/* Public functions ----------------------------------------------------------*/

void vUartSendData(void);
void vUartReceiveData(void);

void vUartSendData(void)
{
  static u8 s_u8TxBuffer[UART_TX_BUF_SIZE];
  static u8 i;
  static u8 *pDataBuffer, u8DataSize;

  SCHTaskBegin();

  while (1)
  {
    SCHTaskQpend();           //任务等待消息

    pDataBuffer = (u8 *)UartTxTcb.pData;
    u8DataSize  = UartTxTcb.Size;

    if (u8DataSize > UART_TX_BUF_SIZE)
    {
      u8DataSize = UART_TX_BUF_SIZE;
      //出错处理
    }

    for (i = 0; i < u8DataSize; i++)
    {
      s_u8TxBuffer[i] = *pDataBuffer; //copy to s_u8TxBuffer
      pDataBuffer++;
    }
    
    for (i = 0; i < u8DataSize; )			
    {
      if (USART_GetFlagStatus(USART1, USART_FLAG_TXE) == SET)
      {
        USART1->DR = s_u8TxBuffer[i++];
        //	UART1_ITConfig(UART1_IT_TXE, ENABLE);
		SCHCurTaskDly(1 / SCH_SYS_TICKS_MS);        //延时发送1个字节所须要的时间
      }
	  else
	  {
		SCHCurTaskDly(0 / SCH_SYS_TICKS_MS);        //无延时下次直接进来查询
	  }      
    }
  }

  SCHTaskEnd();
}


void vUartReceiveData(void)
{
  static u8 s_u8RxBuffer[UART_RX_BUF_SIZE];
  static u8 s_u8RxCounter;
  static u8 *pDataBuffer, u8DataSize;
  static u8 retStat;
  SCHTaskBegin();

  while (1)
  {
    SCHTaskQpend();           //任务等待接收中断发来消息

    pDataBuffer = (u8 *)UartRxTcb.pData;
    u8DataSize  = UartRxTcb.Size;

    if (u8DataSize > UART_RX_BUF_SIZE)
    {
      u8DataSize = UART_RX_BUF_SIZE;
      //出错处理
    }

    for (s_u8RxCounter = 0; s_u8RxCounter < u8DataSize; s_u8RxCounter++)
    {
      s_u8RxBuffer[s_u8RxCounter] = (*pDataBuffer) + 0x11; //copy to s_u8TxBuffer (add 0x11)
      pDataBuffer++;
    }

    //使用如下类似方法便可与其它任务共享UART_TX资源,即任务要使用UART_TX前先检查其队列是否可用,不可用则等待
    for (u8DataSize = 0; u8DataSize < 255; u8DataSize++)  //借用u8DataSize
    {
	  SCHTaskGetQFree(UartTxTcb,retStat);
      if (retStat == SCH_Q_FREE)      //检查UART_TX发送任务队列是否可用
      {
        SCHTaskQpost(UartTxTcb,
		s_u8RxBuffer,
		s_u8RxCounter); //将接收的的数据+0x11后发回去
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



/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/
