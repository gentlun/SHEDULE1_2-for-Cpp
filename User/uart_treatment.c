
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
    SCHTaskQpend();           //����ȴ���Ϣ

    pDataBuffer = (u8 *)UartTxTcb.pData;
    u8DataSize  = UartTxTcb.Size;

    if (u8DataSize > UART_TX_BUF_SIZE)
    {
      u8DataSize = UART_TX_BUF_SIZE;
      //������
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
		SCHCurTaskDly(1 / SCH_SYS_TICKS_MS);        //��ʱ����1���ֽ�����Ҫ��ʱ��
      }
	  else
	  {
		SCHCurTaskDly(0 / SCH_SYS_TICKS_MS);        //����ʱ�´�ֱ�ӽ�����ѯ
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
    SCHTaskQpend();           //����ȴ������жϷ�����Ϣ

    pDataBuffer = (u8 *)UartRxTcb.pData;
    u8DataSize  = UartRxTcb.Size;

    if (u8DataSize > UART_RX_BUF_SIZE)
    {
      u8DataSize = UART_RX_BUF_SIZE;
      //������
    }

    for (s_u8RxCounter = 0; s_u8RxCounter < u8DataSize; s_u8RxCounter++)
    {
      s_u8RxBuffer[s_u8RxCounter] = (*pDataBuffer) + 0x11; //copy to s_u8TxBuffer (add 0x11)
      pDataBuffer++;
    }

    //ʹ���������Ʒ������������������UART_TX��Դ,������Ҫʹ��UART_TXǰ�ȼ��������Ƿ����,��������ȴ�
    for (u8DataSize = 0; u8DataSize < 255; u8DataSize++)  //����u8DataSize
    {
	  SCHTaskGetQFree(UartTxTcb,retStat);
      if (retStat == SCH_Q_FREE)      //���UART_TX������������Ƿ����
      {
        SCHTaskQpost(UartTxTcb,
		s_u8RxBuffer,
		s_u8RxCounter); //�����յĵ�����+0x11�󷢻�ȥ
      break;
      }
      else
      {
        SCHCurTaskDly(1 / SCH_SYS_TICKS_MS);  //delay 1ms
      }
    }

    if (u8DataSize >= 100)
    {
      //������
    }
  }

  SCHTaskEnd();
}



/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/
