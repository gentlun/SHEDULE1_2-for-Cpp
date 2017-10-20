/**
 ******************************************************************************
  * @file TIM/TimeBase/main.c 
  * @author  MCD Application Team
  * @version V3.5.0
  * @date    09/28/2011
  * @brief   Main program body
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
#include "stm32f10x.h"
#include "user.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/
void Peripherals_Config(void);
void vDataInit(void);
void vLedTest(void);

/**/
SCH_TCB UartTxTcb;
SCH_TCB UartRxTcb;
SCH_TCB WdtClearTcb;
SCH_TCB LedTestTcb;
SCH_TCB AdcTreatmentTcb;
SCH_TCB AdcResultSendTcb;


void vUserTaskCreate(void)
{
  //任务创建的顺序即为任务运行的优先级顺序(从高优先级到低优先级)
  SCHTaskCreate(&WdtClearTcb, vWdtClear,0);
  SCHTaskCreate(&UartTxTcb, vUartSendData,1);
  SCHTaskCreate(&UartRxTcb, vUartReceiveData,2);
  SCHTaskCreate(&AdcResultSendTcb, vAdcResultSend,3);
  SCHTaskCreate(&AdcTreatmentTcb, vAdcTreatment,4);
  SCHTaskCreate(&LedTestTcb, vLedTest,5);
}

/* Private functions ---------------------------------------------------------*/

void RemapCheck(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
  /* 专用于Remap功能 检测 */
  GPIO_Write(GPIOA, 0x0000);               //初始化为低电平
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0; //GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;

  GPIO_Init(GPIOA, &GPIO_InitStructure);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO,ENABLE);	
  GPIO_PinRemapConfig ( GPIO_Remap_SWJ_JTAGDisable, ENABLE );
    //GPIO_PinRemapConfig(GPIO_Remap_SWJ_Disable, ENABLE); //禁用JTAG功能
  /*if (GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_0) > 0)  //上电默认Remap
  {
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO,ENABLE);	
	//GPIO_PinRemapConfig ( GPIO_Remap_SWJ_JTAGDisable, ENABLE );
    GPIO_PinRemapConfig(GPIO_Remap_SWJ_Disable, ENABLE); //禁用JTAG功能
  }
  else
  {
    asm("nop");
  }*/
}

void main(void)
{

  __disable_interrupt();

  Peripherals_Config();
  
  //RemapCheck();

  vDataInit();
  
  vUserTaskCreate();

  __enable_interrupt();   /* Enable general interrupts */

  SCHTaskStart();
}

void vLedTest(void)
{
  SCHTaskBegin();

  while (1)
  {
    mGreenLedToggle();

    SCHCurTaskDly(500 / SCH_SYS_TICKS_MS);  //delay 500ms
  }

  SCHTaskEnd();
}


#ifdef  USE_FULL_ASSERT

/**
 * @brief  Reports the name of the source file and the source line number
 *   where the assert_param error has occurred.
 * @param  file: pointer to the source file name
 * @param  line: assert_param error line source number
 * @retval None 
 */
void assert_failed(u8 *file, uint32_t line)
{
  /* User can add his own implementation to report the file name and line number,
   ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  while (1)
  {  
  }
}
#endif










/**
  * @}
  */

/**
  * @}
  */

/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/
