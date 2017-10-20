

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


void vWdtClear(void)
{
  SCHTaskBegin();

  while(1)
  {
		SCHCurTaskDly(10/SCH_SYS_TICKS_MS);     //delay 10ms
		
  /* Reload IWDG counter */
	  IWDG_ReloadCounter();   //25.6MS溢出,只测试了30MS溢出,20MS不会溢出

    WWDG_SetCounter(WWDG_COUNTER_INIT);     //清窗口看门狗,3.64ms~18.2ms之间清
  }

  SCHTaskEnd();
}



/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/
