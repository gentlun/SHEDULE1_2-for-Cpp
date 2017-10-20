

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
	  IWDG_ReloadCounter();   //25.6MS���,ֻ������30MS���,20MS�������

    WWDG_SetCounter(WWDG_COUNTER_INIT);     //�崰�ڿ��Ź�,3.64ms~18.2ms֮����
  }

  SCHTaskEnd();
}



/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/
