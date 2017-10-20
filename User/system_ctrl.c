

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x.h"
#include "user.h"
/* Private defines -----------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
void vRunStep(void);

u8   g_u8SystemRunStep;

uint16_t  g_u16PwmDutySet;    //设定占空比
uint16_t  g_u16PwmDutyCur;    //当前占空比
uint16_t  g_u16PwmDutyCurBak;
//
//// pwm占空比调节,每100us调用一次
//void vPwmDutyAdj(void)
//{
//  static u8 s_u8PwmAdjCnt;
//  static u8 s_u8PwmAdjCntCmp;
//  static uint16_t s_u16PwmDutyReg;
//
//  if (g_u16PwmDutySet != g_u16PwmDutyCur)
//  {
//    if (g_u16PwmDutyCur > P30_DUTY_RATIO)     //30%~100%进行0.5ms/一格调节
//    {
//      s_u8PwmAdjCntCmp = TIME_P5MS_B100US;
//    }
//    else
//    {
//      s_u8PwmAdjCntCmp = TIME_2MS_B100US;
//    }
//
//    if (s_u8PwmAdjCnt >= s_u8PwmAdjCntCmp)
//    {
//      s_u8PwmAdjCnt = 0;
//
//      if (g_u16PwmDutySet > g_u16PwmDutyCur)
//      {
//        g_u16PwmDutyCur++;
//      }
//      else
//      {
//        g_u16PwmDutyCur--;
//      }
//    }
//    else
//    {
//      s_u8PwmAdjCnt++;
//    }
//  }
//
//  if (s_u16PwmDutyReg != g_u16PwmDutyCur)
//  {
//    s_u16PwmDutyReg = g_u16PwmDutyCur;
//
//    /* Set the Pulse value */
//    //static void SetTIM1_CH2_PWM_Duty(uint16_t PulseValue)
//    TIM1->CCR2H = (u8)(g_u16PwmDutyCur >> 8);
//    TIM1->CCR2L = (u8)(g_u16PwmDutyCur);
//  }
//}


void vSystemCtrl(void)
{
  SCHTaskBegin();
  
  while(1)
  {
    vRunStep();
    
    SCHCurTaskDly(10 / SCH_SYS_TICKS_MS);    
  }
  
  SCHTaskEnd();  
}


void vRunStep(void)
{
  switch (g_u8SystemRunStep)
  {
  case SYS_POWER_ON_DELAY:
//    mRedLedOff();
//    mGreenLedOff();
//    mSystemOff();

//    mSCHCurTaskDly(500 / SCH_SYS_TICKS_MS);  //上电延时500ms
//    g_u8SystemRunStep = SYS_STATUS_CHK;        //进入下一步
    break;
    //----------------------------------------
  case SYS_STATUS_CHK:

//    if ((g_u8SystemStatus.bit.DcOvp) || (g_u8SystemStatus.bit.DcUvp) || (g_u8SystemStatus.bit.Otp))  //系统异常
//    {
//      g_u8SystemRunStep = SYS_ERROR;
//    }
//    else
//    {
//      mSystemOn();
//      vNormalLedDisplay();
//    }
    break;
    //----------------------------------------
  case SYS_ERROR:

//    if (g_u8SystemStatus.all == 0)  //恢复正常?
//    {
//      g_u8SystemRunStep = SYS_STATUS_CHK; //系统出现故障后则进行检查故障恢复状态
//    }
//    else
//    {
//      mSystemOff();
//      vErrorLedDisplay();
//    }

    break;
    //----------------------------------------
  default:
    g_u8SystemRunStep = SYS_POWER_ON_DELAY;
  }
}

/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/
