

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x.h"
#include "user.h"
/* Private defines -----------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
void vRunStep(void);

u8   g_u8SystemRunStep;

uint16_t  g_u16PwmDutySet;    //�趨ռ�ձ�
uint16_t  g_u16PwmDutyCur;    //��ǰռ�ձ�
uint16_t  g_u16PwmDutyCurBak;
//
//// pwmռ�ձȵ���,ÿ100us����һ��
//void vPwmDutyAdj(void)
//{
//  static u8 s_u8PwmAdjCnt;
//  static u8 s_u8PwmAdjCntCmp;
//  static uint16_t s_u16PwmDutyReg;
//
//  if (g_u16PwmDutySet != g_u16PwmDutyCur)
//  {
//    if (g_u16PwmDutyCur > P30_DUTY_RATIO)     //30%~100%����0.5ms/һ�����
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

//    mSCHCurTaskDly(500 / SCH_SYS_TICKS_MS);  //�ϵ���ʱ500ms
//    g_u8SystemRunStep = SYS_STATUS_CHK;        //������һ��
    break;
    //----------------------------------------
  case SYS_STATUS_CHK:

//    if ((g_u8SystemStatus.bit.DcOvp) || (g_u8SystemStatus.bit.DcUvp) || (g_u8SystemStatus.bit.Otp))  //ϵͳ�쳣
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

//    if (g_u8SystemStatus.all == 0)  //�ָ�����?
//    {
//      g_u8SystemRunStep = SYS_STATUS_CHK; //ϵͳ���ֹ��Ϻ�����м����ϻָ�״̬
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
