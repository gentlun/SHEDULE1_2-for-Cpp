
/* Define to prevent recursive inclusion -------------------------------------*/
/* Includes ------------------------------------------------------------------*/
#include "stm32f10x.h"
#include "schedule.h"

/* Exported types ------------------------------------------------------------*/

struct FAULT_SYS_BITS
{
  u8 DcOvp:1;
  u8 DcUvp:1;
  u8 DcOcp:1;
  u8 Otp:1;
  u8 B2:1;
  u8 B3:1;
  u8 Rsd2:2;
};
typedef union
{
  u8                 all;
  struct FAULT_SYS_BITS   bit;
}FAULT_SYS_UNION;
extern   FAULT_SYS_UNION  g_u8SystemStatus;


struct WORD_BYTE
{
  __IO  u8 H;
  __IO  u8 L;
};

typedef	union
{
  uint16_t           all;
  struct WORD_BYTE   byte;
}WORD_UNION;



/* Exported macro ------------------------------------------------------------*/
//WWDG counter clock: PCLK1=APB1=HCLK/4; (72/4=18M)/(4096*WWDG_Prescaler_4) = 1098.6Hz 对应T=0.91MS
//最长计数周期(counter-0x40)*910us=20*910us=18.2ms,
//重置计数器时，计数器当前值要小于窗口计数器的值，则有：（wwcnt-wwv）*910us=3.64ms,
//即每次重置计数器后，至少要等待3.64ms之后才能再次重置，但最长重置间隔不能超过18.2ms
#define WWDG_COUNTER_INIT	0x40+20
#define WWDG_WINDOW_VALUE	WWDG_COUNTER_INIT-4

#define   CAPTURETIME       ((uint16_t)24999)
#define   ADJ_PWM_PERIOD		((uint16_t)800)

#define   UART_ISR_RX_BUF_SIZE  20
#define   UART_RX_BUF_SIZE      20
#define   UART_TX_BUF_SIZE      20

#define   ADC_BUF_SIZE   4


#define   VDC_OUT_CH		    ADC_Channel_0
#define   IDC_OUT_CH		    ADC_Channel_1

#define   VDC_OUT_56V		   ((uint16_t)56)
#define   VDC_OUT_52V		   ((uint16_t)52)
#define   VDC_OUT_45V		   ((uint16_t)45)
#define   VDC_OUT_42V		   ((uint16_t)42)

#define   IDC_OUT_60A		   ((uint16_t)60)
#define   IDC_OUT_56A		   ((uint16_t)56)


#define   VDC_OVP_VALUE       VDC_OUT_56V
#define   VDC_OVP_RESTOR      VDC_OUT_52V

#define   VDC_UVP_VALUE       VDC_OUT_42V
#define   VDC_UVP_RESTOR      VDC_OUT_45V

#define   IDC_OCP_VALUE       IDC_OUT_60A
#define   IDC_OCP_RESTOR      IDC_OUT_56A


//调试LED 绿灯 PA3
#define   mGreenLedff()      (GPIOA->ODR &= (uint16_t)(~GPIO_Pin_15))
#define   mGreenLedOn()      (GPIOA->ODR |= (uint16_t)GPIO_Pin_15)
#define   mGreenLedToggle()  (GPIOA->ODR ^= (uint16_t)GPIO_Pin_15)
//#define   mGreenLedToggle()  (GPIOC->ODR ^= (uint16_t)GPIO_Pin_14)

//系统运行状态定义
#define	  SYS_POWER_ON_DELAY	  0	//上电延时
#define	  SYS_STATUS_CHK		    1	//系统状态检查
#define	  SYS_ERROR				      2	//系统故障状态,如欠压,过压等

/* Private variables ---------------------------------------------------------*/
extern uint16_t ADC_ConvertedValueBuffer[];


extern SCH_TCB UartTxTcb;
extern SCH_TCB UartRxTcb;
extern SCH_TCB WdtClearTcb;
extern SCH_TCB LedTestTcb;
extern SCH_TCB AdcTreatmentTcb;
extern SCH_TCB AdcResultSendTcb;

/* Exported functions ------------------------------------------------------- */

extern void Peripherals_Config(void);
extern void vUartSendData(void);
extern void vUartReceiveData(void);
extern void vAdcResultSend(void);
extern void vAdcTreatment(void);
extern void vEepromWrite(void);
extern void vWdtClear(void);
extern void vLedTest(void);


/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/
