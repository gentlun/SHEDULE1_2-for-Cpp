

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x.h"
#include "user.h"
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define ADC1_DR_Address    ((u32)0x4001244C)


/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/

static void RCC_Config(void);
static void USART1_Config(void);
static void USART3_Config(void);
static void TIM1_Config(void);
static void TIM2_Config(void);
static void TIM3_Config(void);
static void ADC_Config(void);
static void GPIO_Config(void);
static void IWDG_Config(void);
static void WWDG_Config(void);
static void NVIC_Config(void);


/* Private functions ---------------------------------------------------------*/

void Peripherals_Config(void)
{
  //#define USE_FULL_ASSERT    (1) 此定义在 “stm8s_conf.h”中
  //注释此句则在调用库函数时不再检查入口参数是否有效，但节省大量资源，可在写初始化时打开，在初始化成功后注释掉

  /* System Clocks Configuration */
  RCC_Config();            //使用内部高速时钟72M,   系统和外围时钟配置

  /* NVIC Config 各种中断配置和使能 */
  NVIC_Config();

  /* GPIO Config */
  GPIO_Config();
  /* Configure all unused GPIO port pins in Analog Input mode (floating input
   trigger OFF), this will reduce the power consumption and increase the device
   immunity against EMI/EMC *************************************************/

  TIM1_Config();
	
//TIM2_Config();

//	TIM3_Config();

  SysTick_Config(7200 - 1); //重载值,最大 0xffffff;  系统时钟频率 72000000/7200=10kHz,0.1ms定时.
  /* Select HCLK/8 as SysTick clock source */
  //SysTick_CLKSourceConfig(SysTick_CLKSource_HCLK_Div8);

  USART1_Config();

//USART3_Config();
		
  ADC_Config();

  //DAC_Config();

  IWDG_Config();  //开启后不能设置断点,因为在断点时,狗还工作着,导致复位

  WWDG_Config();  //开启后不能设置断点,因为在断点时,狗还工作着,导致复位
}

/**
  * @brief  Configures the different system clocks.
  * @param  None
  * @retval None
  */
static void RCC_Config(void)
{
  /* Setup the microcontroller system. Initialize the Embedded Flash Interface,  
     initialize the PLL and update the SystemFrequency variable. */
  SystemInit();     //配置系统时钟为内部高速时钟72M

  RCC_LSICmd(ENABLE);

  /* PCLK1=APB1=HCLK/4 低速外围模块时钟,默认频率是HCLK/2,包括 DAC,PWR,BKP,bxCAN,USB,I2C1_2,USART2_5SPI2_3/I2S,
  WWDG,RTC,TM2_7 具体看datasheet的系统结构 PCLK1=APB1 PCLK2=APB2 */
  RCC_PCLK1Config(RCC_HCLK_Div2);

  /* PCLK2=APB2=HCLK/1 高速外围模块时钟,默认频率是HCLK,包括 ADC1_3,USART1,SPI1,TIM1,TIM8,GPIOA_G,EXTI,AFIO */
  RCC_PCLK2Config(RCC_HCLK_Div1);

    /* Enable DMA clock */
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);

  RCC_APB1PeriphClockCmd(	 RCC_APB1Periph_TIM2  | 
                           RCC_APB1Periph_TIM3  | 
                           RCC_APB1Periph_TIM4  | 
                           RCC_APB1Periph_TIM5  | 
                           RCC_APB1Periph_TIM6  | 
                           RCC_APB1Periph_TIM7  | 
                           //RCC_APB1Periph_TIM12 | 
                           //RCC_APB1Periph_TIM13 | 
                           //RCC_APB1Periph_TIM14 |                             
                           //RCC_APB1Periph_SPI2  | 
                           //RCC_APB1Periph_SPI3  | 
                           RCC_APB1Periph_USART2| 
                           RCC_APB1Periph_USART3	|
                           //RCC_APB1Periph_UART4 | 
                           //RCC_APB1Periph_UART5 | 
                           //RCC_APB1Periph_I2C1  | 
                           //RCC_APB1Periph_I2C2  | 
                           //RCC_APB1Periph_USB   | 
                           //RCC_APB1Periph_CAN1  | 
                           //RCC_APB1Periph_CAN2  | 
                           //RCC_APB1Periph_BKP   | 
                           //RCC_APB1Periph_PWR   | 
                           RCC_APB1Periph_DAC   | 
                           //RCC_APB1Periph_CEC   |
                           RCC_APB1Periph_WWDG  , 
                           ENABLE);

  RCC_APB2PeriphClockCmd(  RCC_APB2Periph_AFIO  |
                           RCC_APB2Periph_GPIOA |
                           RCC_APB2Periph_GPIOB |
                           RCC_APB2Periph_GPIOC |
                           RCC_APB2Periph_GPIOD |
                           RCC_APB2Periph_GPIOE |
                           RCC_APB2Periph_GPIOF |                           
                           RCC_APB2Periph_ADC1  |
                           RCC_APB2Periph_ADC2  |
                           RCC_APB2Periph_TIM1  |
                           RCC_APB2Periph_SPI1  |
                           RCC_APB2Periph_TIM8  |
                           RCC_APB2Periph_USART1  |
                           RCC_APB2Periph_ADC3  |
                           //RCC_APB2Periph_TIM15 |
                           //RCC_APB2Periph_TIM16 |
                           //RCC_APB2Periph_TIM17 |
                           //RCC_APB2Periph_TIM9  |
                           //RCC_APB2Periph_TIM10 |
                           //RCC_APB2Periph_TIM11 |
                           RCC_APB2Periph_GPIOG ,
                           ENABLE);
}


static void USART1_Config(void)
{
  USART_InitTypeDef USART_InitStructure;
  GPIO_InitTypeDef GPIO_InitStructure;

  /* GPIOA Configuration:Pin9->TXT */
  
  GPIO_WriteBit(GPIOA,GPIO_Pin_9 ,Bit_SET);//初始化为高电平
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOA, &GPIO_InitStructure);

  /* GPIOA Configuration:Pin10->RXT */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOA, &GPIO_InitStructure);

  /* USART 串口相关参数配置 */
  USART_InitStructure.USART_BaudRate = 9600;
  USART_InitStructure.USART_WordLength = USART_WordLength_8b;
  USART_InitStructure.USART_StopBits = USART_StopBits_1;
  USART_InitStructure.USART_Parity = USART_Parity_No;
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;

  /* USART configuration */
  USART_Init(USART1, &USART_InitStructure);

  /* Enable USART */
  USART_Cmd(USART1, ENABLE);

  USART_ClearITPendingBit(USART1, USART_IT_TC);

  USART_ClearITPendingBit(USART1, USART_IT_RXNE);

  //	USART_ITConfig(USART1, USART_IT_TC, ENABLE);		//使能发送完中断,要先要向量中断控制中开全局中断
  //使能后立即产生一次中断

  USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
}

static void USART3_Config(void)
{
  USART_InitTypeDef USART_InitStructure;
  GPIO_InitTypeDef GPIO_InitStructure;

  /* GPIOB Configuration:Pin10->TXT */
  GPIO_WriteBit(GPIOB,GPIO_Pin_10 ,Bit_SET);//初始化为高电平
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOB, &GPIO_InitStructure);

  /* GPIOB Configuration:Pin11->RXT */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOB, &GPIO_InitStructure);

  /* USART 串口相关参数配置 */
  USART_InitStructure.USART_BaudRate = 9600;
  USART_InitStructure.USART_WordLength = USART_WordLength_8b;
  USART_InitStructure.USART_StopBits = USART_StopBits_1;
  USART_InitStructure.USART_Parity = USART_Parity_No;
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;

  /* USART configuration */
  USART_Init(USART3, &USART_InitStructure);

  /* Enable USART */
  USART_Cmd(USART3, ENABLE);

  USART_ClearITPendingBit(USART3, USART_IT_TC);

  USART_ClearITPendingBit(USART3, USART_IT_RXNE);

  //	USART_ITConfig(USART3, USART_IT_TC, ENABLE);		//使能发送完中断,要先要向量中断控制中开全局中断
  //使能后立即产生一次中断

  USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);
}

static void TIM1_Config(void)
{
	GPIO_InitTypeDef  			 GPIO_InitStructure;	
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef        TIM_OCInitStructure;
	TIM_BDTRInitTypeDef      TIM_BDTRInitStruct;	


  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8 ;//| GPIO_Pin_9 | GPIO_Pin_10;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;

  GPIO_Init(GPIOA, &GPIO_InitStructure);
	
  /* Time Base configuration */
  TIM_TimeBaseStructure.TIM_Period = 720-1; //设定计数器自动重装值720 周期为10uS
  TIM_TimeBaseStructure.TIM_Prescaler = 0; //与分频器值0，不分频，ABP2时钟=72MHz；
  TIM_TimeBaseStructure.TIM_ClockDivision = 0; //
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up; //计数模式选择。_Up向上计数模式；_Down向下计数模式；_CenterAligned1中央对齐模式1；_CenterAligned2中央对齐模式2；_CenterAligned3中央对齐模式3
  TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;        //设置 周期 计数值

  TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure);

  /* Channel 1、2 Configuration in PWM mode 通道1、2的PWM */
  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1; //PWM模式2和PWM模式1的区别：PWM模式2：占空比随着比较寄存器CCRx的值增加而增大，PWM模式1：占空比随着比较寄存器CCRx的值减小而增大
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; //正向通道有效 OC1=PA8;OC2=PA9
  TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Enable; //反向通道也有效 OC1N=PB13;OC2N=PB14
  TIM_OCInitStructure.TIM_Pulse = 360;         //占空时间 互补的输出正好相反
  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High; //OC1输出极性，Low输出低电平有效；High输出高电平有效 （可换向）；与死区有密切关系，注意短通（Low），如果需要插入死区需设置输出极性为High
  TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCNPolarity_High;    //OC1N互补端的极性，Low输出低电平有效；High输出高电平有效 （可换向）；与死区有密切关系，注意短通（Low），如果需要插入死区需设置输出极性为High
  TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Set; //空闲状态下的非工作状态
  TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCIdleState_Reset; //

  TIM_OC1Init(TIM1, &TIM_OCInitStructure);       //数初始化外设TIMx通道1
//TIM_OC2Init(TIM1, &TIM_OCInitStructure);       //数初始化外设TIMx通道2

  /* 设置刹车和死区寄存器*/
  TIM_BDTRInitStruct.TIM_OSSRState = TIM_OSSRState_Enable;  //TIM_OSSRState 设置在运行模式下非工作状态选项
  TIM_BDTRInitStruct.TIM_OSSIState = TIM_OSSIState_Enable;  //TIM_OSSIState 设置在运行模式下非工作状态选项
  TIM_BDTRInitStruct.TIM_LOCKLevel = TIM_LOCKLevel_OFF;  //TIM_LOCKLevel 设置了锁电平参数——锁电平1
  TIM_BDTRInitStruct.TIM_DeadTime = 0;//0x2B;                    //死区时间约600ns(597.2222223)=43*(1/72MHz)
  TIM_BDTRInitStruct.TIM_Break = TIM_Break_Disable;       //TIM1 刹车输入不使能
  TIM_BDTRInitStruct.TIM_BreakPolarity = TIM_BreakPolarity_Low;  //TIM1 刹车输入管脚极性
  TIM_BDTRInitStruct.TIM_AutomaticOutput = TIM_AutomaticOutput_Enable; //TIM1_AutomaticOutput 自动输出使能
  TIM_BDTRConfig(TIM1, &TIM_BDTRInitStruct); //设置刹车特性，死区时间，锁电平，OSSI，OSSR 状态和 AOE（自动输出使能）

  TIM_OC1PreloadConfig(TIM1, TIM_OCPreload_Enable);

  TIM_ARRPreloadConfig(TIM1, ENABLE);
	
//TIM_ClearITPendingBit(TIM1, TIM_IT_Update);

//TIM_ITConfig(TIM1, TIM_IT_Update, ENABLE);  //使能更新中断

  /* TIM1 counter enable开定时器 */	
  TIM_Cmd(TIM1, ENABLE);

  /* TIM1 Main Output Enable 使能TIM1外设的主输出*/
  TIM_CtrlPWMOutputs(TIM1, ENABLE);
}


static void TIM2_Config(void)
{
  TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;

  /* ---------------------------------------------------------------
  TIM2 Configuration: Output Compare Timing Mode:
  TIM2CLK = 36 MHz, Prescaler = 4, TIM2 counter clock = 7.2 MHz
  CC1 update rate = TIM2 counter clock / CCR1_Val = 146.48 Hz
  CC2 update rate = TIM2 counter clock / CCR2_Val = 219.7 Hz
  CC3 update rate = TIM2 counter clock / CCR3_Val = 439.4 Hz
  CC4 update rate = TIM2 counter clock / CCR4_Val =  878.9 Hz
  --------------------------------------------------------------- */

  /* Time base configuration */

  TIM_TimeBaseStructure.TIM_Period = 10000 - 1; //自动重载值,10MS
  TIM_TimeBaseStructure.TIM_Prescaler = 36 - 1;     //预分频值(第二次预分频),36M/36=1M,对应1US
                                                    //  TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;//时钟分频,在此设置无效?
  TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;    //重复计数寄存器,如重复多少次计数溢出才产生中断
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up; //计数模式:向上,向下,中心对齐1_3

  TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);

  /* Prescaler configuration 预分频值,在上一步设置即可 */
  //  TIM_PrescalerConfig(TIM2, 4, TIM_PSCReloadMode_Immediate);

  //  /* Output Compare Timing Mode configuration: Channel1 */
  //  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_Timing;
  //  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  //  TIM_OCInitStructure.TIM_Pulse = CCR1_Val;
  //  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
  //
  //  TIM_OC1Init(TIM2, &TIM_OCInitStructure);
  //
  //  TIM_OC1PreloadConfig(TIM2, TIM_OCPreload_Disable);
  //
  //  /* Output Compare Timing Mode configuration: Channel2 */
  //  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  //  TIM_OCInitStructure.TIM_Pulse = CCR2_Val;
  //
  //  TIM_OC2Init(TIM2, &TIM_OCInitStructure);
  //
  //  TIM_OC2PreloadConfig(TIM2, TIM_OCPreload_Disable);
  //
  //  /* Output Compare Timing Mode configuration: Channel3 */
  //  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  //  TIM_OCInitStructure.TIM_Pulse = CCR3_Val;
  //
  //  TIM_OC3Init(TIM2, &TIM_OCInitStructure);
  //
  //  TIM_OC3PreloadConfig(TIM2, TIM_OCPreload_Disable);
  //
  //  /* Output Compare Timing Mode configuration: Channel4 */
  //  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  //  TIM_OCInitStructure.TIM_Pulse = CCR4_Val;
  //
  //  TIM_OC4Init(TIM2, &TIM_OCInitStructure);
  //
  //  TIM_OC4PreloadConfig(TIM2, TIM_OCPreload_Disable);
  //
  //  /* TIM IT enable */
  //  TIM_ITConfig(TIM2, TIM_IT_CC1 | TIM_IT_CC2 | TIM_IT_CC3 | TIM_IT_CC4, ENABLE);

  TIM_ClearITPendingBit(TIM2, TIM_IT_Update);

  TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);  //使能更新中断

//TIM_OC1PreloadConfig(TIM2, TIM_OCPreload_Enable);

  TIM_ARRPreloadConfig(TIM2, ENABLE); //自动重载值预加载使能

  /* TIM2 enable counter */
  TIM_Cmd(TIM2, ENABLE);  //使能计数
}


static void TIM3_Config(void)
{
  TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
  TIM_OCInitTypeDef TIM_OCInitStructure;
  GPIO_InitTypeDef GPIO_InitStructure;


  /* GPIOA Configuration:TIM3 Channel1, 2, 3 and 4 as alternate function push-pull */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;

  GPIO_Init(GPIOA, &GPIO_InitStructure);

  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1;
  GPIO_Init(GPIOB, &GPIO_InitStructure);

  /* -----------------------------------------------------------------------
  TIM3 Configuration: generate 4 PWM signals with 4 different duty cycles:
  TIM3CLK = 36 MHz, Prescaler = 0x0, TIM3 counter clock = 36 MHz
  TIM3 ARR Register = 999 => TIM3 Frequency = TIM3 counter clock/(ARR + 1)
  TIM3 Frequency = 36 KHz.
  TIM3 Channel1 duty cycle = (TIM3_CCR1/ TIM3_ARR)* 100 = 50%
  TIM3 Channel2 duty cycle = (TIM3_CCR2/ TIM3_ARR)* 100 = 37.5%
  TIM3 Channel3 duty cycle = (TIM3_CCR3/ TIM3_ARR)* 100 = 25%
  TIM3 Channel4 duty cycle = (TIM3_CCR4/ TIM3_ARR)* 100 = 12.5%
  ----------------------------------------------------------------------- */


  /* Time base configuration */
  TIM_TimeBaseStructure.TIM_Period = 999;     //36M/1000=36K
  TIM_TimeBaseStructure.TIM_Prescaler = 0;
  TIM_TimeBaseStructure.TIM_ClockDivision = 0;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;

  TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);

  /* PWM1 Mode configuration: Channel 1 */
  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_Pulse = 1000 / 2; //CCR1_Val;//占空比
  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;

  TIM_OC1Init(TIM3, &TIM_OCInitStructure);

  TIM_OC1PreloadConfig(TIM3, TIM_OCPreload_Enable);   //预加载使能

  /* PWM1 Mode configuration: Channel 2 */
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_Pulse = 800 / 2; //CCR2_Val;

  TIM_OC2Init(TIM3, &TIM_OCInitStructure);

  TIM_OC2PreloadConfig(TIM3, TIM_OCPreload_Enable);

  /* PWM1 Mode configuration: Channel 3 */
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_Pulse = 600 / 2; //CCR3_Val;

  TIM_OC3Init(TIM3, &TIM_OCInitStructure);

  TIM_OC3PreloadConfig(TIM3, TIM_OCPreload_Enable);

  /* PWM1 Mode configuration: Channel 4 */
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_Pulse = 400 / 2; //CCR4_Val;

  TIM_OC4Init(TIM3, &TIM_OCInitStructure);

  TIM_OC4PreloadConfig(TIM3, TIM_OCPreload_Enable);

  TIM_ARRPreloadConfig(TIM3, ENABLE); //自动重载值预加载使能

  /* TIM3 enable counter */
  TIM_Cmd(TIM3, ENABLE);
}
/**************************
void DAC_Config(void)
*************************/
//static void DAC_Configuration(void)
//{
//	GPIO_InitTypeDef GPIO_InitStructure;
//
//  /* Once the DAC channel is enabled, the corresponding GPIO pin is automatically
//     connected to the DAC converter. In order to avoid parasitic consumption,
//     the GPIO pin should be configured in analog */
//  GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_4;
//  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
//  GPIO_Init(GPIOA, &GPIO_InitStructure);
//
//	/* DAC Periph clock enable */
//  RCC_APB1PeriphClockCmd(RCC_APB1Periph_DAC, ENABLE);
//
///* TIM6 Configuration */
//  TIM_PrescalerConfig(TIM6, 0xF, TIM_PSCReloadMode_Update);
//  TIM_SetAutoreload(TIM6, 0xFF);
//  /* TIM6 TRGO selection */
//  TIM_SelectOutputTrigger(TIM6, TIM_TRGOSource_Update);
//
// /* DAC channel1 Configuration */
//  DAC_InitStructure.DAC_Trigger = DAC_Trigger_T6_TRGO;//同TIM6触发
//  DAC_InitStructure.DAC_WaveGeneration = DAC_WaveGeneration_None;
//  DAC_InitStructure.DAC_OutputBuffer = DAC_OutputBuffer_Disable;
//  DAC_Init(DAC_Channel_1, &DAC_InitStructure);
//
////  /* DMA2 channel3 configuration */
////  DMA_DeInit(DMA2_Channel3);
////  DMA_InitStructure.DMA_PeripheralBaseAddr = DAC_DHR8R1_Address;
////  DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)&Escalator8bit;
////  DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;
////  DMA_InitStructure.DMA_BufferSize = 6;
////  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
////  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
////  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
////  DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
////  DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
////  DMA_InitStructure.DMA_Priority = DMA_Priority_High;
////  DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
////  DMA_Init(DMA2_Channel3, &DMA_InitStructure);
////
////  /* Enable DMA2 Channel3 */
////  DMA_Cmd(DMA2_Channel3, ENABLE);
//
//  /* Enable DAC Channel1: Once the DAC channel1 is enabled, PA.04 is
//     automatically connected to the DAC converter. */
//  DAC_Cmd(DAC_Channel_1, ENABLE);
//
//  /* Enable DMA for DAC Channel1 */
////  DAC_DMACmd(DAC_Channel_1, ENABLE);
//
///* TIM6 enable counter */
//  TIM_Cmd(TIM6, ENABLE);
//}


static void ADC_Config(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
  ADC_InitTypeDef  ADC_InitStructure;
  DMA_InitTypeDef  DMA_InitStructure;


  /* Configure PA.00, PA.01 and PA.02 (ADC Channel 0, Channel 1 and Channel 2)
  as analog input ----------------------------------------------------------*/
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
  GPIO_Init(GPIOA, &GPIO_InitStructure);

/* DMA channel1 configuration ----------------------------------------------*/
  DMA_DeInit(DMA1_Channel1);
  DMA_InitStructure.DMA_PeripheralBaseAddr = ADC1_DR_Address;//ADC1->DR
  DMA_InitStructure.DMA_MemoryBaseAddr = (u32)&ADC_ConvertedValueBuffer;
  DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
  DMA_InitStructure.DMA_BufferSize = 2;
  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
  DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
  DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
  DMA_InitStructure.DMA_Priority = DMA_Priority_High;
  DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
  DMA_Init(DMA1_Channel1, &DMA_InitStructure);

  /* Enable DMA channel1 */
  DMA_Cmd(DMA1_Channel1, ENABLE);

  /* ADC1 configuration ------------------------------------------------------*/
  ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;
  ADC_InitStructure.ADC_ScanConvMode = ENABLE;
  ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;
  ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
  ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
  ADC_InitStructure.ADC_NbrOfChannel = 2; //扫描通道数 必须是 1-16之间
  ADC_Init(ADC1, &ADC_InitStructure);

  //ADC的输入时钟不得超过14MHz，它是由PCLK2经分频产生
  /* ADC1 regular channel10 configuration */
  //第三个参数是指定改通道所要执行扫描的顺序(1-16),如通道10可以设置为1,即最先开始采样通道10.
  //如果只用到通道1 5 9,则可配置对应顺序为 1 2 3,或其它,这样扫描就可以跳过没有使用的通道
  ADC_RegularChannelConfig(ADC1, ADC_Channel_0, 1, ADC_SampleTime_239Cycles5);
  ADC_RegularChannelConfig(ADC1, ADC_Channel_1, 2, ADC_SampleTime_239Cycles5);
//ADC_RegularChannelConfig(ADC1, ADC_Channel_2, 3, ADC_SampleTime_239Cycles5);
	
  /* Enable ADC1 DMA */
  ADC_DMACmd(ADC1, ENABLE);

  /* Enable ADC1 */
  ADC_Cmd(ADC1, ENABLE);  //第一次只是将ADC模块唤醒,之后才是启动转换

  /* Enable ADC1 reset calibaration register */
  ADC_ResetCalibration(ADC1);
  /* Check the end of ADC1 reset calibration register */
  while(ADC_GetResetCalibrationStatus(ADC1));

  /* Start ADC1 calibaration */
  ADC_StartCalibration(ADC1);
  /* Check the end of ADC1 calibration */
  while(ADC_GetCalibrationStatus(ADC1));

  /* Start ADC1 Software Conversion */
//ADC_SoftwareStartConvCmd(ADC1, ENABLE);

//  /* Test on DMA1 channel1 transfer complete flag */
//  while(!DMA_GetFlagStatus(DMA1_FLAG_TC1));
//  /* Clear DMA1 channel1 transfer complete flag */
//  DMA_ClearFlag(DMA1_FLAG_TC1);
}
/**
  * @brief  Configure the GPIOD Pins.
  * @param  None
  * @retval None
  */
static void GPIO_Config(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
  GPIO_PinRemapConfig ( GPIO_Remap_SWJ_JTAGDisable, ENABLE );
  /* GPIOA Configuration:Pin4, 5, 6 and 7 as alternate function push-pull */
  GPIO_Write(GPIOA, 0x0000);               //初始化为低电平
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_All;//GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;

  GPIO_Init(GPIOA, &GPIO_InitStructure);

	
  /* GPIOB Configuration:Pin4, 5, 6 and 7 as alternate function push-pull */
  GPIO_Write(GPIOB, 0x0000);               //初始化为低电平
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_All; //GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;

  GPIO_Init(GPIOB, &GPIO_InitStructure);

  /* GPIOC Configuration:Pin13 as alternate function push-pull */
  GPIO_Write(GPIOC, 0x0000);               //初始化为低电平
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_All; //GPIO_Pin_13;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;

  GPIO_Init(GPIOC, &GPIO_InitStructure);
}



static void IWDG_Config(void)
{
  /* Enable write access to IWDG_PR and IWDG_RLR registers */
  IWDG_WriteAccessCmd(IWDG_WriteAccess_Enable);
  
  /* IWDG counter clock: 40KHz(LSI) / 32 = 1.25 KHz 对应T=0.8MS */
  IWDG_SetPrescaler(IWDG_Prescaler_32);   //最大256分频

  /* Set counter reload value to 31 */
  IWDG_SetReload(32 - 1);       //32*0.8=25.6MS  

  /* Disable write access to IWDG_PR and IWDG_RLR registers */
  IWDG_WriteAccessCmd(IWDG_WriteAccess_Disable);

  /* Reload IWDG counter */
//	IWDG_ReloadCounter();

  /* Enable IWDG (the LSI oscillator will be enabled by hardware) */
  IWDG_Enable();  //在选项设置中可设置为硬件使能,开启后不能设置断点,因为在断点时,狗还工作着,导致复位
}


static void WWDG_Config(void)
{
  /* WWDG counter clock: PCLK1=APB1=HCLK/4;  18M/(4096*WWDG_Prescaler_4) = 1098.6Hz 对应T=0.91MS */
  WWDG_SetPrescaler(WWDG_Prescaler_4);   //

  WWDG_SetWindowValue(WWDG_WINDOW_VALUE);  

  WWDG_SetCounter(WWDG_COUNTER_INIT);

  /* Enable WWDG */
  WWDG_Enable(WWDG_COUNTER_INIT);  //
}

/**
  * @brief  Configure the nested vectored interrupt controller.
  * @param  None
  * @retval None
  */
static void NVIC_Config(void)
{

  NVIC_InitTypeDef NVIC_InitStructure;

  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);     //设置中断优先级分组原则,所有4位用于抢占优先级

  /* Enable the TIM1 gloabal Interrupt */
  NVIC_InitStructure.NVIC_IRQChannel = TIM1_UP_IRQn;         //设置中断通道
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;   //抢占优先级,0-15
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;      //响应优先级,0-15,两优先级共可设置为4位
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;         //使能中断,详见 NVIC_Priority_Table

  NVIC_Init(&NVIC_InitStructure);

  /* Enable the TIM2 gloabal Interrupt */
  NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;         //设置中断通道
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;   //抢占优先级,0-15
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;      //响应优先级,0-15,两优先级共可设置为4位
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;         //使能中断,详见 NVIC_Priority_Table

  NVIC_Init(&NVIC_InitStructure);

  /* Enable the USART1 Interrupt */
  NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;           //设置中断通道
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 14; //抢占优先级,0-15
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;      //响应优先级,0-15,两优先级共可设置为4位
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;         //使能中断,详见 NVIC_Priority_Table

  NVIC_Init(&NVIC_InitStructure);

}




/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/
