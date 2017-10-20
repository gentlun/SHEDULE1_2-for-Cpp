

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
  //#define USE_FULL_ASSERT    (1) �˶����� ��stm8s_conf.h����
  //ע�ʹ˾����ڵ��ÿ⺯��ʱ���ټ����ڲ����Ƿ���Ч������ʡ������Դ������д��ʼ��ʱ�򿪣��ڳ�ʼ���ɹ���ע�͵�

  /* System Clocks Configuration */
  RCC_Config();            //ʹ���ڲ�����ʱ��72M,   ϵͳ����Χʱ������

  /* NVIC Config �����ж����ú�ʹ�� */
  NVIC_Config();

  /* GPIO Config */
  GPIO_Config();
  /* Configure all unused GPIO port pins in Analog Input mode (floating input
   trigger OFF), this will reduce the power consumption and increase the device
   immunity against EMI/EMC *************************************************/

  TIM1_Config();
	
//TIM2_Config();

//	TIM3_Config();

  SysTick_Config(7200 - 1); //����ֵ,��� 0xffffff;  ϵͳʱ��Ƶ�� 72000000/7200=10kHz,0.1ms��ʱ.
  /* Select HCLK/8 as SysTick clock source */
  //SysTick_CLKSourceConfig(SysTick_CLKSource_HCLK_Div8);

  USART1_Config();

//USART3_Config();
		
  ADC_Config();

  //DAC_Config();

  IWDG_Config();  //�����������öϵ�,��Ϊ�ڶϵ�ʱ,����������,���¸�λ

  WWDG_Config();  //�����������öϵ�,��Ϊ�ڶϵ�ʱ,����������,���¸�λ
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
  SystemInit();     //����ϵͳʱ��Ϊ�ڲ�����ʱ��72M

  RCC_LSICmd(ENABLE);

  /* PCLK1=APB1=HCLK/4 ������Χģ��ʱ��,Ĭ��Ƶ����HCLK/2,���� DAC,PWR,BKP,bxCAN,USB,I2C1_2,USART2_5SPI2_3/I2S,
  WWDG,RTC,TM2_7 ���忴datasheet��ϵͳ�ṹ PCLK1=APB1 PCLK2=APB2 */
  RCC_PCLK1Config(RCC_HCLK_Div2);

  /* PCLK2=APB2=HCLK/1 ������Χģ��ʱ��,Ĭ��Ƶ����HCLK,���� ADC1_3,USART1,SPI1,TIM1,TIM8,GPIOA_G,EXTI,AFIO */
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
  
  GPIO_WriteBit(GPIOA,GPIO_Pin_9 ,Bit_SET);//��ʼ��Ϊ�ߵ�ƽ
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

  /* USART ������ز������� */
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

  //	USART_ITConfig(USART1, USART_IT_TC, ENABLE);		//ʹ�ܷ������ж�,Ҫ��Ҫ�����жϿ����п�ȫ���ж�
  //ʹ�ܺ���������һ���ж�

  USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
}

static void USART3_Config(void)
{
  USART_InitTypeDef USART_InitStructure;
  GPIO_InitTypeDef GPIO_InitStructure;

  /* GPIOB Configuration:Pin10->TXT */
  GPIO_WriteBit(GPIOB,GPIO_Pin_10 ,Bit_SET);//��ʼ��Ϊ�ߵ�ƽ
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOB, &GPIO_InitStructure);

  /* GPIOB Configuration:Pin11->RXT */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOB, &GPIO_InitStructure);

  /* USART ������ز������� */
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

  //	USART_ITConfig(USART3, USART_IT_TC, ENABLE);		//ʹ�ܷ������ж�,Ҫ��Ҫ�����жϿ����п�ȫ���ж�
  //ʹ�ܺ���������һ���ж�

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
  TIM_TimeBaseStructure.TIM_Period = 720-1; //�趨�������Զ���װֵ720 ����Ϊ10uS
  TIM_TimeBaseStructure.TIM_Prescaler = 0; //���Ƶ��ֵ0������Ƶ��ABP2ʱ��=72MHz��
  TIM_TimeBaseStructure.TIM_ClockDivision = 0; //
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up; //����ģʽѡ��_Up���ϼ���ģʽ��_Down���¼���ģʽ��_CenterAligned1�������ģʽ1��_CenterAligned2�������ģʽ2��_CenterAligned3�������ģʽ3
  TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;        //���� ���� ����ֵ

  TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure);

  /* Channel 1��2 Configuration in PWM mode ͨ��1��2��PWM */
  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1; //PWMģʽ2��PWMģʽ1������PWMģʽ2��ռ�ձ����űȽϼĴ���CCRx��ֵ���Ӷ�����PWMģʽ1��ռ�ձ����űȽϼĴ���CCRx��ֵ��С������
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; //����ͨ����Ч OC1=PA8;OC2=PA9
  TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Enable; //����ͨ��Ҳ��Ч OC1N=PB13;OC2N=PB14
  TIM_OCInitStructure.TIM_Pulse = 360;         //ռ��ʱ�� ��������������෴
  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High; //OC1������ԣ�Low����͵�ƽ��Ч��High����ߵ�ƽ��Ч ���ɻ��򣩣������������й�ϵ��ע���ͨ��Low���������Ҫ���������������������ΪHigh
  TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCNPolarity_High;    //OC1N�����˵ļ��ԣ�Low����͵�ƽ��Ч��High����ߵ�ƽ��Ч ���ɻ��򣩣������������й�ϵ��ע���ͨ��Low���������Ҫ���������������������ΪHigh
  TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Set; //����״̬�µķǹ���״̬
  TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCIdleState_Reset; //

  TIM_OC1Init(TIM1, &TIM_OCInitStructure);       //����ʼ������TIMxͨ��1
//TIM_OC2Init(TIM1, &TIM_OCInitStructure);       //����ʼ������TIMxͨ��2

  /* ����ɲ���������Ĵ���*/
  TIM_BDTRInitStruct.TIM_OSSRState = TIM_OSSRState_Enable;  //TIM_OSSRState ����������ģʽ�·ǹ���״̬ѡ��
  TIM_BDTRInitStruct.TIM_OSSIState = TIM_OSSIState_Enable;  //TIM_OSSIState ����������ģʽ�·ǹ���״̬ѡ��
  TIM_BDTRInitStruct.TIM_LOCKLevel = TIM_LOCKLevel_OFF;  //TIM_LOCKLevel ����������ƽ������������ƽ1
  TIM_BDTRInitStruct.TIM_DeadTime = 0;//0x2B;                    //����ʱ��Լ600ns(597.2222223)=43*(1/72MHz)
  TIM_BDTRInitStruct.TIM_Break = TIM_Break_Disable;       //TIM1 ɲ�����벻ʹ��
  TIM_BDTRInitStruct.TIM_BreakPolarity = TIM_BreakPolarity_Low;  //TIM1 ɲ������ܽż���
  TIM_BDTRInitStruct.TIM_AutomaticOutput = TIM_AutomaticOutput_Enable; //TIM1_AutomaticOutput �Զ����ʹ��
  TIM_BDTRConfig(TIM1, &TIM_BDTRInitStruct); //����ɲ�����ԣ�����ʱ�䣬����ƽ��OSSI��OSSR ״̬�� AOE���Զ����ʹ�ܣ�

  TIM_OC1PreloadConfig(TIM1, TIM_OCPreload_Enable);

  TIM_ARRPreloadConfig(TIM1, ENABLE);
	
//TIM_ClearITPendingBit(TIM1, TIM_IT_Update);

//TIM_ITConfig(TIM1, TIM_IT_Update, ENABLE);  //ʹ�ܸ����ж�

  /* TIM1 counter enable����ʱ�� */	
  TIM_Cmd(TIM1, ENABLE);

  /* TIM1 Main Output Enable ʹ��TIM1����������*/
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

  TIM_TimeBaseStructure.TIM_Period = 10000 - 1; //�Զ�����ֵ,10MS
  TIM_TimeBaseStructure.TIM_Prescaler = 36 - 1;     //Ԥ��Ƶֵ(�ڶ���Ԥ��Ƶ),36M/36=1M,��Ӧ1US
                                                    //  TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;//ʱ�ӷ�Ƶ,�ڴ�������Ч?
  TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;    //�ظ������Ĵ���,���ظ����ٴμ�������Ų����ж�
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up; //����ģʽ:����,����,���Ķ���1_3

  TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);

  /* Prescaler configuration Ԥ��Ƶֵ,����һ�����ü��� */
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

  TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);  //ʹ�ܸ����ж�

//TIM_OC1PreloadConfig(TIM2, TIM_OCPreload_Enable);

  TIM_ARRPreloadConfig(TIM2, ENABLE); //�Զ�����ֵԤ����ʹ��

  /* TIM2 enable counter */
  TIM_Cmd(TIM2, ENABLE);  //ʹ�ܼ���
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
  TIM_OCInitStructure.TIM_Pulse = 1000 / 2; //CCR1_Val;//ռ�ձ�
  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;

  TIM_OC1Init(TIM3, &TIM_OCInitStructure);

  TIM_OC1PreloadConfig(TIM3, TIM_OCPreload_Enable);   //Ԥ����ʹ��

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

  TIM_ARRPreloadConfig(TIM3, ENABLE); //�Զ�����ֵԤ����ʹ��

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
//  DAC_InitStructure.DAC_Trigger = DAC_Trigger_T6_TRGO;//ͬTIM6����
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
  ADC_InitStructure.ADC_NbrOfChannel = 2; //ɨ��ͨ���� ������ 1-16֮��
  ADC_Init(ADC1, &ADC_InitStructure);

  //ADC������ʱ�Ӳ��ó���14MHz��������PCLK2����Ƶ����
  /* ADC1 regular channel10 configuration */
  //������������ָ����ͨ����Ҫִ��ɨ���˳��(1-16),��ͨ��10��������Ϊ1,�����ȿ�ʼ����ͨ��10.
  //���ֻ�õ�ͨ��1 5 9,������ö�Ӧ˳��Ϊ 1 2 3,������,����ɨ��Ϳ�������û��ʹ�õ�ͨ��
  ADC_RegularChannelConfig(ADC1, ADC_Channel_0, 1, ADC_SampleTime_239Cycles5);
  ADC_RegularChannelConfig(ADC1, ADC_Channel_1, 2, ADC_SampleTime_239Cycles5);
//ADC_RegularChannelConfig(ADC1, ADC_Channel_2, 3, ADC_SampleTime_239Cycles5);
	
  /* Enable ADC1 DMA */
  ADC_DMACmd(ADC1, ENABLE);

  /* Enable ADC1 */
  ADC_Cmd(ADC1, ENABLE);  //��һ��ֻ�ǽ�ADCģ�黽��,֮���������ת��

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
  GPIO_Write(GPIOA, 0x0000);               //��ʼ��Ϊ�͵�ƽ
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_All;//GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;

  GPIO_Init(GPIOA, &GPIO_InitStructure);

	
  /* GPIOB Configuration:Pin4, 5, 6 and 7 as alternate function push-pull */
  GPIO_Write(GPIOB, 0x0000);               //��ʼ��Ϊ�͵�ƽ
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_All; //GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;

  GPIO_Init(GPIOB, &GPIO_InitStructure);

  /* GPIOC Configuration:Pin13 as alternate function push-pull */
  GPIO_Write(GPIOC, 0x0000);               //��ʼ��Ϊ�͵�ƽ
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_All; //GPIO_Pin_13;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;

  GPIO_Init(GPIOC, &GPIO_InitStructure);
}



static void IWDG_Config(void)
{
  /* Enable write access to IWDG_PR and IWDG_RLR registers */
  IWDG_WriteAccessCmd(IWDG_WriteAccess_Enable);
  
  /* IWDG counter clock: 40KHz(LSI) / 32 = 1.25 KHz ��ӦT=0.8MS */
  IWDG_SetPrescaler(IWDG_Prescaler_32);   //���256��Ƶ

  /* Set counter reload value to 31 */
  IWDG_SetReload(32 - 1);       //32*0.8=25.6MS  

  /* Disable write access to IWDG_PR and IWDG_RLR registers */
  IWDG_WriteAccessCmd(IWDG_WriteAccess_Disable);

  /* Reload IWDG counter */
//	IWDG_ReloadCounter();

  /* Enable IWDG (the LSI oscillator will be enabled by hardware) */
  IWDG_Enable();  //��ѡ�������п�����ΪӲ��ʹ��,�����������öϵ�,��Ϊ�ڶϵ�ʱ,����������,���¸�λ
}


static void WWDG_Config(void)
{
  /* WWDG counter clock: PCLK1=APB1=HCLK/4;  18M/(4096*WWDG_Prescaler_4) = 1098.6Hz ��ӦT=0.91MS */
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

  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);     //�����ж����ȼ�����ԭ��,����4λ������ռ���ȼ�

  /* Enable the TIM1 gloabal Interrupt */
  NVIC_InitStructure.NVIC_IRQChannel = TIM1_UP_IRQn;         //�����ж�ͨ��
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;   //��ռ���ȼ�,0-15
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;      //��Ӧ���ȼ�,0-15,�����ȼ���������Ϊ4λ
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;         //ʹ���ж�,��� NVIC_Priority_Table

  NVIC_Init(&NVIC_InitStructure);

  /* Enable the TIM2 gloabal Interrupt */
  NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;         //�����ж�ͨ��
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;   //��ռ���ȼ�,0-15
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;      //��Ӧ���ȼ�,0-15,�����ȼ���������Ϊ4λ
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;         //ʹ���ж�,��� NVIC_Priority_Table

  NVIC_Init(&NVIC_InitStructure);

  /* Enable the USART1 Interrupt */
  NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;           //�����ж�ͨ��
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 14; //��ռ���ȼ�,0-15
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;      //��Ӧ���ȼ�,0-15,�����ȼ���������Ϊ4λ
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;         //ʹ���ж�,��� NVIC_Priority_Table

  NVIC_Init(&NVIC_InitStructure);

}




/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/
