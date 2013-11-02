/* general control */

/*****************************************************************************/
/*  Module     : DSP_Main                                       Version 1.0  */
/*****************************************************************************/
/*                                                                           */
/*  Function   : Initialization for DA/AD system, using DMA. Initializes and */
/*               starts ADC0, ADC1, DAC1 and DAC2 and their DMA channels for */
/*               blockwise signalprocessing.                                 */
/*               DMA uses double buffering for more effiziency               */
/*                                                                           */
/*               This code is based on the example code                      */
/*               'DAC_SignalsGeneration' and 'ADC_Interleaved_DMAmode2'      */
/*               from MCD Application Team 19-September-2011                 */
/*                                                                           */
/*                                                                           */
/*  Procedures : main()                                                      */
/*                                                                           */
/*  Author     : I. Oesch                                                    */
/*                                                                           */
/*  History    : 10.09.2013  IO Created                                      */
/*                                                                           */
/*  File       : DSPMain.c                                                   */
/*                                                                           */
/*****************************************************************************/
/*  Berner Fachhochschule   *      Fachbereich EKT                           */
/*  TI Burgdorf             *      Digitale Signalverarbeitung               */
/*****************************************************************************/

/* imports */
#include "stm32f4_discovery.h"
#include "config.h"
#include "SignalProcessing.h"
#include "DSPMain.h"


/* module constant declaration */


// ADC 1-3 Baseadresses 0x40012000 - 0x400123FF
// DAC     Baseadress   0x40007400 - 0x400077FF

/* Definitiom of register adresses */
#define DAC_DHR12R2_ADDRESS    0x40007414
#define DAC_DHR8R1_ADDRESS     0x40007410

#define DAC_DHR12L1_ADDRESS    0x4000740C
#define DAC_DHR12L2_ADDRESS    0x40007418

#define ADC_DR1_ADDRESS    ((uint32_t)0x4001204C)
#define ADC_DR2_ADDRESS    ((uint32_t)0x4001214C)
#define ADC_DR3_ADDRESS    ((uint32_t)0x4001224C)

#define ADC_CDR_ADDRESS    ((uint32_t)0x40012308)


/* module type declaration */

/* module data declaration */



/* Buffers for DMA double buffering       */
/* ...a and ...b belongs together,        */
/* one will be used by DMA, the other     */
/* by signal processing, will be swapped  */
/* whenever DMA has finished its transfer */
uint16_t Buffer1_a[BLOCK_SIZE] = {0};
uint16_t Buffer1_b[BLOCK_SIZE] = {0};
uint16_t TxBuffer1_a[BLOCK_SIZE] = {0};
uint16_t TxBuffer1_b[BLOCK_SIZE] = {0};

#if NUMBER_OF_CHANNELS == 2
uint16_t Buffer2_a[BLOCK_SIZE] = {0};
uint16_t Buffer2_b[BLOCK_SIZE] = {0};
uint16_t TxBuffer2_a[BLOCK_SIZE] = {0};
uint16_t TxBuffer2_b[BLOCK_SIZE] = {0};
#endif



/* module procedure declaration */
void TIM2_Config(void);
void Interrupt_Config(void);
void ADC_Common_Config(void);
void ADC_Ch1_Config(void);
void ADC_Ch2_Config(void);
void DAC_Common_Config(void);
void DAC_Ch1_Config(void);
void DAC_Ch2_Config(void);


/*****************************************************************************/
/*  Procedure   : TIM12_Config                                               */
/*****************************************************************************/
/*                                                                           */
/*  Function    : Configures timer 12 channel 2 as clock source for          */
/*                AAF-Filters on PB15 (Allowed frequencies go from 1Hz to    */
/*                45kHz), clock source must be 100*fg                        */
/*                                                                           */
/*  Type        : Global                                                     */
/*                                                                           */
/*  Input Para  : Fg: Cut off frequency off AAF (Will be set to max or min   */
/*                    allowed values if out of range)                        */
/*                                                                           */
/*  Output Para : None                                                       */
/*                                                                           */
/*                                                                           */
/*  Author      : I. Oesch                                                   */
/*                                                                           */
/*  History     : 21.10.2013  IO Created                                     */
/*                                                                           */
/*****************************************************************************/
void TIM12_Config(unsigned int Fg)
{
    /* procedure data */
    GPIO_InitTypeDef GPIO_InitStructure;
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
    TIM_OCInitTypeDef  TIM_OCInitStructure;

    /* Compute the prescaler value */
    unsigned int RequiredDivider = (SYSCLK + Fg*50) / (Fg*100);

    unsigned int PrescalerValue = 0;
    unsigned int TimerPeriod = RequiredDivider-1;

    /* procedure code */

    /* Make value for Fg in-range of 1 to 45000Hz */
    if (Fg < 1) {
       Fg = 1;
    }
    if (Fg > 45000) {
       Fg = 45000;
    }

    /* TIM12 clock enable */
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM12, ENABLE);

    /* GPIOB clock enable */
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);

    /* GPIOB Configuration:  TIM12 CH2 (PB15) */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP ;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    /* Connect TIM3 pins to AF2 */
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource15, GPIO_AF_TIM12);

    /* Determine Prescaler Value, if divider is > 2^16 we need a prescaler */
    /* to be un teh suer side we use 2^15 as limit...                      */
    if (RequiredDivider > 32768) {
       /* Prescaler and timer divide by N+1 */
       PrescalerValue = RequiredDivider/32768;
       TimerPeriod = RequiredDivider/(PrescalerValue+1)-1;
    }

  /* Time base configuration */
  TIM_TimeBaseStructure.TIM_Period = TimerPeriod;
  TIM_TimeBaseStructure.TIM_Prescaler = PrescalerValue;
  TIM_TimeBaseStructure.TIM_ClockDivision = 0;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
  TIM_TimeBaseInit(TIM12, &TIM_TimeBaseStructure);

  /* PWM1 Mode configuration: Channel2 */
  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_Pulse = TimerPeriod/2;
  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
  TIM_OC2Init(TIM12, &TIM_OCInitStructure);

  TIM_OC2PreloadConfig(TIM12, TIM_OCPreload_Enable);

  TIM_ARRPreloadConfig(TIM12, ENABLE);

  /* TIM12 enable counter */
  TIM_Cmd(TIM12, ENABLE);

}
/*****************************************************************************/
/*  End         : TIM12_Config                                               */
/*****************************************************************************/


/*****************************************************************************/
/*  Procedure   : main                                                       */
/*****************************************************************************/
/*                                                                           */
/*  Function    : This is the main program, which initializes ADC, DAC, DMA  */
/*                Timer, GPIO and interrupts for signalprocessing and then   */
/*                waits in an endless loop.                                  */
/*                The real signal processing will hapen in the DMA-interrupt */
/*                routines.                                                  */
/*                                                                           */
/*  Type        : Global                                                     */
/*                                                                           */
/*  Input Para  : argc   Number of comandline arguments                      */
/*                argv   Vector with commandline arguments                   */
/*                                                                           */
/*  Output Para : Errorcode to operatingsystem                               */
/*                                                                           */
/*                                                                           */
/*  Author      : I. Oesch                                                   */
/*                                                                           */
/*  History     : 10.09.2013  IO Created                                     */
/*                                                                           */
/*****************************************************************************/
int main(int argc, char *argv[])
{
  /*!< At this stage the microcontroller clock setting is already configured, 
       this is done through SystemInit() function which is called from startup
       file (startup_stm32f4xx.s) before to branch to application main.
       To reconfigure the default setting of SystemInit() function, refer to
       system_stm32f4xx.c file
     */

    /* procedure data */
    GPIO_InitTypeDef GPIO_InitStructure;


    /* procedure code */

	/* Call the init function from the user */
	InitProcessing();
	TIM12_Config(4000);
	/* Configure the required interrupts */
	Interrupt_Config();

  /* GPIOD Peripherie clock enable */
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);

  /* Configure PD12, PD13, PD14 and PD15 (Leds) */
  /* and PD0, PD6, PD7, PD8, PD9 and PD10 Pin   */
  /* in output pushpull mode                    */
  GPIO_InitStructure.GPIO_Pin =      GPIO_Pin_0 | GPIO_Pin_6 | GPIO_Pin_7 | GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_10 | GPIO_Pin_12 | GPIO_Pin_13| GPIO_Pin_14| GPIO_Pin_15;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOD, &GPIO_InitStructure);

  /******************************************************************************/
  /*               ADCs interface clock, pin and DMA configuration              */
  /******************************************************************************/


   /* Initialize AD-System and Analog input pins */
   ADC_Common_Config();

   /* Initialize ADC 1 and its DMA */
   ADC_Ch1_Config();

#if NUMBER_OF_CHANNELS == 2
   /* Initialize ADC 2 and its DMA */
  ADC_Ch2_Config();
#endif


    /* Initialize DA-System and Analog output pins */
    DAC_Common_Config();

    /* Initialize DAC 1 and its DMA */
    DAC_Ch1_Config();

#if NUMBER_OF_CHANNELS == 2
    /* Initialize DAC 2 and its DMA */
    DAC_Ch2_Config();
#endif

    /* Initialize (and start) Sampling timer */
    TIM2_Config();

   /* Now the ADC/DAC system with its DMA and interrupts is running    */
   /* and since the signalprocessing is done in the interrupts we have */
   /* nothing more to do but wait...                                   */

   while (1)
   {
       /* Any not directly signal processing releated stuff may be done here */

	   /* Call user IDLE function */
	   IdleFunction();
   }
}
/*****************************************************************************/
/*  End         : main                                                       */
/*****************************************************************************/

/*****************************************************************************/
/*  Procedure   : TIM2_Config                                                */
/*****************************************************************************/
/*                                                                           */
/*  Function    : Configures timer 2 as trigger for ADC/DAC units, thus      */
/*                generating the samplingrate                                */
/*                                                                           */
/*  Type        : Global                                                     */
/*                                                                           */
/*  Input Para  : None                                                       */
/*                                                                           */
/*  Output Para : None                                                       */
/*                                                                           */
/*                                                                           */
/*  Author      : I. Oesch                                                   */
/*                                                                           */
/*  History     : 10.09.2013  IO Created                                     */
/*                                                                           */
/*****************************************************************************/
void TIM2_Config(void)
{
   /* procedure data */
   TIM_TimeBaseInitTypeDef    TIM_TimeBaseStructure;

   /* procedure code */

   /* Enable clock for Timer 2 */
   RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);

   /* Initialize Timebase structure (Fill with default values) */
   TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);

   /* Set Timer period to sampling frequency (Timer counts with SYSCLK) */
   /* Hint: (SYSCLK + FS/2)/Fs => SYSCLK/FS + 1/2 => Rounded Value without using of float */
   TIM_TimeBaseStructure.TIM_Period = ((long)(SYSCLK) + FS/2)/FS;

   /* No Prescaler */
   TIM_TimeBaseStructure.TIM_Prescaler = 0;
   TIM_TimeBaseStructure.TIM_ClockDivision = 0;

   /* We count up */
   TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;

   /* Setup timer with given values */
   TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);

   /* Generate trigger, whenever counter reloads (Defined by period) */
   TIM_SelectOutputTrigger(TIM2, TIM_TRGOSource_Update);
  
#ifdef SHOW_FS
   /* Generate interrupt, whenever counter reloads (Defined by period) */
   TIM_ITConfig 	(TIM2, TIM_IT_Update, ENABLE);
#endif

   /* Let timer 2 run... */
   TIM_Cmd(TIM2, ENABLE);
}
/*****************************************************************************/
/*  End         : TIM2_Config                                                */
/*****************************************************************************/

/*****************************************************************************/
/*  Procedure   : Interrupt_Config                                           */
/*****************************************************************************/
/*                                                                           */
/*  Function    : Configures interrupts as required                          */
/*                                                                           */
/*  Type        : Global                                                     */
/*                                                                           */
/*  Input Para  : None                                                       */
/*                                                                           */
/*  Output Para : None                                                       */
/*                                                                           */
/*                                                                           */
/*  Author      : I. Oesch                                                   */
/*                                                                           */
/*  History     : 10.09.2013  IO Created                                     */
/*                                                                           */
/*****************************************************************************/
void Interrupt_Config(void)
{
   /* procedure data */
   NVIC_InitTypeDef NVIC_InitStructure;

   /* procedure code */



   /* Configure the interruptcontroller priority management */
   NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);

   /* Enable the DMA2 Stream0 (ADC1) gloabal Interrupt */
   NVIC_InitStructure.NVIC_IRQChannel = DMA2_Stream0_IRQn;
   NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
   NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
   NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
   NVIC_Init(&NVIC_InitStructure);

#if NUMBER_OF_CHANNELS == 2
   /* Enable the DMA2 Stream2 (ADC2) gloabal Interrupt */
   NVIC_InitStructure.NVIC_IRQChannel = DMA2_Stream2_IRQn;
   NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
   NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
   NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
   NVIC_Init(&NVIC_InitStructure);
#endif

   /* Enable the DMA1 Stream5 (DAC1) gloabal Interrupt */
   NVIC_InitStructure.NVIC_IRQChannel = DMA1_Stream5_IRQn;
   NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
   NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
   NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
   NVIC_Init(&NVIC_InitStructure);

#if NUMBER_OF_CHANNELS == 2
   /* Enable the DMA1 Stream6 (DAC2) gloabal Interrupt */
   NVIC_InitStructure.NVIC_IRQChannel = DMA1_Stream6_IRQn;
   NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
   NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
   NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
   NVIC_Init(&NVIC_InitStructure);
#endif

#ifdef SHOW_FS
   /* Enable the TIM2 (Samplingrate timer) gloabal Interrupt */
   NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;
   NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
   NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
   NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
   NVIC_Init(&NVIC_InitStructure);
#endif
}
/*****************************************************************************/
/*  End         : Interrupt_Config                                           */
/*****************************************************************************/

/*****************************************************************************/
/*  Procedure   : ADC_Common_Config                                          */
/*****************************************************************************/
/*                                                                           */
/*  Function    : Configures the common features for both ADC                */
/*                                                                           */
/*  Type        : Global                                                     */
/*                                                                           */
/*  Input Para  : None                                                       */
/*                                                                           */
/*  Output Para : None                                                       */
/*                                                                           */
/*                                                                           */
/*  Author      : I. Oesch                                                   */
/*                                                                           */
/*  History     : 10.09.2013  IO Created                                     */
/*                                                                           */
/*****************************************************************************/
void ADC_Common_Config(void)
{
    /* procedure data */
    GPIO_InitTypeDef GPIO_InitStructure;
    ADC_CommonInitTypeDef ADC_CommonInitStructure;

    /* procedure code */

    /* Enable the peripheral clocks for ADC1, ADC2, Analog input and DMA */
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2 | RCC_AHB1Periph_GPIOC, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1 | RCC_APB2Periph_ADC2 |
                           RCC_APB2Periph_ADC3, ENABLE);

   /* Configure ADC Channel 11 & 12 (pin pc1 and pc2) as analog input */
   GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1 | GPIO_Pin_2;
   GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
   GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ;
   GPIO_Init(GPIOC, &GPIO_InitStructure);

   /* Configure ADCs as independent, define conversion timing */
   ADC_CommonInitStructure.ADC_Mode = ADC_Mode_Independent;
   ADC_CommonInitStructure.ADC_TwoSamplingDelay = ADC_TwoSamplingDelay_5Cycles;
   ADC_CommonInitStructure.ADC_DMAAccessMode = ADC_DMAAccessMode_Disabled;
   ADC_CommonInitStructure.ADC_Prescaler = ADC_Prescaler_Div2;
   ADC_CommonInit(&ADC_CommonInitStructure);
}
/*****************************************************************************/
/*  End         : ADC_Common_Config                                          */
/*****************************************************************************/

/*****************************************************************************/
/*  Procedure   : ADC_Ch1_Config                                             */
/*****************************************************************************/
/*                                                                           */
/*  Function    : Configures ADC 1 and its DMA channel                       */
/*                                                                           */
/*                DMA is configured in doublebuffering mode, thus there are  */
/*                two buffers used. One is used in the current transfer, the */
/*                other can be used by the signal processing algorithm.      */
/*                                                                           */
/*                When the DMA has filled its current buffer, an interrupt   */
/*                will be issued an the second buffer will be used for       */
/*                transfer. This swapping of buffers will continue for ever  */
/*                without any need of interaction by the user                */
/*                                                                           */
/*  Type        : Global                                                     */
/*                                                                           */
/*  Input Para  : None                                                       */
/*                                                                           */
/*  Output Para : None                                                       */
/*                                                                           */
/*                                                                           */
/*  Author      : I. Oesch                                                   */
/*                                                                           */
/*  History     : 10.09.2013  IO Created                                     */
/*                                                                           */
/*****************************************************************************/
void ADC_Ch1_Config(void)
{
    /* procedure data */
    DMA_InitTypeDef       DMA_InitStructure;
    ADC_InitTypeDef       ADC_InitStructure;

    /* procedure code */

    /* DMA2 Stream0 channel0 configuration */

	/* Channel 0 is connected to ADC1 */
	DMA_InitStructure.DMA_Channel = DMA_Channel_0;

	/* Read Values from ADC 1 Data register */
    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)ADC_DR1_ADDRESS;

    /* Write Values to Buffer1_a */
    DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)&Buffer1_a;
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;

    /* Define number of values to read int one block */
    DMA_InitStructure.DMA_BufferSize = BLOCK_SIZE;

    /* Define increment mode for Pheripherie and memory adress */
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;

    /* Size of data to transfer is 16bit */
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
    DMA_InitStructure.DMA_MemoryDataSize = DMA_PeripheralDataSize_HalfWord;

    /* We want to use double buffering */
    DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;

    /* Additional housekeeping stuff... */
    DMA_InitStructure.DMA_Priority = DMA_Priority_High;
    DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;
    DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_HalfFull;
    DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
    DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;

    /* Initialize DMA with given values */
    DMA_Init(DMA2_Stream0, &DMA_InitStructure);

    /* Define second buffer for doublebuffering */
    DMA_DoubleBufferModeConfig(DMA2_Stream0,  (uint32_t)&Buffer1_b, DMA_Memory_0);

    /* Enable doublebuffering */
    DMA_DoubleBufferModeCmd 	(DMA2_Stream0, ENABLE);

    /* Enable DMA-Interrupts */
    DMA_ITConfig 	(DMA2_Stream0, DMA_IT_TC, ENABLE);
    DMA_ITConfig 	(DMA2_Stream0, DMA_IT_HT, ENABLE);
    DMA_ITConfig 	(DMA2_Stream0, DMA_IT_TE, ENABLE);
    DMA_ITConfig 	(DMA2_Stream0, DMA_IT_FE, ENABLE);

    /* Enable DMA2 Stream0 */
    DMA_Cmd(DMA2_Stream0, ENABLE);

    /* ADC1 configuration */

    /* Single scan, 12 Bit resolution */
    ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;
    ADC_InitStructure.ADC_ScanConvMode = DISABLE;
    ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;
    ADC_InitStructure.ADC_NbrOfConversion = 1;

    /* External trigger by timer 2 */
    ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_Rising;
    ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_T2_TRGO;

    /* Left alignement (Result in upper 12 bit, lowest 4 bits will be 0) */
    ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Left;

    /* Initialize ADC with given values */
    ADC_Init(ADC1, &ADC_InitStructure);

    /* Use channel 12, 3-Cycle conversion */
    ADC_RegularChannelConfig(ADC1, ADC_Channel_12, 1, ADC_SampleTime_3Cycles);

    /* Enable DMA request after conversion */
    ADC_DMARequestAfterLastTransferCmd 	(ADC1, ENABLE);


    /* Enable ADC1 DMA */
    ADC_DMACmd(ADC1, ENABLE);

    /* Enable ADC1 */
    ADC_Cmd(ADC1, ENABLE);
}
/*****************************************************************************/
/*  End         : ADC_Ch1_Config                                             */
/*****************************************************************************/

/*****************************************************************************/
/*  Procedure   : ADC_Ch2_Config                                             */
/*****************************************************************************/
/*                                                                           */
/*  Function    : Configures ADC 2 and its DMA channel                       */
/*                                                                           */
/*                DMA is configured in doublebuffering mode, thus there are  */
/*                two buffers used. One is used in the current transfer, the */
/*                other can be used by the signal processing algorithm.      */
/*                                                                           */
/*                When the DMA has filled its current buffer, an interrupt   */
/*                will be issued an the second buffer will be used for       */
/*                transfer. This swapping of buffers will continue for ever  */
/*                without any need of interaction by the user                */
/*                                                                           */
/*  Type        : Global                                                     */
/*                                                                           */
/*  Input Para  : None                                                       */
/*                                                                           */
/*  Output Para : None                                                       */
/*                                                                           */
/*                                                                           */
/*  Author      : I. Oesch                                                   */
/*                                                                           */
/*  History     : 10.09.2013  IO Created                                     */
/*                                                                           */
/*****************************************************************************/
#if NUMBER_OF_CHANNELS == 2
void ADC_Ch2_Config(void)
{
    /* procedure data */
    DMA_InitTypeDef       DMA_InitStructure;
    ADC_InitTypeDef       ADC_InitStructure;

    /* procedure code */

    /* DMA2 Stream2 channel1 configuration */

    /* Channel 1 is connected to ADC2 */
    DMA_InitStructure.DMA_Channel = DMA_Channel_1;

    /* Read Values from ADC 2 Data register */
    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)ADC_DR2_ADDRESS;

    /* Write Values to Buffer2_a */
    DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)&Buffer2_a;
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;

    /* Define number of values to read int one block */
    DMA_InitStructure.DMA_BufferSize = BLOCK_SIZE;

    /* Define increment mode for Pheripherie and memory adress */
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;

    /* Size of data to transfer is 16bit */
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
    DMA_InitStructure.DMA_MemoryDataSize = DMA_PeripheralDataSize_HalfWord;

    /* We want to use double buffering */
    DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;

    /* Additional housekeeping stuff... */
    DMA_InitStructure.DMA_Priority = DMA_Priority_High;
    DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;
    DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_HalfFull;
    DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
    DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;

    /* Initialize DMA with given values */
    DMA_Init(DMA2_Stream2, &DMA_InitStructure);

    /* Define second buffer for doublebuffering */
    DMA_DoubleBufferModeConfig(DMA2_Stream2,  (uint32_t)&Buffer2_b, DMA_Memory_0);

    /* Enable doublebuffering */
    DMA_DoubleBufferModeCmd 	(DMA2_Stream2, ENABLE);

    /* Enable DMA-Interrupts */
    DMA_ITConfig 	(DMA2_Stream2, DMA_IT_TC, ENABLE);
    DMA_ITConfig 	(DMA2_Stream2, DMA_IT_HT, ENABLE);
    DMA_ITConfig 	(DMA2_Stream2, DMA_IT_TE, ENABLE);
    DMA_ITConfig 	(DMA2_Stream2, DMA_IT_FE, ENABLE);

    /* Enable DMA2 Stream2 */
    DMA_Cmd(DMA2_Stream2, ENABLE);

    /* ADC2 configuration */

    /* Single scan, 12 Bit resolution */
    ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;
    ADC_InitStructure.ADC_ScanConvMode = DISABLE;
    ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;

    /* External trigger by timer 2 */
    ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_Rising;
    ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_T2_TRGO;

    /* Left alignement (Result in upper 12 bit, lowest 4 bits will be 0) */
    ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Left;
    ADC_InitStructure.ADC_NbrOfConversion = 1;

    /* Initialize ADC with given values */
    ADC_Init(ADC2, &ADC_InitStructure);

    /* Use channel 11, 3-Cycle conversion */
    ADC_RegularChannelConfig(ADC2, ADC_Channel_11, 1, ADC_SampleTime_3Cycles);

    /* Enable DMA request after conversion */
    ADC_DMARequestAfterLastTransferCmd 	(ADC2, ENABLE);

    /* Enable ADC2 DMA */
    ADC_DMACmd(ADC2, ENABLE);

    /* Enable ADC2 **************************************************************/
    ADC_Cmd(ADC2, ENABLE);
}
#endif
/*****************************************************************************/
/*  End         : ADC_Ch1_Config                                             */
/*****************************************************************************/

/*****************************************************************************/
/*  Procedure   : DAC_Ch2_Config                                             */
/*****************************************************************************/
/*                                                                           */
/*  Function    : Configures DAC 2 and its DMA channel                       */
/*                                                                           */
/*                DMA is configured in doublebuffering mode, thus there are  */
/*                two buffers used. One is used in the current transfer, the */
/*                other can be used by the signal processing algorithm.      */
/*                                                                           */
/*                When the DMA has emptied its current buffer, an interrupt  */
/*                will be issued an the second buffer will be used for       */
/*                transfer. This swapping of buffers will continue for ever  */
/*                without any need of interaction by the user                */
/*                                                                           */
/*  Type        : Global                                                     */
/*                                                                           */
/*  Input Para  : None                                                       */
/*                                                                           */
/*  Output Para : None                                                       */
/*                                                                           */
/*                                                                           */
/*  Author      : I. Oesch                                                   */
/*                                                                           */
/*  History     : 10.09.2013  IO Created                                     */
/*                                                                           */
/*****************************************************************************/
#if NUMBER_OF_CHANNELS == 2
void DAC_Ch2_Config(void)
{
    /* procedure data */
    DMA_InitTypeDef DMA_InitStructure;
    DAC_InitTypeDef       DAC_InitStructure;

    /* procedure code */

   /* DMA1_Stream6 channel7 configuration */
  
   /* Channel 7 is connected to ADC2 */
   DMA_InitStructure.DMA_Channel = DMA_Channel_7;

   /* Write Values to DAC 2 Data register */
   DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)DAC_DHR12L2_ADDRESS;

   /* Read Values from TxBuffer2_a */
   DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)&TxBuffer2_a;
   DMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral;

   /* Define number of values to read int one block */
   DMA_InitStructure.DMA_BufferSize = BLOCK_SIZE;

   /* Define increment mode for Pheripherie and memory adress */
   DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
   DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;

   /* Size of data to transfer is 16bit */
   DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
   DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;

   /* We want to use double buffering */
   DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;

   /* Additional housekeeping stuff... */
   DMA_InitStructure.DMA_Priority = DMA_Priority_High;
   DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;
   DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_HalfFull;
   DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
   DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;

   /* Initialize DMA with given values */
   DMA_Init(DMA1_Stream6, &DMA_InitStructure);

   /* Define second buffer for doublebuffering */
  DMA_DoubleBufferModeConfig(DMA1_Stream6,  (uint32_t)&TxBuffer2_b, DMA_Memory_0);

  /* Enable doublebuffering */
  DMA_DoubleBufferModeCmd 	(DMA1_Stream6, ENABLE);

  /* Enable DMA-Interrupts */
  DMA_ITConfig 	(DMA1_Stream6, DMA_IT_TC, ENABLE);
  DMA_ITConfig 	(DMA1_Stream6, DMA_IT_HT, ENABLE);
  DMA_ITConfig 	(DMA1_Stream6, DMA_IT_TE, ENABLE);
  DMA_ITConfig 	(DMA1_Stream6, DMA_IT_FE, ENABLE);

  /* Enable DMA1 Stream6 */
  DMA_Cmd(DMA1_Stream6, ENABLE);

  /* DAC channel2 Configuration */

  /* External trigger by timer 2 */
  DAC_InitStructure.DAC_Trigger = DAC_Trigger_T2_TRGO;

  /*Normal DAC Mode, enable extra (analog)output buffer */
  DAC_InitStructure.DAC_WaveGeneration = DAC_WaveGeneration_None;
  DAC_InitStructure.DAC_OutputBuffer = DAC_OutputBuffer_Enable;

  /* Initialize DAC with given values */
  DAC_Init(DAC_Channel_2, &DAC_InitStructure);

  /* Enable DAC Channel2 */
  DAC_Cmd(DAC_Channel_2, ENABLE);

  /* Enable DMA for DAC Channel2 */
  DAC_DMACmd(DAC_Channel_2, ENABLE);
}
#endif
/*****************************************************************************/
/*  End         : DAC_Ch2_Config                                             */
/*****************************************************************************/

/*****************************************************************************/
/*  Procedure   : DAC_Common_Config                                          */
/*****************************************************************************/
/*                                                                           */
/*  Function    : Configures the common features for both DAC                */
/*                                                                           */
/*  Type        : Global                                                     */
/*                                                                           */
/*  Input Para  : None                                                       */
/*                                                                           */
/*  Output Para : None                                                       */
/*                                                                           */
/*                                                                           */
/*  Author      : I. Oesch                                                   */
/*                                                                           */
/*  History     : 10.09.2013  IO Created                                     */
/*                                                                           */
/*****************************************************************************/
void DAC_Common_Config(void)
{
    /* procedure data */
    GPIO_InitTypeDef GPIO_InitStructure;

    /* procedure code */

   /* DMA1 clock and GPIOA clock enable (to be used with DAC) */
   RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1 | RCC_AHB1Periph_GPIOA, ENABLE);

   /* DAC Peripherie clock enable */
   RCC_APB1PeriphClockCmd(RCC_APB1Periph_DAC, ENABLE);

   /* DAC channel 1 & 2 (DAC_OUT1 = PA.4, DAC_OUT2 = PA.5) configuration */
   /* Analog out, disable push and pull resistors                        */
   GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4 | GPIO_Pin_5;
   GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
   GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
   GPIO_Init(GPIOA, &GPIO_InitStructure);
}
/*****************************************************************************/
/*  End         : DAC_Common_Config                                          */
/*****************************************************************************/

/*****************************************************************************/
/*  Procedure   : DAC_Ch1_Config                                             */
/*****************************************************************************/
/*                                                                           */
/*  Function    : Configures DAC 1 and its DMA channel                       */
/*                                                                           */
/*                DMA is configured in doublebuffering mode, thus there are  */
/*                two buffers used. One is used in the current transfer, the */
/*                other can be used by the signal processing algorithm.      */
/*                                                                           */
/*                When the DMA has emptied its current buffer, an interrupt  */
/*                will be issued an the second buffer will be used for       */
/*                transfer. This swapping of buffers will continue for ever  */
/*                without any need of interaction by the user                */
/*                                                                           */
/*  Type        : Global                                                     */
/*                                                                           */
/*  Input Para  : None                                                       */
/*                                                                           */
/*  Output Para : None                                                       */
/*                                                                           */
/*                                                                           */
/*  Author      : I. Oesch                                                   */
/*                                                                           */
/*  History     : 10.09.2013  IO Created                                     */
/*                                                                           */
/*****************************************************************************/
void DAC_Ch1_Config(void)
{
    /* procedure data */
    DMA_InitTypeDef DMA_InitStructure;
    DAC_InitTypeDef       DAC_InitStructure;

    /* procedure code */


  /* DMA1_Stream5 channel7 configuration */

  /* Channel 7 is connected to ADC1 */
  DMA_InitStructure.DMA_Channel = DMA_Channel_7;

  /* Write Values to DAC 2 Data register */
  DMA_InitStructure.DMA_PeripheralBaseAddr = DAC_DHR12L1_ADDRESS;

  /* Read Values from TxBuffer2_a */
  DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)&TxBuffer1_a;
  DMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral;

  /* Define number of values to read int one block */
  DMA_InitStructure.DMA_BufferSize = BLOCK_SIZE;

  /* Define increment mode for Pheripherie and memory adress */
  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;

  /* Size of data to transfer is 16bit */
  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
  DMA_InitStructure.DMA_MemoryDataSize = DMA_PeripheralDataSize_HalfWord;

  /* We want to use double buffering */
  DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;

  /* Additional housekeeping stuff... */
  DMA_InitStructure.DMA_Priority = DMA_Priority_High;
  DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;         
  DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_HalfFull;
  DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
  DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;

  /* Initialize DMA with given values */
  DMA_Init(DMA1_Stream5, &DMA_InitStructure);

  /* Define second buffer for doublebuffering */
  DMA_DoubleBufferModeConfig(DMA1_Stream5,  (uint32_t)&TxBuffer1_b, DMA_Memory_0);

  /* Enable doublebuffering */
  DMA_DoubleBufferModeCmd 	(DMA1_Stream5, ENABLE);

  /* Enable DMA-Interrupts */
  DMA_ITConfig 	(DMA1_Stream5, DMA_IT_TC, ENABLE);
  DMA_ITConfig 	(DMA1_Stream5, DMA_IT_HT, ENABLE);
  DMA_ITConfig 	(DMA1_Stream5, DMA_IT_TE, ENABLE);
  DMA_ITConfig 	(DMA1_Stream5, DMA_IT_FE, ENABLE);

  /* Enable DMA1 Stream5 */
  DMA_Cmd(DMA1_Stream5, ENABLE);
  
  /* DAC channel1 Configuration */

  /* External trigger by timer 2 */
  DAC_InitStructure.DAC_Trigger = DAC_Trigger_T2_TRGO;

  /*Normal DAC Mode, enable extra (analog)output buffer */
  DAC_InitStructure.DAC_WaveGeneration = DAC_WaveGeneration_None;
  DAC_InitStructure.DAC_OutputBuffer = DAC_OutputBuffer_Enable;

  /* Initialize DAC with given values */
  DAC_Init(DAC_Channel_1, &DAC_InitStructure);

  /* Enable DAC Channel 1 */
  DAC_Cmd(DAC_Channel_1, ENABLE);

  /* Enable DMA for DAC Channel 1 */
  DAC_DMACmd(DAC_Channel_1, ENABLE);
}
/*****************************************************************************/
/*  End         : DAC_Ch1_Config                                             */
/*****************************************************************************/

/*****************************************************************************/
/*  Procedure   : ProcessBuffer                                              */
/*****************************************************************************/
/*                                                                           */
/*  Function    : Is called from DMA-Interrupt whenever a new set of buffers */
/*                is ready for processing                                    */
/*                Checks if doublebuffering for all channels is still        */
/*                synchron, stops system if synchronisation is lost          */
/*                                                                           */
/*                                                                           */
/*  Type        : Global                                                     */
/*                                                                           */
/*  Input Para  : None                                                       */
/*                                                                           */
/*  Output Para : None                                                       */
/*                                                                           */
/*                                                                           */
/*  Author      : I. Oesch                                                   */
/*                                                                           */
/*  History     : 10.09.2013  IO Created                                     */
/*                                                                           */
/*****************************************************************************/
void ProcessBuffer(void)
{
    /* procedure data */
    int Buffer1;
    int Buffer2;
    int Buffer3;
    int Buffer4;

    /* procedure code */

    /* Clear Idle-Led (for time measurements) */
	GPIO_ResetBits(GPIOD, GPIO_Pin_14);

	/* Get active buffer of all channels */
	Buffer1 = DMA_GetCurrentMemoryTarget(DMA2_Stream0) ;
	Buffer3 = DMA_GetCurrentMemoryTarget(DMA1_Stream5) ;
#if NUMBER_OF_CHANNELS == 2
	Buffer2 = DMA_GetCurrentMemoryTarget(DMA2_Stream2) ;
	Buffer4 = DMA_GetCurrentMemoryTarget(DMA1_Stream6) ;

	/* check if all channels use the same active buffer */
#if BLOCK_SIZE == 1
    if ((Buffer1 != Buffer2)||(Buffer1 == Buffer3)||(Buffer1 == Buffer4)) {
        /* We got Trouble, buffers are asynchron */
        /* All leds on... */
              FatalError();

    }
#else
	if ((Buffer1 != Buffer2)||(Buffer1 != Buffer3)||(Buffer1 != Buffer4)) {
        /* We got Trouble, buffers are asynchron */
        /* All leds on... */
		      FatalError();

	}
#endif
#endif

	/* Determine which set of buffers to use */
	if (Buffer1 == 1) {
        /* Signal active buffers */
	    GPIO_SetBits(GPIOD, GPIO_Pin_12);
		GPIO_ResetBits(GPIOD, GPIO_Pin_13);

		/* Call ProcessBlock() with the new buffers */
#if NUMBER_OF_CHANNELS == 2
	    ProcessBlock(Buffer1_a, Buffer2_a, TxBuffer1_a, TxBuffer2_a);
#else
	    ProcessBlock(Buffer1_a, TxBuffer1_a);
#endif
	} else {
        /* Signal active buffers */
		GPIO_SetBits(GPIOD, GPIO_Pin_13);
		GPIO_ResetBits(GPIOD, GPIO_Pin_12);

	    /* Call ProcessBlock() with the new buffers */
#if NUMBER_OF_CHANNELS == 2
	    ProcessBlock(Buffer1_b, Buffer2_b, TxBuffer1_b, TxBuffer2_b);
#else
	    ProcessBlock(Buffer1_b, TxBuffer1_b);
#endif

	}
	/* Set Idle-Led (for time measurements) */
   GPIO_SetBits(GPIOD, GPIO_Pin_14);

}
/*****************************************************************************/
/*  End         : ProcessBuffer                                              */
/*****************************************************************************/

/*****************************************************************************/
/*  Procedure   : Delay                                                      */
/*****************************************************************************/
/*                                                                           */
/*  Function    : Delay code execution for some time (Not Calibrated!)       */
/*                                                                           */
/*                                                                           */
/*  Type        : Global                                                     */
/*                                                                           */
/*  Input Para  : dlyTicks:  Time to wait (Relative, not calibrated)         */
/*                                                                           */
/*  Output Para : None                                                       */
/*                                                                           */
/*                                                                           */
/*  Author      : I. Oesch                                                   */
/*                                                                           */
/*  History     : 10.09.2013  IO Created                                     */
/*                                                                           */
/*****************************************************************************/
volatile int WaitWait;

static void Delay (uint32_t dlyTicks) {
    /* procedure data */

    /* procedure code */

    /* Just use up some processor time */
   dlyTicks >>= 5;
   while (dlyTicks-- > 0) {
      WaitWait = 0x80;
      while (WaitWait-- >0) {
      }
   }
}
/*****************************************************************************/
/*  End         : Delay                                                      */
/*****************************************************************************/

/*****************************************************************************/
/*  Procedure   : FatalError                                                 */
/*****************************************************************************/
/*                                                                           */
/*  Function    : Just flashes LEDs and never returns                        */
/*                                                                           */
/*                                                                           */
/*  Type        : Global                                                     */
/*                                                                           */
/*  Input Para  : None                                                       */
/*                                                                           */
/*  Output Para : None                                                       */
/*                                                                           */
/*                                                                           */
/*  Author      : I. Oesch                                                   */
/*                                                                           */
/*  History     : 10.09.2013  IO Created                                     */
/*                                                                           */
/*****************************************************************************/
void FatalError(void) {
    /* procedure data */

    /* procedure code */

    /* Just flashes LED forever */
 while(1) {
  GPIO_SetBits(GPIOD, GPIO_Pin_12);
  GPIO_SetBits(GPIOD, GPIO_Pin_13);
  GPIO_SetBits(GPIOD, GPIO_Pin_14);
  GPIO_SetBits(GPIOD, GPIO_Pin_15);
  /* Insert delay */
  Delay(0x3FFFFF);
  GPIO_ResetBits(GPIOD, GPIO_Pin_12);
  GPIO_SetBits(GPIOD, GPIO_Pin_13);
  GPIO_ResetBits(GPIOD, GPIO_Pin_14);
  GPIO_SetBits(GPIOD, GPIO_Pin_15);
  /* Insert delay */
  Delay(0x3FFFFF);
  GPIO_SetBits(GPIOD, GPIO_Pin_12);
  GPIO_ResetBits(GPIOD, GPIO_Pin_13);
  GPIO_SetBits(GPIOD, GPIO_Pin_14);
  GPIO_ResetBits(GPIOD, GPIO_Pin_15);
  /* Insert delay */
  Delay(0x3FFFFF);
  };
}
/*****************************************************************************/
/*  End         : FatalError                                                 */
/*****************************************************************************/

#if 0

/*
 * Callback used by stm32f4_discovery_audio_codec.c.
 * Refer to stm32f4_discovery_audio_codec.h for more info.
 */
void EVAL_AUDIO_TransferComplete_CallBack(uint32_t pBuffer, uint32_t Size){
  /* TODO, implement your code here */
  return;
}

/*
 * Callback used by stm324xg_eval_audio_codec.c.
 * Refer to stm324xg_eval_audio_codec.h for more info.
 */
uint16_t EVAL_AUDIO_GetSampleCallBack(void){
  /* TODO, implement your code here */
  return -1;
}
#endif

/*****************************************************************************/
/*  End Module  : DSP_Main                                                   */
/*****************************************************************************/


