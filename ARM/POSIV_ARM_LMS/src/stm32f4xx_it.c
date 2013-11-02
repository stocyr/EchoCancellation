/**
  ******************************************************************************
  * @file    Project/STM32F4xx_StdPeriph_Template/stm32f4xx_it.c 
  * @author  MCD Application Team
  * @version V1.1.0
  * @date    18-January-2013
  *          14.10.2013 OSI1 DMA-Interrupthandler added
  * @brief   Main Interrupt Service Routines.
  *          This file provides template for all exceptions handler and 
  *          peripherals interrupt service routine.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2013 STMicroelectronics</center></h2>
  *
  * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
  * You may not use this file except in compliance with the License.
  * You may obtain a copy of the License at:
  *
  *        http://www.st.com/software_license_agreement_liberty_v2
  *
  * Unless required by applicable law or agreed to in writing, software 
  * distributed under the License is distributed on an "AS IS" BASIS, 
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_it.h"
#include "stm32f4_discovery.h"
#include "config.h"

/** @addtogroup Template_Project
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/******************************************************************************/
/*            Cortex-M4 Processor Exceptions Handlers                         */
/******************************************************************************/

/**
  * @brief   This function handles NMI exception.
  * @param  None
  * @retval None
  */
void NMI_Handler(void)
{
}

/**
  * @brief  This function handles Hard Fault exception.
  * @param  None
  * @retval None
  */
void HardFault_Handler(void)
{
  /* Go to infinite loop when Hard Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Memory Manage exception.
  * @param  None
  * @retval None
  */
void MemManage_Handler(void)
{
  /* Go to infinite loop when Memory Manage exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Bus Fault exception.
  * @param  None
  * @retval None
  */
void BusFault_Handler(void)
{
  /* Go to infinite loop when Bus Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Usage Fault exception.
  * @param  None
  * @retval None
  */
void UsageFault_Handler(void)
{
  /* Go to infinite loop when Usage Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles SVCall exception.
  * @param  None
  * @retval None
  */
void SVC_Handler(void)
{
}

/**
  * @brief  This function handles Debug Monitor exception.
  * @param  None
  * @retval None
  */
void DebugMon_Handler(void)
{
}

/**
  * @brief  This function handles PendSVC exception.
  * @param  None
  * @retval None
  */
void PendSV_Handler(void)
{
}

/**
  * @brief  This function handles SysTick Handler.
  * @param  None
  * @retval None
  */
void SysTick_Handler(void)
{
/*  TimingDelay_Decrement(); */
}

/******************************************************************************/
/*                 STM32F4xx Peripherals Interrupt Handlers                   */
/*  Add here the Interrupt Handler for the used peripheral(s) (PPP), for the  */
/*  available peripheral interrupt handler's name please refer to the startup */
/*  file (startup_stm32f40xx.s/startup_stm32f427x.s).                         */
/******************************************************************************/

/**
  * @brief  This function handles External line 0 interrupt request.
  * @param  None
  * @retval None
  */
#if 0
void EXTI0_IRQHandler(void)
{
    extern __IO uint8_t SelectedWavesForm, KeyPressed;
  if(EXTI_GetITStatus(USER_BUTTON_EXTI_LINE) != RESET)
  {
#if 0
    /* Change the wave */
    KeyPressed = 0;

    /* Change the selected waves forms */
    SelectedWavesForm = !SelectedWavesForm;
#endif
    /* Clear the Right Button EXTI line pending bit */
    EXTI_ClearITPendingBit(USER_BUTTON_EXTI_LINE);
  }
}
#endif

/*****************************************************************************/
/*  Procedure   : DMAx_Streamy_IRQHandler                                    */
/*****************************************************************************/
/*                                                                           */
/*  Function    : IRQ-Handler for the 4 DMA Interrupts of ADC1, ADC2, DAC1   */
/*                and DAC2                                                   */
/*                Whenever all 4 channels are ready, ProcessBuffer() will be */
/*                called                                                     */
/*                                                                           */
/*  Type        : Interrupthandler                                           */
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

/* Variables to keep track of which buffers are ready */

volatile int TransferCompleteADC1 = 0;
volatile int TransferCompleteADC2 = 0;
volatile int TransferCompleteDAC1 = 0;
volatile int TransferCompleteDAC2 = 0;

extern void ProcessBuffer(void);



void DMA1_Stream5_IRQHandler(void)
{
    /* procedure data */

    /* procedure code */

	//GPIO_ResetBits(GPIOD, GPIO_Pin_6);

    /* Check for Transfer complete interrupt */
	if (DMA_GetITStatus(DMA1_Stream5, DMA_IT_TCIF5) == SET ){

	   /* Signal interrupt (for time measurement) */
       GPIO_ToggleBits(GPIOD, GPIO_Pin_6);

       /* Clear interrupt pending flag */
       DMA_ClearITPendingBit(DMA1_Stream5, DMA_IT_TCIF5);

       /* Mark buffer 1 as ready */
       TransferCompleteDAC1 = 1;

       /* Call processbuffer() and clear readyflags if all blocks are ready */
#if NUMBER_OF_CHANNELS == 2
       if (TransferCompleteADC1 && TransferCompleteADC2 && TransferCompleteDAC1 && TransferCompleteDAC2) {
    	  TransferCompleteADC1 = 0;
    	  TransferCompleteADC2 = 0;
    	  TransferCompleteDAC1 = 0;
    	  TransferCompleteDAC2 = 0;
          ProcessBuffer();
       }
#else
       if (TransferCompleteADC1 && TransferCompleteDAC1) {
    	  TransferCompleteADC1 = 0;
    	  TransferCompleteDAC1 = 0;
          ProcessBuffer();
       }
#endif
	}
	/* for all other interrupts, just clear the pending flags */
	if (DMA_GetITStatus(DMA1_Stream5, DMA_IT_HTIF5) == SET ){
       DMA_ClearITPendingBit(DMA1_Stream5, DMA_IT_HTIF5);
	}
	if (DMA_GetITStatus(DMA1_Stream5, DMA_IT_TEIF5) == SET ){
       DMA_ClearITPendingBit(DMA1_Stream5, DMA_IT_TEIF5);
	}
	if (DMA_GetITStatus(DMA1_Stream5, DMA_IT_DMEIF5) == SET ){
       DMA_ClearITPendingBit(DMA1_Stream5, DMA_IT_DMEIF5);
	}
	if (DMA_GetITStatus(DMA1_Stream5, DMA_IT_FEIF5) == SET ){
       DMA_ClearITPendingBit(DMA1_Stream5, DMA_IT_FEIF5);
	}
}





void DMA1_Stream6_IRQHandler(void)
{
    /* procedure data */

    /* procedure code */

	//GPIO_ResetBits(GPIOD, GPIO_Pin_7);
    /* Check for Transfer complete interrupt */
	if (DMA_GetITStatus(DMA1_Stream6, DMA_IT_TCIF6) == SET ){

	   /* Signal interrupt (for time measurement) */
   	   GPIO_ToggleBits(GPIOD, GPIO_Pin_7);

       /* Clear interrupt pending flag */
       DMA_ClearITPendingBit(DMA1_Stream6, DMA_IT_TCIF6);

       /* Mark buffer 2 as ready */
       TransferCompleteDAC2 = 1;

       /* Call processbuffer() and clear readyflags if all blocks are ready */
#if NUMBER_OF_CHANNELS == 2
       if (TransferCompleteADC1 && TransferCompleteADC2 && TransferCompleteDAC1 && TransferCompleteDAC2) {
    	  TransferCompleteADC1 = 0;
    	  TransferCompleteADC2 = 0;
    	  TransferCompleteDAC1 = 0;
    	  TransferCompleteDAC2 = 0;
          ProcessBuffer();
       }
#else
       if (TransferCompleteADC1 && TransferCompleteDAC1) {
    	  TransferCompleteADC1 = 0;
    	  TransferCompleteDAC1 = 0;
          ProcessBuffer();
       }
#endif
	}
    /* for all other interrupts, just clear the pending flags */
	if (DMA_GetITStatus(DMA1_Stream6, DMA_IT_HTIF6) == SET ){
       DMA_ClearITPendingBit(DMA1_Stream6, DMA_IT_HTIF6);
	}
	if (DMA_GetITStatus(DMA1_Stream6, DMA_IT_TEIF6) == SET ){
       DMA_ClearITPendingBit(DMA1_Stream6, DMA_IT_TEIF6);
	}
	if (DMA_GetITStatus(DMA1_Stream6, DMA_IT_DMEIF6) == SET ){
       DMA_ClearITPendingBit(DMA1_Stream6, DMA_IT_DMEIF6);
	}
	if (DMA_GetITStatus(DMA1_Stream6, DMA_IT_FEIF6) == SET ){
       DMA_ClearITPendingBit(DMA1_Stream6, DMA_IT_FEIF6);
	}
}

void DMA2_Stream0_IRQHandler(void)
{
    /* procedure data */

    /* procedure code */

	//GPIO_ResetBits(GPIOD, GPIO_Pin_8);
    /* Check for Transfer complete interrupt */
	if (DMA_GetITStatus(DMA2_Stream0, DMA_IT_TCIF0) == SET ){

	   /* Signal interrupt (for time measurement) */
       GPIO_ToggleBits(GPIOD, GPIO_Pin_8);

       /* Clear interrupt pending flag */
       DMA_ClearITPendingBit(DMA2_Stream0, DMA_IT_TCIF0);

       /* Mark buffer 3 as ready */
       TransferCompleteADC1 = 1;

       /* Call processbuffer() and clear readyflags if all blocks are ready */
#if NUMBER_OF_CHANNELS == 2
       if (TransferCompleteADC1 && TransferCompleteADC2 && TransferCompleteDAC1 && TransferCompleteDAC2) {
    	  TransferCompleteADC1 = 0;
    	  TransferCompleteADC2 = 0;
    	  TransferCompleteDAC1 = 0;
    	  TransferCompleteDAC2 = 0;
          ProcessBuffer();
       }
#else
       if (TransferCompleteADC1 && TransferCompleteDAC1) {
    	  TransferCompleteADC1 = 0;
    	  TransferCompleteDAC1 = 0;
          ProcessBuffer();
       }
#endif

	}
    /* for all other interrupts, just clear the pending flags */
	if (DMA_GetITStatus(DMA2_Stream0, DMA_IT_HTIF0) == SET ){
       DMA_ClearITPendingBit(DMA2_Stream0, DMA_IT_HTIF0);
	}
	if (DMA_GetITStatus(DMA2_Stream0, DMA_IT_TEIF0) == SET ){
       DMA_ClearITPendingBit(DMA2_Stream0, DMA_IT_TEIF0);
	}
	if (DMA_GetITStatus(DMA2_Stream0, DMA_IT_DMEIF0) == SET ){
       DMA_ClearITPendingBit(DMA2_Stream0, DMA_IT_DMEIF0);
	}
	if (DMA_GetITStatus(DMA2_Stream0, DMA_IT_FEIF0) == SET ){
       DMA_ClearITPendingBit(DMA2_Stream0, DMA_IT_FEIF0);
	}


}


void DMA2_Stream2_IRQHandler(void)
{
    /* procedure data */

    /* procedure code */

	//GPIO_ResetBits(GPIOD, GPIO_Pin_9);
    /* Check for Transfer complete interrupt */
	if (DMA_GetITStatus(DMA2_Stream2, DMA_IT_TCIF2) == SET ){

	   /* Signal interrupt (for time measurement) */
       GPIO_ToggleBits(GPIOD, GPIO_Pin_9);

       /* Clear interrupt pending flag */
       DMA_ClearITPendingBit(DMA2_Stream2, DMA_IT_TCIF2);

       /* Mark buffer 4 as ready */
       TransferCompleteADC2 = 1;

       /* Call processbuffer() and clear readyflags if all blocks are ready */
#if NUMBER_OF_CHANNELS == 2
       if (TransferCompleteADC1 && TransferCompleteADC2 && TransferCompleteDAC1 && TransferCompleteDAC2) {
    	  TransferCompleteADC1 = 0;
    	  TransferCompleteADC2 = 0;
    	  TransferCompleteDAC1 = 0;
    	  TransferCompleteDAC2 = 0;
          ProcessBuffer();
       }
#else
       if (TransferCompleteADC1 && TransferCompleteDAC1) {
    	  TransferCompleteADC1 = 0;
    	  TransferCompleteDAC1 = 0;
          ProcessBuffer();
       }
#endif
	}
    /* for all other interrupts, just clear the pending flags */
	if (DMA_GetITStatus(DMA2_Stream2, DMA_IT_HTIF2) == SET ){
       DMA_ClearITPendingBit(DMA2_Stream2, DMA_IT_HTIF2);
	}
	if (DMA_GetITStatus(DMA2_Stream2, DMA_IT_TEIF2) == SET ){
       DMA_ClearITPendingBit(DMA2_Stream2, DMA_IT_TEIF2);
	}
	if (DMA_GetITStatus(DMA2_Stream2, DMA_IT_DMEIF2) == SET ){
       DMA_ClearITPendingBit(DMA2_Stream2, DMA_IT_DMEIF2);
	}
	if (DMA_GetITStatus(DMA2_Stream2, DMA_IT_FEIF2) == SET ){
       DMA_ClearITPendingBit(DMA2_Stream2, DMA_IT_FEIF2);
	}

}
/*****************************************************************************/
/*  End         : DMAx_Streamy_IRQHandler                                    */
/*****************************************************************************/

#ifdef SHOW_FS

void TIM2_IRQHandler(void)
{
    GPIO_ToggleBits(GPIOD, GPIO_Pin_10);
	TIM_ClearITPendingBit 	(TIM2, TIM_IT_Update);
}
#endif



/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
