/* general control */

/*****************************************************************************/
/*  Module     : FFT-IFFT-Demo                                  Version 1.0  */
/*****************************************************************************/
/*                                                                           */
/*  Function   : A Demo for float32, Q31 and Q16 FFT, transforms channel 1   */
/*               by FFT and direct by IFFT, without any processing in        */
/*               between. The double transformed signal is presented as      */
/*               output on channel 1.                                        */
/*               This is just an example to demonstrate the usage of the FFT */
/*               IFFT routines from the arm-CMSIS-DSP-Lib                    */
/*                                                                           */
/*               Attention, twiddle factor table must be generated at        */
/*               since the free version of the compiler is limitet to 32k    */
/*               codesize.                                                   */
/*                                                                           */
/*  Procedures : InitProcessing()                                            */
/*               ProcessBlock()                                              */
/*               IdleFunction()                                              */
/*                                                                           */
/*  Author     : I. Oesch                                                    */
/*                                                                           */
/*  History    : 10.09.2013  IO Created                                      */
/*                                                                           */
/*  File       : SignalProcessingFFT.c                                       */
/*                                                                           */
/*****************************************************************************/
/*  Berner Fachhochschule   *      Fachbereich EKT                           */
/*  TI Burgdorf             *      Digitale Signalverarbeitung               */
/*****************************************************************************/

/* imports */
#include "SignalProcessing.h"
#include "DSPMain.h"
#include "stm32f4_discovery.h"
#include <math.h>

/* module constant declaration */

/* Select the number format */
//#define MAKEFFT_FLOAT
#define MAKEFFT_Q31
//#define MAKEFFT_Q15

//Supported FFT Lengths are 16, 64, 256, 1024, 2048, 4096

#define FFT_SIZE BLOCK_SIZE

/* Determine log2(FFT_SIZE) and check for consistency */
#if FFT_SIZE == 16
#define LOG_2_FFTSIZE 4
#elif FFT_SIZE == 32
#define LOG_2_FFTSIZE 5
#elif FFT_SIZE == 64
#define LOG_2_FFTSIZE 6
#elif FFT_SIZE == 128
#define LOG_2_FFTSIZE 7
#elif FFT_SIZE == 256
#define LOG_2_FFTSIZE 8
#elif FFT_SIZE == 512
#define LOG_2_FFTSIZE 9
#elif FFT_SIZE == 1024
#define LOG_2_FFTSIZE 10
#elif FFT_SIZE == 2048
#define LOG_2_FFTSIZE 11
#elif FFT_SIZE == 4096
#define LOG_2_FFTSIZE 12
#else
#error ILLEGAL_FFTSIZE
#endif

/* module type declaration */

/* module data declaration */

#ifdef MAKEFFT_FLOAT
/* Buffers for float variant */
float32_t FFT_Workbuffer[FFT_SIZE*2];
float32_t twiddleCoef[6144];

#elif defined(MAKEFFT_Q31)
/* Buffers for q32 variant */
q31_t FFT_WorkbufferQ31[FFT_SIZE*2];
q31_t twiddleCoefQ31[6144];

#elif defined(MAKEFFT_Q15)
/* Buffers for q15 variant */
q15_t FFT_WorkbufferQ15[FFT_SIZE*2];
q15_t twiddleCoefQ15[6144];
#endif

/* Status from algorithm */
arm_status status;

/* storage for configuration of FFT Algorithm */
arm_cfft_radix2_instance_f32 FFT_State;
arm_cfft_radix2_instance_f32 IFFT_State;
arm_cfft_radix2_instance_q31 FFT_StateQ31;
arm_cfft_radix2_instance_q31 IFFT_StateQ31;
arm_cfft_radix2_instance_q15 FFT_StateQ15;
arm_cfft_radix2_instance_q15 IFFT_StateQ15;

#ifdef MAKEFFT_FLOAT
/*****************************************************************************/
/*  Procedure   : InitProcessing                                             */
/*****************************************************************************/
/*                                                                           */
/*  Function    : Is called from main, before starting the System to         */
/*                initialize all required buffers and tables, especially     */
/*                the twiddle factors must be generated here.                */
/*                                                                           */
/*  Type        : Global                                                     */
/*                                                                           */
/*  Input Para  : None                                                       */
/*                                                                           */
/*  Output Para : None                                                       */
/*                                                                           */
/*  Author      : I. Oesch                                                   */
/*                                                                           */
/*  History     : 10.09.2013  IO  Created                                    */
/*                                                                           */
/*****************************************************************************/
void InitProcessing(void)
{

    /* procedure data */
    int i;

    /* procedure code */
   status = ARM_MATH_SUCCESS;

   /* Initialize the CFFT/CIFFT module */
   status = arm_cfft_radix2_init_f32(&FFT_State, FFT_SIZE, 0, 1);
   if (status != ARM_MATH_SUCCESS) {
      FatalError();
   }
   status = arm_cfft_radix2_init_f32(&IFFT_State, FFT_SIZE, 1, 1);
   if (status != ARM_MATH_SUCCESS) {
      FatalError();
   }


   /* generate twiddle factors for fft (code taken from CMSIS/DSP_Lib/arm_common_tables.c) */
#define N 4096
#define PI 3.14159265358979

   for(i = 0; i < 3*N/4; i++)
    {
   	   twiddleCoef[2*i]= cos(i * 2*PI/(float)N);
   	   twiddleCoef[2*i+1]= sin(i * 2*PI/(float)N);
    }
}
/*****************************************************************************/
/*  End         : InitProcessing                                             */
/*****************************************************************************/

/*****************************************************************************/
/*  Procedure   : ProcessBlock                                               */
/*****************************************************************************/
/*                                                                           */
/*  Function    : Is called from the DMA-interrupt whenever a block of new   */
/*                samples has been collected. This Function just collects    */
/*                blocks up to FFT_SIZE samples, and calculates the FFT /    */
/*                IFFT over the last FFT_SIZE samples.                       */
/*                The twice transformed signal will be presented on          */
/*                channel 1 output                                           */
/*                                                                           */
/*                Input samples are in unsigned 16-Bit format, Values        */
/*                ranging from 0 to 65535, to get signed values, 32768       */
/*                must be subtracted (and added again before placed in       */
/*                output buffer)                                             */
/*                                                                           */
/*                                                                           */
/*  Type        : Global                                                     */
/*                                                                           */
/*  Input Para  : Channel1_in  Pointer to block of input data from channel 1 */
/*                Channel2_in  Pointer to block of input data from channel 2 */
/*                Channel1_out Pointer to block of output data to channel 1  */
/*                Channel2_out Pointer to block of output data to channel 2  */
/*                                                                           */
/*  Output Para : None                                                       */
/*                                                                           */
/*  Author      : I. Oesch                                                   */
/*                                                                           */
/*  History     : 10.09.2013  IO  Created                                    */
/*                                                                           */
/*****************************************************************************/
#if NUMBER_OF_CHANNELS == 2
void ProcessBlock(uint16_t *Channel1_in, uint16_t *Channel2_in, uint16_t *Channel1_out, uint16_t *Channel2_out)
{

    /* procedure data */
    int i, j;

    /* procedure code */

	/* Copy into workbuffer und make complex */
    for (i = 0, j=0; i < FFT_SIZE; i++) {
    	FFT_Workbuffer[j++] = ((float32_t)(Channel1_in[i] - 32768));
    	FFT_Workbuffer[j++] = 0;
    }
    /* (Led Toggling just for timing measurements) */
    GPIO_SetBits(GPIOD, GPIO_Pin_0);

    /* Process the data through the CFFT/CIFFT module */
    arm_cfft_radix2_f32(&FFT_State, FFT_Workbuffer);

    /* (Led Toggling just for timing measurements) */
    GPIO_ResetBits(GPIOD, GPIO_Pin_0);

    /* and directly inverse transform */
    arm_cfft_radix2_f32(&IFFT_State, FFT_Workbuffer);

    /* (Led Toggling just for timing measurements) */
    GPIO_SetBits(GPIOD, GPIO_Pin_0);

    /* Just copies the result of the transforming process to the output  */
    /* (eg the content of OutputBufferFilter1) and undoes normalizing */
    for (i = 0; i < BLOCK_SIZE; i++) {

		   Channel1_out[i] = FFT_Workbuffer[2*i]+32768;

		   /* Unprocessed samples on output 2 */
	       Channel2_out[i] = Channel2_in[i];
   }

    /* (Led Toggling just for timing measurements) */
    GPIO_ResetBits(GPIOD, GPIO_Pin_0);
}
#else
void ProcessBlock(uint16_t *Channel1_in, uint16_t *Channel1_out)
{
    int i;
	for (i = 0; i < BLOCK_SIZE; i++) {
		Channel1_out[i] = Channel1_in[i];
	}
}
#endif
/*****************************************************************************/
/*  End         : ProcessBlock                                               */
/*****************************************************************************/


#elif defined(MAKEFFT_Q31)
/*****************************************************************************/
/*  Procedure   : InitProcessing                                             */
/*****************************************************************************/
/*                                                                           */
/*  Function    : Is called from main, before starting the System to         */
/*                initialize all required buffers and tables, especially     */
/*                the twiddle factors must be generated here.                */
/*                                                                           */
/*  Type        : Global                                                     */
/*                                                                           */
/*  Input Para  : None                                                       */
/*                                                                           */
/*  Output Para : None                                                       */
/*                                                                           */
/*  Author      : I. Oesch                                                   */
/*                                                                           */
/*  History     : 10.09.2013  IO  Created                                    */
/*                                                                           */
/*****************************************************************************/
void InitProcessing(void)
{

    /* procedure data */
    int i;

    /* procedure code */
   status = ARM_MATH_SUCCESS;

   /* Initialize the CFFT/CIFFT module */
   status = arm_cfft_radix2_init_q31(&FFT_StateQ31, FFT_SIZE, 0, 1);

   if (status != ARM_MATH_SUCCESS) {
      FatalError();
   }
   status = arm_cfft_radix2_init_q31(&IFFT_StateQ31, FFT_SIZE, 1, 1);

   if (status != ARM_MATH_SUCCESS) {
      FatalError();
   }

   /* generate twiddle factors for fft (code taken from CMSIS/DSP_Lib/arm_common_tables.c) */
#define N 4096
#define PI 3.14159265358979

   for(i = 0; i < 3*N/4; i++)
    {
	   double val = cos(i * 2*PI/(float)N)*0x80000000u+0.5;
	   if (val > 0x7fffffffLu) {
	    val = 0x7fffffffLu;
	   }
	   if (val < -(256.0*256*256*127)) {
	    val = -(256.0*256*256*127);
	   }
	   twiddleCoefQ31[2*i]= val;

	   val =  sin(i * 2*PI/(float)N)*0x80000000u+0.5;
	   if (val > 0x7fffffffLu) {
	    val = 0x7fffffffLu;
	   }
	   if (val < -(256.0*256*256*127)) {
	    val = -(256.0*256*256*127);
	   }
	   twiddleCoefQ31[2*i+1]= val;
    }
}
/*****************************************************************************/
/*  End         : InitProcessing                                             */
/*****************************************************************************/

/*****************************************************************************/
/*  Procedure   : ProcessBlock                                               */
/*****************************************************************************/
/*                                                                           */
/*  Function    : Is called from the DMA-interrupt whenever a block of new   */
/*                samples has been collected. This Function just collects    */
/*                blocks up to FFT_SIZE samples, and calculates the FFT /    */
/*                IFFT over the last FFT_SIZE samples.                       */
/*                The twice transformed signal will be presented on          */
/*                channel 1 output                                           */
/*                                                                           */
/*                Input samples are in unsigned 16-Bit format, Values        */
/*                ranging from 0 to 65535, to get signed values, 32768       */
/*                must be subtracted (and added again before placed in       */
/*                output buffer)                                             */
/*                                                                           */
/*                                                                           */
/*  Type        : Global                                                     */
/*                                                                           */
/*  Input Para  : Channel1_in  Pointer to block of input data from channel 1 */
/*                Channel2_in  Pointer to block of input data from channel 2 */
/*                Channel1_out Pointer to block of output data to channel 1  */
/*                Channel2_out Pointer to block of output data to channel 2  */
/*                                                                           */
/*  Output Para : None                                                       */
/*                                                                           */
/*  Author      : I. Oesch                                                   */
/*                                                                           */
/*  History     : 10.09.2013  IO  Created                                    */
/*                                                                           */
/*****************************************************************************/
#if NUMBER_OF_CHANNELS == 2
void ProcessBlock(uint16_t *Channel1_in, uint16_t *Channel2_in, uint16_t *Channel1_out, uint16_t *Channel2_out)
{

    /* procedure data */
    int i, j;

    /* procedure code */

	/* Copy into workbuffer und make complex */
    for (i = 0, j=0; i < FFT_SIZE; i++) {
    	FFT_WorkbufferQ31[j++] = ((q31_t)(Channel1_in[i] - 32768))<<16;
    	FFT_WorkbufferQ31[j++] = 0;
    }

    /* (Led Toggling just for timing measurements) */
    GPIO_SetBits(GPIOD, GPIO_Pin_0);

    /* Process the data through the CFFT/CIFFT module */
    arm_cfft_radix2_q31(&FFT_StateQ31, FFT_WorkbufferQ31);

    /* (Led Toggling just for timing measurements) */
    GPIO_ResetBits(GPIOD, GPIO_Pin_0);

    /* and directly inverse transform */
    arm_cfft_radix2_q31(&IFFT_StateQ31, FFT_WorkbufferQ31);

    /* (Led Toggling just for timing measurements) */
    GPIO_SetBits(GPIOD, GPIO_Pin_0);

    /* Just copies the result of the transforming process to the output  */
    /* (eg the content of OutputBufferFilter1) and undoes normalizing */
    for (i = 0; i < BLOCK_SIZE; i++) {

		   Channel1_out[i] = ((FFT_WorkbufferQ31[2*i]>>(16-LOG_2_FFTSIZE-2))+32678);
           /* Unprocessed samples on output 2 */
           Channel2_out[i] = Channel2_in[i];
    }

    /* (Led Toggling just for timing measurements) */
    GPIO_ResetBits(GPIOD, GPIO_Pin_0);
}
#else
void ProcessBlock(uint16_t *Channel1_in, uint16_t *Channel1_out)
{
    int i;
	for (i = 0; i < BLOCK_SIZE; i++) {
		Channel1_out[i] = Channel1_in[i];
	}

}
#endif
/*****************************************************************************/
/*  End         : ProcessBlock                                               */
/*****************************************************************************/

#elif defined(MAKEFFT_Q15)
/*****************************************************************************/
/*  Procedure   : InitProcessing                                             */
/*****************************************************************************/
/*                                                                           */
/*  Function    : Is called from main, before starting the System to         */
/*                initialize all required buffers and tables, especially     */
/*                the twiddle factors must be generated here.                */
/*                                                                           */
/*  Type        : Global                                                     */
/*                                                                           */
/*  Input Para  : None                                                       */
/*                                                                           */
/*  Output Para : None                                                       */
/*                                                                           */
/*  Author      : I. Oesch                                                   */
/*                                                                           */
/*  History     : 10.09.2013  IO  Created                                    */
/*                                                                           */
/*****************************************************************************/
void InitProcessing(void)
{

    /* procedure data */
    int i;

    /* procedure code */

   status = ARM_MATH_SUCCESS;

   /* Initialize the CFFT/CIFFT module */
   status = arm_cfft_radix2_init_q15(&FFT_StateQ15, FFT_SIZE, 0, 1);
   if (status != ARM_MATH_SUCCESS) {
      FatalError();
   }
   status = arm_cfft_radix2_init_q15(&IFFT_StateQ15, FFT_SIZE, 1, 1);
   if (status != ARM_MATH_SUCCESS) {
      FatalError();
   }


   /* generate twiddle factors for fft (code taken from CMSIS/DSP_Lib/arm_common_tables.c) */
#define N 4096
#define PI 3.14159265358979

   for(i = 0; i < 3*N/4; i++)
    {
	   double val = cos(i * 2*PI/(float)N)*0x8000u+0.5;
	   if (val > 0x7fffL) {
	    val = 0x7fffL;
	   }
	   if (val < -0x8000L) {
	    val = -0x8000L;
	   }
	   twiddleCoefQ15[2*i]=val ;

	   val = sin(i * 2*PI/(float)N)*0x8000u+0.5;
	   if (val > 0x7fffL) {
	    val = 0x7fffL;
	   }
	   if (val < -0x8000L) {
	    val = -0x8000L;
	   }
	   twiddleCoefQ15[2*i+1]= val;
    }
}
/*****************************************************************************/
/*  End         : InitProcessing                                             */
/*****************************************************************************/

/*****************************************************************************/
/*  Procedure   : ProcessBlock                                               */
/*****************************************************************************/
/*                                                                           */
/*  Function    : Is called from the DMA-interrupt whenever a block of new   */
/*                samples has been collected. This Function just collects    */
/*                blocks up to FFT_SIZE samples, and calculates the FFT /    */
/*                IFFT over the last FFT_SIZE samples.                       */
/*                The twice transformed signal will be presented on          */
/*                channel 1 output                                           */
/*                                                                           */
/*                Input samples are in unsigned 16-Bit format, Values        */
/*                ranging from 0 to 65535, to get signed values, 32768       */
/*                must be subtracted (and added again before placed in       */
/*                output buffer)                                             */
/*                                                                           */
/*                                                                           */
/*  Type        : Global                                                     */
/*                                                                           */
/*  Input Para  : Channel1_in  Pointer to block of input data from channel 1 */
/*                Channel2_in  Pointer to block of input data from channel 2 */
/*                Channel1_out Pointer to block of output data to channel 1  */
/*                Channel2_out Pointer to block of output data to channel 2  */
/*                                                                           */
/*  Output Para : None                                                       */
/*                                                                           */
/*  Author      : I. Oesch                                                   */
/*                                                                           */
/*  History     : 10.09.2013  IO  Created                                    */
/*                                                                           */
/*****************************************************************************/
#if NUMBER_OF_CHANNELS == 2
void ProcessBlock(uint16_t *Channel1_in, uint16_t *Channel2_in, uint16_t *Channel1_out, uint16_t *Channel2_out)
{

    /* procedure data */
    int i, j;

    /* procedure code */

	/* Copy into workbuffer und make complex */
    for (i = 0, j=0; i < FFT_SIZE; i++) {
    	FFT_WorkbufferQ15[j++] = ((q15_t)(Channel1_in[i] - 32768));
    	FFT_WorkbufferQ15[j++] = 0;
    }
    /* (Led Toggling just for timing measurements) */
    GPIO_SetBits(GPIOD, GPIO_Pin_0);

    /* Process the data through the CFFT/CIFFT module */
    arm_cfft_radix2_q15(&FFT_StateQ15, FFT_WorkbufferQ15);

    /* (Led Toggling just for timing measurements) */
    GPIO_ResetBits(GPIOD, GPIO_Pin_0);

    /* and directly inverse transform */
    arm_cfft_radix2_q15(&IFFT_StateQ15, FFT_WorkbufferQ15);

    /* (Led Toggling just for timing measurements) */
    GPIO_SetBits(GPIOD, GPIO_Pin_0);

    /* Just copies the result of the transforming process to the output  */
    /* (eg the content of OutputBufferFilter1) and undoes normalizing */
    for (i = 0; i < BLOCK_SIZE; i++) {

		   Channel1_out[i] = (FFT_WorkbufferQ15[2*i] << (LOG_2_FFTSIZE+2))+ 32768;
           /* Unprocessed samples on output 2 */
           Channel2_out[i] = Channel2_in[i];
    }

    /* (Led Toggling just for timing measurements) */
    GPIO_ResetBits(GPIOD, GPIO_Pin_0);
}
#else
void ProcessBlock(uint16_t *Channel1_in, uint16_t *Channel1_out)
{
    int i;
	for (i = 0; i < BLOCK_SIZE; i++) {
		Channel1_out[i] = Channel1_in[i];
	}

}
#endif
/*****************************************************************************/
/*  End         : ProcessBlock                                               */
/*****************************************************************************/


#else

/* Just an example for Q15 math */
void InitProcessing(void) {

}

/* Just an example for Q15 math */
#if NUMBER_OF_CHANNELS == 2
void ProcessBlock(uint16_t *Channel1_in, uint16_t *Channel2_in, uint16_t *Channel1_out, uint16_t *Channel2_out)
{
    int i;
    q31_t mul1;
    q15_t x;
	for (i = 0; i < BLOCK_SIZE; i++) {
		//Channel1_out[i] = Channel1_in[i];
		//Channel2_out[i] = Channel2_in[i];
		x =  Channel1_in[i] - 32768;
		 mul1 = (q31_t) ((q15_t) x * (q15_t) x);
		Channel1_out[i] = (q15_t) __SSAT(mul1 >> 15, 16);
		x =  Channel2_in[i] - 32768;
		 mul1 = (q31_t) ((q15_t) x * (q15_t) x);
		Channel2_out[i] = (q15_t) __SSAT(mul1 >> 15, 16) + 32768;
	}
}
#else
void ProcessBlock(uint16_t *Channel1_in, uint16_t *Channel1_out)
{
    int i;
	for (i = 0; i < BLOCK_SIZE; i++) {
		Channel1_out[i] = Channel1_in[i];
	}

}
#endif
#endif

/*****************************************************************************/
/*  Procedure   : IdleFunction                                               */
/*****************************************************************************/
/*                                                                           */
/*  Function    : Is called from main in an endless loop, here some not      */
/*                directly signalprocessing stuff may be done.               */
/*                (Like reactions on change on port pin or signaling         */
/*                 events on output pins)                                    */
/*                                                                           */
/*  Type        : Global                                                     */
/*                                                                           */
/*  Input Para  : None                                                       */
/*                                                                           */
/*  Output Para : None                                                       */
/*                                                                           */
/*  Author      : I. Oesch                                                   */
/*                                                                           */
/*  History     : 10.09.2013  IO  Created                                    */
/*                                                                           */
/*****************************************************************************/
void IdleFunction(void)
{
    /* procedure data */

    /* procedure code */

}
/*****************************************************************************/
/*  End         : IdleFunction                                               */
/*****************************************************************************/

/*****************************************************************************/
/*  End Module  : FIR-Filter-Demo                                            */
/*****************************************************************************/

