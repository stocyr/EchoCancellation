/* general control */

/*****************************************************************************/
/*  Module     : FFT-Filterbank-Demo                            Version 1.0  */
/*****************************************************************************/
/*                                                                           */
/*  Function   : A Demo for float32, Q31 and Q16 FFT Filterbank, filters     */
/*               channel 1 by FFT - Mulpiplying by transferfunction in       */
/*               frequency domain - IFFT                                     */
/*               This is just an example to demonstrate the usage of the     */
/*               FFT/IFFT routines from the arm-CMSIS-DSP-Lib                */
/*                                                                           */
/*                                                                           */
/*  Procedures : InitProcessing()                                            */
/*               ProcessBlock()                                              */
/*               IdleFunction()                                              */
/*                                                                           */
/*  Author     : I. Oesch                                                    */
/*                                                                           */
/*  History    : 10.09.2013  IO Created                                      */
/*                                                                           */
/*  File       : SignalProcessingFFTFilterbank.c                             */
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

/* Supported FFT Lengths are 16, 64, 256, 1024, 2048, 4096 */

#define FFT_SIZE (4*BLOCK_SIZE)

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
float32_t FFT_TimeHistory[FFT_SIZE];
float32_t OutputBufferFilter1[FFT_SIZE];
float32_t WindowWeights[FFT_SIZE];
float32_t FilterWeights[FFT_SIZE];
float32_t twiddleCoef[6144];

#elif defined(MAKEFFT_Q31)
/* Buffers for q32 variant */
q31_t FFT_WorkbufferQ31[FFT_SIZE*2];
q31_t FFT_TimeHistoryQ31[FFT_SIZE];
q31_t OutputBufferFilter1Q31[FFT_SIZE];
q31_t WindowWeightsQ31[FFT_SIZE];
q31_t FilterWeightsQ31[FFT_SIZE];
q31_t twiddleCoefQ31[6144];

#elif defined(MAKEFFT_Q15)
/* Buffers for q15 variant */
q15_t FFT_WorkbufferQ15[FFT_SIZE*2];
q15_t FFT_TimeHistoryQ15[FFT_SIZE];
q15_t OutputBufferFilter1Q15[FFT_SIZE];
q15_t WindowWeightsQ15[FFT_SIZE];
q15_t FilterWeightsQ15[FFT_SIZE];
q15_t twiddleCoefQ15[6144];
#endif

/* Status from algorithm */
arm_status status;

/* storage for configuration of FFT Algorithm */
#ifdef MAKEFFT_FLOAT
arm_cfft_radix2_instance_f32 FFT_State;
arm_cfft_radix2_instance_f32 IFFT_State;
#elif defined(MAKEFFT_Q31)
arm_cfft_radix2_instance_q31 FFT_StateQ31;
arm_cfft_radix2_instance_q31 IFFT_StateQ31;
#elif defined(MAKEFFT_Q15)
arm_cfft_radix2_instance_q15 FFT_StateQ15;
arm_cfft_radix2_instance_q15 IFFT_StateQ15;
#endif


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

   /* clear buffers */
   for (i = 0; i < FFT_SIZE; i++) {
	   OutputBufferFilter1[i] = 0.0f;
	   FFT_TimeHistory[i] = 0.0f;
   }

   /* Just create some nice frequency-response */
   for (i = 0; i <= FFT_SIZE/2; i++)
   {
//        FilterWeights[i] = 0.0;
//        FilterWeights[i] = i*(2.0/FFT_SIZE);
      FilterWeights[i] = (i/8)%4==0?1.0:0.0;
//      FilterWeights[i] = 1.0/(((i/8)%8)+1);
//      FilterWeights[i] = 1.0/(((i/8)%8)*((i/8)%8)+1);
//      FilterWeights[i] = 1.0 / (1 << ((i/8)%8));
   }

   /* mirror the filterweigts for the upper half of the array */
   for (i = 1; i < FFT_SIZE/2; i++)
   {
      FilterWeights[FFT_SIZE-i] =  FilterWeights[i];
   }


   /* generate twiddle factors for fft (code taken from CMSIS/DSP_Lib/arm_common_tables.c) */
#define N 4096
#define PI 3.14159265358979
   for(i = 0; i < 3*N/4; i++)
    {
   	twiddleCoef[2*i]= cos(i * 2*PI/(float)N);
   	twiddleCoef[2*i+1]= sin(i * 2*PI/(float)N);
    }

   /* Create the weights for the windowing-function for the time-domain signal */
   for (i = 0; i < FFT_SIZE; i++)
   {
      WindowWeights[i] = 1.0/(sqrt(4.0*0.54*0.54+2*0.46*0.46))*(0.54 - 0.46*cos(PI/FFT_SIZE*(2*i+1)));
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
/*                blocks up to FFT_SIZE samples, and calculates the FFT over */
/*                the last FFT_SIZE samples.                                 */
/*                The exponential mean of the transformed signal will be     */
/*                presented on channel 1 output                              */
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


	   /* Move the contents of the inputbuffer one quarter down */
	   /* (Length/2 must be one quarter of FFT-Size             */
	   for (i = 0; i < FFT_SIZE -  FFT_SIZE/4; i++) {
		   FFT_TimeHistory[i] = FFT_TimeHistory[FFT_SIZE/4 + i];
	   }

	   /* Put new block of data in last quarter of FFT-Inputbuffer         */
	   /* Samples are normalized (Not really required...)                  */
	   for (i = 0; i <  FFT_SIZE/4; i++) {
           /* Make sample signed by subtracting 2^15 */
		   FFT_TimeHistory[FFT_SIZE -  FFT_SIZE/4 + i] =   ((float32_t)(Channel1_in[i] - 32768))*(1.0f/32768.0f);
	   }

	   /* Apply the weighting (Windowing) function to the inputbuffer and  */
	   /* place weighted samples into FFT-Workbuffer                       */
	   /* Real and Imaginary part of samples are interleaved in workbuffer */
	   for (i = 0; i < FFT_SIZE; i++) {
		   FFT_Workbuffer[2*i] = FFT_TimeHistory[i] * WindowWeights[i];
	       /* Imaginary-part is 0 */
		   FFT_Workbuffer[2*i+1] = 0.0f;
	   }


	  {
	     /* (Led Toggling just for timing measurements) */
	    GPIO_SetBits(GPIOD, GPIO_Pin_0);

	    /* Apply the FFT to the workbuffer */
	    arm_cfft_radix2_f32(&FFT_State, FFT_Workbuffer);

        /* (Led Toggling just for timing measurements) */
	    GPIO_ResetBits(GPIOD, GPIO_Pin_0);

#if 1
	     /* Apply the Filterweights to the frequency-bins */
	     for (i = 0; i < 2*FFT_SIZE; i += 2) {
	        /* Real part */
	    	 FFT_Workbuffer[i]   *= FilterWeights[i/2];

			/* Imaginary part */
	    	 FFT_Workbuffer[i+1] *= FilterWeights[i/2];
	     }
#endif

	     /* and transform back to time domain */
	     arm_cfft_radix2_f32(&IFFT_State, FFT_Workbuffer);

	     /* (Led Toggling just for timing measurements) */
	     GPIO_SetBits(GPIOD, GPIO_Pin_0);

	  }


	   /* Move the contents of the outputbuffer one quarter down */
	   /* (Length/2 must be one quarter of FFT-Size             */
	   for (i = 0; i < FFT_SIZE -  FFT_SIZE/4; i++) {
	       OutputBufferFilter1[i] = OutputBufferFilter1[FFT_SIZE/4 + i];
	   }

	   /* for the lower 3/4 of the Work/Outputbuffer:                    */
	   /* Apply the weighting (Windowing) function to the FFT-Workbuffer */
	   /* (Result of the IFFT) and add the weighted samples to the       */
	   /* outputbuffer                                                   */
	   /* Real and Imaginary part of samples are interleaved in workbuffer */
	   for (i = 0; i <  FFT_SIZE -  FFT_SIZE/4; i++) {
	       OutputBufferFilter1[i] += FFT_Workbuffer[2*i] * WindowWeights[i];
	   }

	   /* For the highest quarter of the Work/Outputbuffer:                    */
	   /* Apply the weighting (Windowing) function to the FFT-Workbuffer */
	   /* (Result of the IFFT) and copy the weighted samples to the       */
	   /* outputbuffer                                                   */
	   /* Real and Imaginary part of samples are interleaved in workbuffer */
	   for (; i <  FFT_SIZE; i++) {
	       OutputBufferFilter1[i] = FFT_Workbuffer[2*i] * WindowWeights[i];
	   }


	   /* Just copies the result of the filtering process to the output  */
	   /* (eg the content of OutputBufferFilter1) and undoes normalizing */
	   for (j = 0; j < FFT_SIZE/4; j++) {
	      float Result =  OutputBufferFilter1[j] * 32768.0f;

	      /* Saturate result */
	      if (Result > 32767.0) {
	         Result = 32767.0;
	      }
	      if (Result < -32767.0) {
	         Result = -32767.0;
	      }
	      /* Make unsigned */
	      Channel1_out[j] = Result+32768;

          /* Unprocessed samples on output 2 */
          Channel2_out[j] = Channel2_in[j];
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
void InitProcessing(void) {

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

   /* clear buffers */
   for (i = 0; i < FFT_SIZE; i++) {
	   OutputBufferFilter1Q31[i] = 0;
	   FFT_TimeHistoryQ31[i] = 0;
   }

   /* Just create some nice frequency-response */
   for (i = 0; i <= FFT_SIZE/2; i++)
   {
//        FilterWeights[i] = 0.0;
//        FilterWeights[i] = i*(2.0/FFT_SIZE);
      FilterWeightsQ31[i] = (i/8)%4==0?0x7FFFFFFFL:0;
//      FilterWeights[i] = 1.0/(((i/8)%8)+1);
//      FilterWeights[i] = 1.0/(((i/8)%8)*((i/8)%8)+1);
//      FilterWeights[i] = 1.0 / (1 << ((i/8)%8));
   }

   /* mirror the filterweigts for the upper half of the array */
   for (i = 1; i < FFT_SIZE/2; i++)
   {
      FilterWeightsQ31[FFT_SIZE-i] =  FilterWeightsQ31[i];
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


   /* Create the weights for the windowing-function for the time-domain signal */
   for (i = 0; i < FFT_SIZE; i++)
   {
	   double val =  1.0/(sqrt(4.0*0.54*0.54+2*0.46*0.46))*(0.54 - 0.46*cos(PI/FFT_SIZE*(2*i+1)))*0x80000000u+0.5;
	   if (val > 0x7fffffffLu) {
	    val = 0x7fffffffLu;
	   }
	   if (val < -(256.0*256*256*127)) {
	    val = -(256.0*256*256*127);
	   }
       WindowWeightsQ31[i] = val;
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
/*                blocks up to FFT_SIZE samples, and calculates the FFT over */
/*                the last FFT_SIZE samples.                                 */
/*                The exponential mean of the transformed signal will be     */
/*                presented on channel 1 output                              */
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

	   /* Move the contents of the inputbuffer one quarter down */
	   /* (Length/2 must be one quarter of FFT-Size)            */
	   for (i = 0; i < FFT_SIZE -  FFT_SIZE/4; i++) {
		   FFT_TimeHistoryQ31[i] = FFT_TimeHistoryQ31[FFT_SIZE/4 + i];
	   }

	   /* Put new block of data in last quarter of FFT-Inputbuffer         */
	   /* Samples are converted to Q31 normalized                          */
	   for (i = 0; i <  FFT_SIZE/4; i++) {
           /* Make sample signed by subtracting 2^15 */
		   FFT_TimeHistoryQ31[FFT_SIZE -  FFT_SIZE/4 + i] =   ((q31_t)(Channel1_in[i] - 32768))<<16;
	   }

	   /* Apply the weighting (Windowing) function to the inputbuffer and  */
	   /* place weighted samples into FFT-Workbuffer                       */
	   /* Real and Imaginary part of samples are interleaved in workbuffer */
	   for (i = 0; i < FFT_SIZE; i++) {
		   // rq31 = ((q63_t) o1q31 * o2q31 >> 31;
		   FFT_WorkbufferQ31[2*i] = ((q63_t)FFT_TimeHistoryQ31[i] * WindowWeightsQ31[i]) >> 31;
	       /* Imaginary-part is 0 */
		   FFT_WorkbufferQ31[2*i+1] = 0;
	   }

        /* (Led Toggling just for timing measurements) */
	    GPIO_SetBits(GPIOD, GPIO_Pin_0);

	    /* Apply the FFT to the workbuffer */
	    arm_cfft_radix2_q31(&FFT_StateQ31, FFT_WorkbufferQ31);

        /* (Led Toggling just for timing measurements) */
        GPIO_ResetBits(GPIOD, GPIO_Pin_0);

#if 1
	     /* Apply the Filterweights to the frequency-bins */
	     for (i = 0; i < 2*FFT_SIZE; i += 2) {
	         /* Real part */
             // rq31 = ((q63_t) o1q31 * o2q31 >> 31;
	    	 FFT_WorkbufferQ31[i]   = ((q63_t)FFT_WorkbufferQ31[i]* FilterWeightsQ31[i/2]) >> 31;

	 		 /* Imaginary part */
	         // rq31 = ((q63_t) o1q31 * o2q31 >> 31;
	    	 FFT_WorkbufferQ31[i+1] = ((q63_t)FFT_WorkbufferQ31[i+1]* FilterWeightsQ31[i/2]) >> 31;
	     }
#endif

	     /* and transform back to time domain */
		 arm_cfft_radix2_q31(&IFFT_StateQ31, FFT_WorkbufferQ31);

         /* (Led Toggling just for timing measurements) */
		 GPIO_SetBits(GPIOD, GPIO_Pin_0);

	   /* Move the contents of the outputbuffer one quarter down */
	   /* (Length/2) must be one quarter of FFT-Size             */
	   for (i = 0; i < FFT_SIZE -  FFT_SIZE/4; i++) {
	       OutputBufferFilter1Q31[i] = OutputBufferFilter1Q31[FFT_SIZE/4 + i];
	   }

	   /* for the lower 3/4 of the Work/Outputbuffer:                    */
	   /* Apply the weighting (Windowing) function to the FFT-Workbuffer */
	   /* (Result of the IFFT) and add the weighted samples to the       */
	   /* outputbuffer                                                   */
	   /* Real and Imaginary part of samples are interleaved in workbuffer */
	   for (i = 0; i <  FFT_SIZE -  FFT_SIZE/4; i++) {
	       OutputBufferFilter1Q31[i] += ((q63_t)FFT_WorkbufferQ31[2*i] * WindowWeightsQ31[i]) >> 31;
	   }

	   /* For the highest quarter of the Work/Outputbuffer:                    */
	   /* Apply the weighting (Windowing) function to the FFT-Workbuffer */
	   /* (Result of the IFFT) and copy the weighted samples to the       */
	   /* outputbuffer                                                   */
	   /* Real and Imaginary part of samples are interleaved in workbuffer */
	   for (; i <  FFT_SIZE; i++) {
	       OutputBufferFilter1Q31[i] = ((q63_t)FFT_WorkbufferQ31[2*i] * WindowWeightsQ31[i]) >> 31;
	   }

	   /* Just copies the result of the filtering process to the output  */
	   /* (eg the content of OutputBufferFilter1) and undoes normalizing */
	   for (j = 0; j < FFT_SIZE/4; j++) {
	      float Result =  ((OutputBufferFilter1Q31[j]>>(16-LOG_2_FFTSIZE-2)));

	      /* Saturate result */
          if (Result > 32767.0) {
	         Result = 32767.0;
	      }
	      if (Result < -32767.0) {
	         Result = -32767.0;
	      }
          /* Make unsigned */
	      Channel1_out[j] = Result+32768;

          /* Unprocessed samples on output 2 */
          Channel2_out[j] = Channel2_in[j];
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

#error Q15 variant untested, might not work
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
void InitProcessing(void) {

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

   /* clear buffers */
   for (i = 0; i < FFT_SIZE; i++) {
	   OutputBufferFilter1Q15[i] = 0;
	   FFT_TimeHistoryQ15[i] = 0;
   }

   /* Just create some nice frequency-response */
   for (i = 0; i <= FFT_SIZE/2; i++)
   {
//        FilterWeights[i] = 0.0;
//        FilterWeights[i] = i*(2.0/FFT_SIZE);
      FilterWeightsQ15[i] = (i/8)%4==0?0x7FFF:0;
//      FilterWeights[i] = 1.0/(((i/8)%8)+1);
//      FilterWeights[i] = 1.0/(((i/8)%8)*((i/8)%8)+1);
//      FilterWeights[i] = 1.0 / (1 << ((i/8)%8));
   }

   /* mirror the filterweigts for the upper half of the array */
   for (i = 1; i < FFT_SIZE/2; i++)
   {
      FilterWeightsQ15[FFT_SIZE-i] =  FilterWeightsQ15[i];
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

   /* Create the weights for the windowing-function for the time-domain signal */
   for (i = 0; i < FFT_SIZE; i++)
   {
	   double val =  1.0/(sqrt(4.0*0.54*0.54+2*0.46*0.46))*(0.54 - 0.46*cos(PI/FFT_SIZE*(2*i+1)))*0x8000u+0.5;
	   if (val > 0x7fffLu) {
	    val = 0x7fffLu;
	   }
	   if (val < -(32768)) {
	    val = -(32768);
	   }
       WindowWeightsQ15[i] = val;
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
/*                blocks up to FFT_SIZE samples, and calculates the FFT over */
/*                the last FFT_SIZE samples.                                 */
/*                The exponential mean of the transformed signal will be     */
/*                presented on channel 1 output                              */
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
    static float32_t val;
    q15_t mul1;
    q15_t x;

    /* procedure code */

	   /* Move the contents of the inputbuffer one quarter down */
	   /* (Length/2 must be one quarter of FFT-Size             */
	   for (i = 0; i < FFT_SIZE -  FFT_SIZE/4; i++) {
		   FFT_TimeHistoryQ15[i] = FFT_TimeHistoryQ15[FFT_SIZE/4 + i];
	   }

	   /* Put new block of data in last quarter of FFT-Inputbuffer         */
	   for (i = 0; i <  FFT_SIZE/4; i++) {
           /* Make sample signed by subtracting 2^15 */
		   FFT_TimeHistoryQ15[FFT_SIZE -  FFT_SIZE/4 + i] =  Channel1_in[i] - 32768;
	   }

	   /* Apply the weighting (Windowing) function to the inputbuffer and  */
	   /* place weighted samples into FFT-Workbuffer                       */
	   /* Real and Imaginary part of samples are interleaved in workbuffer */
	   for (i = 0; i < FFT_SIZE; i++) {
		   // rq31 = ((q31_t) o1q31 * o2q31 >> 15;
		   FFT_WorkbufferQ15[2*i] = ((q31_t)FFT_TimeHistoryQ15[i] * WindowWeightsQ15[i]) >> 15;
	       /* Imaginary-part is 0 */
		   FFT_WorkbufferQ15[2*i+1] = 0;
	   }

       /* (Led Toggling just for timing measurements) */
	   GPIO_SetBits(GPIOD, GPIO_Pin_0);

	   /* Apply the FFT to the workbuffer  */
	   arm_cfft_radix2_q15(&FFT_StateQ15, FFT_WorkbufferQ15);

	   /* (Led Toggling just for timing measurements) */
	   GPIO_ResetBits(GPIOD, GPIO_Pin_0);

	#if 1
	     /* Apply the Filterweights to the frequency-bins */
	     for (i = 0; i < 2*FFT_SIZE; i += 2) {
	        /* Real part */
	    	 FFT_WorkbufferQ15[i]   = ((q31_t)FFT_WorkbufferQ15[i]* FilterWeightsQ15[i/2]) >> 15;

			/* Imaginary part */
	    	 FFT_WorkbufferQ15[i+1] = ((q31_t)FFT_WorkbufferQ15[i+1]* FilterWeightsQ15[i/2]) >> 15;
	     }
	#endif

	   /* and transform back to time domain */
	   arm_cfft_radix2_q15(&IFFT_StateQ15, FFT_WorkbufferQ15);

       /* (Led Toggling just for timing measurements) */
       GPIO_SetBits(GPIOD, GPIO_Pin_0);


	   /* Move the contents of the outputbuffer one quarter down */
	   /* (Length/2 must be one quarter of FFT-Size             */
	   for (i = 0; i < FFT_SIZE -  FFT_SIZE/4; i++) {
	       OutputBufferFilter1Q15[i] = OutputBufferFilter1Q15[FFT_SIZE/4 + i];
	   }

	   /* for the lower 3/4 of the Work/Outputbuffer:                    */
	   /* Apply the weighting (Windowing) function to the FFT-Workbuffer */
	   /* (Result of the IFFT) and add the weighted samples to the       */
	   /* outputbuffer                                                   */
	   /* Real and Imaginary part of samples are interleaved in workbuffer */
	   for (i = 0; i <  FFT_SIZE -  FFT_SIZE/4; i++) {
	       OutputBufferFilter1Q15[i] += ((q31_t)FFT_WorkbufferQ15[2*i] * WindowWeightsQ15[i]) >> 15;
	   }

	   /* For the highest quarter of the Work/Outputbuffer:                    */
	   /* Apply the weighting (Windowing) function to the FFT-Workbuffer */
	   /* (Result of the IFFT) and copy the weighted samples to the       */
	   /* outputbuffer                                                   */
	   /* Real and Imaginary part of samples are interleaved in workbuffer */
	   for (; i <  FFT_SIZE; i++) {
	       OutputBufferFilter1Q15[i] = ((q31_t)FFT_WorkbufferQ15[2*i] * WindowWeightsQ15[i]) >> 15;
	   }


	   /* Just copies the result of the filtering process to the output  */
	   /* (eg the content of OutputBufferFilter1) and undoes normalizing */
	   for (j = 0; j < FFT_SIZE/4; j++) {
		  q31_t Result =   (OutputBufferFilter1Q15[2*i] << (LOG_2_FFTSIZE+2))+ 32768;

		  /* Saturate result */
	      if (Result > 32767) {
	         Result = 32767;
	      }
	      if (Result < -32767) {
	         Result = -32767;
	      }
          /* Make unsigned */
	      Channel1_out[j] = Result+32768;

          /* Unprocessed samples on output 2 */
          Channel2_out[j] = Channel2_in[j];
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
/*  End Module  : FFT-Filterbank-Demo                                        */
/*****************************************************************************/


