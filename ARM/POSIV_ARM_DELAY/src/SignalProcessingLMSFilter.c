/* general control */

/*****************************************************************************/
/*  Module     : FIR-Filter-Demo                                Version 1.0  */
/*****************************************************************************/
/*                                                                           */
/*  Function   : A Demo for float32, Q31 and Q16 Fir Filter, filters         */
/*               channel 1                                                   */
/*               This is just an example to demonstrate the usage of the     */
/*               FIR-filter routines from the arm-CMSIS-DSP-Lib              */
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
/*  File       : SignalProcessingFIR.c                                       */
/*                                                                           */
/*****************************************************************************/
/*  Berner Fachhochschule   *      Fachbereich EKT                           */
/*  TI Burgdorf             *      Digitale Signalverarbeitung               */
/*****************************************************************************/

/* imports */
#include "SignalProcessing.h"
#include "stm32f4_discovery.h"
#include <math.h>

/* module constant declaration */

/* Select the number format */
//#define MAKEFIR_FLOAT
//#define MAKEFIR_Q31
#define MAKEFIR_Q15


/* module type declaration */

/* module data declaration */


#ifdef MAKEFIR_FLOAT
/* Buffers for float variant */
float32_t OutBuffer[BLOCK_SIZE];
float32_t InBuffer[BLOCK_SIZE];

// Filter Bandpass, 40dB Daempfung, Sperr bis 6000Hz, ab 15000Hz, Durchlass 9000Hz- 12000HzFs=44100,
#define FILTER_LENGTH 32
float32_t State[FILTER_LENGTH + BLOCK_SIZE - 1];
// float-coefficients for FIR-Filter
float32_t Coeffs[FILTER_LENGTH] =
{

    -0.00128356873484252380,
    -0.00513960226177860750,
    -0.00035466705579610116,
    -0.01448468075355693700,
    0.00212448554574338340,
    0.03077746209558913900,
    0.00228142564397016030,
    -0.02673835503266832100,
    -0.00193924305902075720,
    -0.02123709708282432400,
    -0.02359241451296175500,
    0.10478497364525105000,
    0.08450995954559575900,
    -0.18064319901032852000,
    -0.15928162900584514000,
    0.20232399997168940000,
    0.20232399997168940000,
    -0.15928162900584514000,
    -0.18064319901032852000,
    0.08450995954559575900,
    0.10478497364525105000,
    -0.02359241451296175500,
    -0.02123709708282432400,
    -0.00193924305902075720,
    -0.02673835503266832100,
    0.00228142564397016030,
    0.03077746209558913900,
    0.00212448554574338340,
    -0.01448468075355693700,
    -0.00035466705579610116,
    -0.00513960226177860750,
    -0.00128356873484252380
};

#elif defined(MAKEFIR_Q31)

/* Buffers for q32 variant */
q31_t OutBufferQ31[BLOCK_SIZE];
q31_t InBufferQ31[BLOCK_SIZE];

// Filter Bandpass, 40dB Daempfung, Sperr bis 6000Hz, ab 15000Hz, Durchlass 9000Hz- 12000HzFs=44100,
#define FILTER_LENGTH 32
q31_t StateQ31[FILTER_LENGTH + BLOCK_SIZE - 1];
// q32-coefficients for FIR-Filter
q31_t CoeffsQ31[FILTER_LENGTH] =
{   -2756443,
    -11037212,
    -761642,
    -31105615,
    4562298,
    66094097,
    4899324,
    -57420180,
    -4164493,
    -45606319,
    -50664324,
    225024017,
    181483756,
    -387928316,
    -342054694,
    434487482,
    434487482,
    -342054694,
    -387928316,
    181483756,
    225024017,
    -50664324,
    -45606319,
    -4164493,
    -57420180,
    4899324,
    66094097,
    4562298,
    -31105615,
    -761642,
    -11037212,
    -2756443

};

#elif defined(MAKEFIR_Q15)

/* Buffers for q15 variant */
q15_t OutBufferQ15[BLOCK_SIZE];
q15_t xQ15[BLOCK_SIZE];
q15_t yQ15[BLOCK_SIZE];
// Filter Bandpass, 40dB Daempfung, Sperr bis 6000Hz, ab 15000Hz, Durchlass 9000Hz- 12000HzFs=44100,
#define FILTER_LENGTH 1600
q15_t StateQ15[FILTER_LENGTH + BLOCK_SIZE - 1];
// q16-coefficients for FIR-Filter
q15_t CoeffsQ15[FILTER_LENGTH];


#endif

/* storage for configuration of FIR Algorithm */
#ifdef MAKEFIR_FLOAT
arm_fir_instance_f32 FirStateFloat;
#elif defined(MAKEFIR_Q31)
arm_fir_instance_q31 FirStateQ31;
#elif defined(MAKEFIR_Q15)
arm_fir_instance_q15 FirStateQ15;
#endif


#ifdef MAKEFIR_FLOAT
/*****************************************************************************/
/*  Procedure   : InitProcessing                                             */
/*****************************************************************************/
/*                                                                           */
/*  Function    : Is called from main, before starting the System to         */
/*                initialize all required buffers and tables.                */
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

    /* procedure code */

    /* Initialize the FIR module */
    arm_fir_init_f32(&FirStateFloat, FILTER_LENGTH, Coeffs, State, BLOCK_SIZE);

}
/*****************************************************************************/
/*  End         : InitProcessing                                             */
/*****************************************************************************/


/*****************************************************************************/
/*  Procedure   : ProcessBlock                                               */
/*****************************************************************************/
/*                                                                           */
/*  Function    : Is called from the DMA-interrupt whenever a block of new   */
/*                samples has been collected. This Function just calls       */
/*                the FIR algorithm for the given block of samples           */
/*                (For samplewise processing BLOCK_SIZE must be 1)           */
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
    int i;

    /* procedure code */

    /* Copy samples into workbuffer and convert to signed */
    for (i = 0; i < BLOCK_SIZE; i++) {
        InBuffer[i] = ((float32_t)(Channel1_in[i] - 32768));
    }


    /* Set bit, just for time measurements */
    GPIO_SetBits(GPIOD, GPIO_Pin_0);

    /* Filter one block of samples */
    arm_fir_f32 (&FirStateFloat, InBuffer, OutBuffer, BLOCK_SIZE);

    /* Reset bit, just for time measurements */
    GPIO_ResetBits(GPIOD, GPIO_Pin_0);


    /* Copy filtered samples to outputbuffer and convert to unsigned */
    for (i = 0; i < BLOCK_SIZE; i++) {

        /* Filtered samples on output 1, make unsigned */
        Channel1_out[i] = OutBuffer[i]+32678;

        /* Unfiltered samples on output 2 */
        Channel2_out[i] = Channel2_in[i];
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
/*****************************************************************************/
/*  End         : ProcessBlock                                               */
/*****************************************************************************/

#elif defined(MAKEFIR_Q31)

/*****************************************************************************/
/*  Procedure   : InitProcessing                                             */
/*****************************************************************************/
/*                                                                           */
/*  Function    : Is called from main, before starting the System to         */
/*                initialize all required buffers and tables.                */
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

    /* procedure code */

    /* Initialize the FIR module */
    arm_fir_init_q31(&FirStateQ31, FILTER_LENGTH, CoeffsQ31, StateQ31, BLOCK_SIZE);

}
/*****************************************************************************/
/*  End         : InitProcessing                                             */
/*****************************************************************************/


/*****************************************************************************/
/*  Procedure   : ProcessBlock                                               */
/*****************************************************************************/
/*                                                                           */
/*  Function    : Is called from the DMA-interrupt whenever a block of new   */
/*                samples has been collected. This Function just calls       */
/*                the FIR algorithm for the given block of samples           */
/*                (For samplewise processing BLOCK_SIZE must be 1)           */
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
    int i;

    /* procedure code */

    /* Copy samples into workbuffer and convert to signed */
    for (i = 0; i < BLOCK_SIZE; i++) {
        InBufferQ31[i] = ((q31_t)(Channel1_in[i] - 32768))<<16;
    }

    /* Set bit, just for time measurements */
    GPIO_SetBits(GPIOD, GPIO_Pin_0);

    /* Filter one block of samples */
    arm_fir_q31 (&FirStateQ31, InBufferQ31, OutBufferQ31, BLOCK_SIZE);

    /* Reset bit, just for time measurements */
    GPIO_ResetBits(GPIOD, GPIO_Pin_0);

    /* Copy filtered samples to outputbuffer and convert to unsigned */
    for (i = 0; i < BLOCK_SIZE; i++) {

        /* Filtered samples on output 1, make unsigned  */
        Channel1_out[i] = ((OutBufferQ31[i]>>16)+32678);

        /* Unfiltered samples on output 2 */
        Channel2_out[i] = Channel2_in[i];
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
/*****************************************************************************/
/*  End         : ProcessBlock                                               */
/*****************************************************************************/

#elif defined(MAKEFIR_Q15)

/*****************************************************************************/
/*  Procedure   : InitProcessing                                             */
/*****************************************************************************/
/*                                                                           */
/*  Function    : Is called from main, before starting the System to         */
/*                initialize all required buffers and tables.                */
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

    /* Initialize Delay Filter */
	CoeffsQ15[0] = 2767;
	CoeffsQ15[400] = 4767;
	CoeffsQ15[800] = 8767;
	CoeffsQ15[1200] = 16767;
	//CoeffsQ15[1599] = 32767;
    /* procedure code */

    /* Initialize the FIR module */
    arm_fir_init_q15(&FirStateQ15, FILTER_LENGTH, CoeffsQ15, StateQ15, BLOCK_SIZE);

}
/*****************************************************************************/
/*  End         : InitProcessing                                             */
/*****************************************************************************/


/*****************************************************************************/
/*  Procedure   : ProcessBlock                                               */
/*****************************************************************************/
/*                                                                           */
/*  Function    : Is called from the DMA-interrupt whenever a block of new   */
/*                samples has been collected. This Function just calls       */
/*                the FIR algorithm for the given block of samples           */
/*                (For samplewise processing BLOCK_SIZE must be 1)           */
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
void ProcessBlock(uint16_t *Channel1_in, uint16_t *Channel2_in, uint16_t *Channel1_out, uint16_t *Channel2_out) {

    /* procedure data */
    int i;
    q15_t err[BLOCK_SIZE];
    /* procedure code */

    /* Copy samples into workbuffer and convert to signed */
    for (i = 0; i < BLOCK_SIZE; i++) {
        xQ15[i] = ((q15_t) (Channel1_in[i] - 32768));
        yQ15[i] = ((q15_t) (Channel2_in[i] - 32768));
    }

    /* Set bit, just for time measurements */
    GPIO_SetBits(GPIOD, GPIO_Pin_0 );

    /* Filter one block of samples */
    arm_fir_q15(&FirStateQ15, xQ15, OutBufferQ15, BLOCK_SIZE);
    for(i = 0; i<BLOCK_SIZE;i++){
    	err[i] = yQ15[i] - OutBufferQ15[i];
    }

    /* Reset bit, just for time measurements */
    GPIO_ResetBits(GPIOD, GPIO_Pin_0);

    /* Copy filtered samples to outputbuffer and convert to unsigned */
    for (i = 0; i < BLOCK_SIZE; i++) {

        /* Filtered samples on output 1, make unsigned  */
        Channel1_out[i] = err[i] + 32678;

        /* Unfiltered samples on output 2 */
        Channel2_out[i] = err[i] + 32678;
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
        x = Channel1_in[i] - 32768;
        mul1 = (q31_t) ((q15_t) x * (q15_t) x);
        Channel1_out[i] = (q15_t) __SSAT(mul1 >> 15, 16);
        x = Channel2_in[i] - 32768;
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

