#ifndef DSPMAIN_H
#define DSPMAIN_H
/*****************************************************************************/
/*  Header     : DSP_Main                                       Version 1.0  */
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
/*  Procedures : none                                                        */
/*                                                                           */
/*  Author     : I. Oesch                                                    */
/*                                                                           */
/*  History    : 10.09.2013  IO Created                                      */
/*                                                                           */
/*  File       : DSPMain.h                                                   */
/*                                                                           */
/*****************************************************************************/
/*  Berner Fachhochschule   *      Fachbereich EKT                           */
/*  TI Burgdorf             *      Digitale Signalverarbeitung               */
/*****************************************************************************/

/* imports */
#include "arm_dot_prod_q15.c"
#include "../Libraries/CMSIS/DSP_Lib/Source/MatrixFunctions/arm_mat_add_q15.c"

/* module constant declaration  */

/* module type declaration      */

/* module data declaration      */

/* module procedure declaration */
extern void FatalError(void);

/*****************************************************************************/
/*  End Header  : DSP_Main                                                   */
/*****************************************************************************/
#endif
