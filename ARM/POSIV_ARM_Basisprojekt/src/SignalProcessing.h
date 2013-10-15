#ifndef SIGNALPROCESSING_H
#define SIGNALPROCESSING_H
/*****************************************************************************/
/*  Header     : SignalProcessing                               Version 1.0  */
/*****************************************************************************/
/*                                                                           */
/*  Function   : Does the signalprocessing                                   */
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
/*  File       : SignalProcessing.h                                          */
/*                                                                           */
/*****************************************************************************/
/*  Berner Fachhochschule   *      Fachbereich EKT                           */
/*  TI Burgdorf             *      Digitale Signalverarbeitung               */
/*****************************************************************************/

/* imports */
#include "config.h"
#include "arm_math.h"

/* module constant declaration  */

/* Size of block of samples to collect for processing */

/* module type declaration      */

/* module data declaration      */

/* module procedure declaration */
void InitProcessing(void);
void IdleFunction(void);
#if NUMBER_OF_CHANNELS == 2
void ProcessBlock(uint16_t *Channel1_in, uint16_t *Channel2_in, uint16_t *Channel1_out, uint16_t *Channel2_out);
#else
void ProcessBlock(uint16_t *Channel1_in, uint16_t *Channel1_out);
#endif

/*****************************************************************************/
/*  End Header  : SignalProcessing                                           */
/*****************************************************************************/
#endif


