#ifndef CONFIG_H
#define CONFIG_H
/*****************************************************************************/
/*  Header     : Config                                         Version 1.0  */
/*****************************************************************************/
/*                                                                           */
/*  Function   : Configures projectwide settings                             */
/*                                                                           */
/*                                                                           */
/*  Procedures : none                                                        */
/*                                                                           */
/*  Author     : I. Oesch                                                    */
/*                                                                           */
/*  History    : 10.09.2013  IO Created                                      */
/*                                                                           */
/*  File       : config.h                                                    */
/*                                                                           */
/*****************************************************************************/
/*  Berner Fachhochschule   *      Fachbereich EKT                           */
/*  TI Burgdorf             *      Digitale Signalverarbeitung               */
/*****************************************************************************/

/* imports */

/* module constant declaration  */

/* Number of channels to process (1 or 2) */
#define NUMBER_OF_CHANNELS 2

/* Size of block of samples to collect for processing */
#define BLOCK_SIZE 1
#define NFIR 1700
#define DELAY 60

/* Required sampling frequency */
/* (Is derived from SYSCLK, Fs = SYSCLK/round(SYSCLK/Fs)) */
/* nearest possible Fs will be selected */
#define FS 8000

/* System clock, normally 84000000 */
#define SYSCLK 84000000


/* module type declaration      */

/* module data declaration      */

/* module procedure declaration */

/*****************************************************************************/
/*  End Header  : Config                                                     */
/*****************************************************************************/
#endif
