/* 
 * File:     clock.h
 * Author:   Peter Thornton
 * Purpose:  clock and timing parameters
 * Created:  13 May 2020
 */

#define FOSC 32000000    // define FOSC=clock speed in Hz
// a standard 1 millisecond delay for TIMER1, assuming 1:256 prescalar
#define DELAYMSEC 62L    // = 0.001 * FOSC / (2 * 256)





