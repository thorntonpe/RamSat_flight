/* 
 * File:     init.h
 * Author:   Peter Thornton
 * Purpose:  data type and prototypes for initialization routines
 * Created:  13 May 2020
 */

#include "clock.h"

// data types
typedef struct
{
    long u2br_request;   // UART2 requested baud rate
    long u2br_actual;    // UART2 actual baud rate
} init_data_type;

// function prototypes
void init_peripherals(init_data_type *init_data_p);




