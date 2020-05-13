/*
 * File:    init.c
 * Author:  Peter Thornton
 * Created: 13 May 2020
 * Purpose: Initialize all peripherals
 *  
*/

#include "xc.h"
#include "init.h"
#include "uart.h"

void init_peripherals(init_data_type *init_data_p)
{
    init_data_p->u2br_actual = init_uart2(FOSC, init_data_p->u2br_request);
}
