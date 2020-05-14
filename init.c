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
#include "spi.h"

void init_peripherals(init_data_type *init_data_p)
{
    // Initialize the UART2 peripheral (He-100 transceiver or USB))
    init_data_p->u2br_actual = init_uart2(FOSC, init_data_p->u2br_request);
    
    // Initialize the SPI peripherals:
    // SPI1: SD-card interface (on MBM)
    // SPI2: Arducam interfaces (camera #1 and camera #2)
    // SPI3: Serial Flash Memory interface (on PPM)
    init_data_p->spi1_prescalar = init_spi1(FOSC, init_data_p->spi1_fsck);
    init_data_p->spi2_prescalar = init_spi2(FOSC, init_data_p->spi2_fsck);
    init_data_p->spi3_prescalar = init_spi3(FOSC, init_data_p->spi3_fsck);
}
