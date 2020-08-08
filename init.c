/*
 * File:       init.c
 * Author:     Peter Thornton
 * Purpose:    Initialize all required PIC24F peripherals
 * Created on: 13 May 2020
 *  
*/

#include "xc.h"
#include "uart.h"
#include "spi.h"
#include "i2c.h"
#include "adc.h"
#include "init.h"

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
    
    // initialize the I2C peripherals
    // I2C 1 used for: RTC, iMTQ, EPS, Bat, ANTS, Arducam1
    // I2C 2 used for: Arducam2
    init_data_p->brg1_reload = init_i2c1(FOSC, init_data_p->i2c1br);
    init_data_p->brg2_reload = init_i2c2(FOSC, init_data_p->i2c2br);
    
    // initialize the 10-bit ADC peripheral, for sun sensor data
    init_data_p->adc_iserror = init_adc();
}
