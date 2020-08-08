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
    long spi1_prescalar; // SPI1 clock prescalar
    long spi1_fsck;      // SPI1 clock speed
    long spi2_prescalar; // SPI2 clock prescalar
    long spi2_fsck;      // SPI2 clock speed
    long spi3_prescalar; // SPI3 clock prescalar
    long spi3_fsck;      // SPI3 clock speed    
    long i2c1br;         // desired I2C_1 baud rate = 100 kHz
    long brg1_reload;    // reload value for I2C_1 BRG
    long i2c2br;         // desired I2C_2 baud rate = 100 kHz
    long brg2_reload;    // reload value for I2C_2 BRG
    int  adc_iserror;    // error condition for initialization of ADC
} init_data_type;

// function prototypes
void init_peripherals(init_data_type *init_data_p);




