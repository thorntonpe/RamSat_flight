/*
 * File:       RamSat_MBM_test1_main.c
 * Author:     Peter Thornton
 * Purpose:    RamSat flight software
 * Created on: 12 May 2020
 */


#include "xc.h"
#include "PIC_config.h"
#include "init.h"
#include "uart.h"
#include "sfm.h"
#include "sd_test.h"
#include <stdio.h>
#include <string.h>
#include <math.h>

// Configuration for ground testing and flight configurations
// Input/Output configuration: Define USB or HE100, but not both
#define USB      // Enable USB I/O (ground testing only)
//#define HE100    // Enable He-100 transceiver I/O (ground testing or flight)

int main(void) {
    
    init_data_type init_data; // data structure for peripheral initialization
    
    // Initialize TIMER1 (16-bit), using system clock and 1:256 prescalar
    T1CON = 0x8030;
    long wait;          // Timer trigger
    int ui;             // user input, for ground testing

    // Set desired baud rates for UART2 (He-100 radio or USB interfaces)
#ifdef HE100
    init_data.u2br_request = 9600;   // UART2 desired baud rate for radio
#endif
#ifdef USB    
    init_data.u2br_request = 115200; // UART2 desired baud rate for COM port
    char msg[128];      // character string for messages to user via COM port
#endif
    
    // set desired clock speeds for SPI peripherals
    init_data.spi1_fsck = 250000;  // SD card, initial speed (250kHz)
    init_data.spi2_fsck = 4000000; // Arducams (4MHz)
    init_data.spi3_fsck = 4000000; // Serial Flash Memory (4MHz)
    
    // Initialize the PIC peripherals
    init_peripherals(&init_data);

#ifdef USB
    // enable level translation for USB I/O through U1 and U16 chips
    _TRISC1 = 0;        // Set Port C, pin 1 as output (OE-USB)
    _RC1 = 0;           // set the OE-USB signal low, to allow level translation
#endif

    // give all devices an adequate time to reset after power-on
    wait = 1000 * DELAYMSEC;
    TMR1 = 0;
    while (TMR1 < wait);
    
    // Tests of components that use PIC peripherals
    // These might later get packaged as a single test routine with telemetry
    // SPI components:
    //     Serial flash memory: write-read
    int sfm_iserror = test_sfm();
    //     SD card: write-read-delete
    int sd_iserror = test_sd_write_read_delete();
    
    
#ifdef USB
    // Common header for ground testing output
    write_string2("---------------------------------------------");
    write_string2("RamSat flight software: ground testing output");
    write_string2("---------------------------------------------");
    sprintf(msg,"UART2: Requested baud rate = %ld", init_data.u2br_request);
    write_string2(msg);
    sprintf(msg,"UART2: Actual baud rate    = %ld", init_data.u2br_actual);
    write_string2(msg);
    sprintf(msg,"SPI1: clock frequency = %ld", init_data.spi1_fsck);
    write_string2(msg);
    sprintf(msg,"SPI1: clock prescalar = %ld", init_data.spi1_prescalar);
    write_string2(msg);
    sprintf(msg,"SPI2: clock frequency = %ld", init_data.spi2_fsck);
    write_string2(msg);
    sprintf(msg,"SPI2: clock prescalar = %ld", init_data.spi2_prescalar);
    write_string2(msg);
    sprintf(msg,"SPI3: clock frequency = %ld", init_data.spi3_fsck);
    write_string2(msg);
    sprintf(msg,"SPI3: clock prescalar = %ld", init_data.spi3_prescalar);
    write_string2(msg);
    sprintf(msg,"SFM: Test is_error = %d", sfm_iserror);
    write_string2(msg);
    sprintf(msg,"SD: Test is_error = %d", sd_iserror);
    write_string2(msg);
    
#endif

    // test the SD card
    // test_sd_mount_describe();

    // hold here
    while (1);

}
