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
#include "rtc.h"
#include "datetime.h"
#include "adc.h"
#include "eps_bat.h"
#include "arducam_user.h"
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
    init_data.spi1_fsck = 250000;  // SD card, initial speed (250kHz) **check re-mount**
    init_data.spi2_fsck = 4000000; // Arducams (4MHz)
    init_data.spi3_fsck = 4000000; // Serial Flash Memory (4MHz)

    // set desired data rate for I2C peripherals
    // I2C 1 used for: RTC, iMTQ, EPS, Bat, ANTS, Arducam1
    // I2C 2 used for: Arducam2
    init_data.i2c1br = 100000;  // desired I2C_1 baud rate = 100 kHz
    init_data.i2c2br = 100000;  // desired I2C_2 baud rate = 100 kHz
    
    // Initialize the PIC24F peripherals
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
    
    // switch on power to the cameras
    unsigned char cameras_on_status = eps_cameras_on();
    
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
    
    // test the RTC and ISO8601 formatted datetime string function
    write_string2("-----------------------------------");
    write_string2("Test: Real Time Clock and Julian Date");
    write_string2("-----------------------------------");
    rtc_clearhalt();
    char isodatetime[25];
    get_isodatetime(isodatetime);
    write_string2(isodatetime);
    // test the julian date function
    double jdate;
    get_juliandate(&jdate);
    sprintf(msg,"Jdate = %.5lf",jdate);
    write_string2(msg);
    
    // test ADC read from channels AN8-AN15
    write_string2("-----------------------------------");
    write_string2("Test: Sun Sensor Analog to Digital Conversion");
    write_string2("-----------------------------------");
    int ad_result = adc_test_ssac();
    sprintf(msg,"ADC: AN8  = %d",ad_result);
    write_string2(msg);
    sprintf(msg,"ADC: AN9  = %d",ADC1BUF1);
    write_string2(msg);
    sprintf(msg,"ADC: AN10 = %d",ADC1BUF2);
    write_string2(msg);
    sprintf(msg,"ADC: AN11 = %d",ADC1BUF3);
    write_string2(msg);
    sprintf(msg,"ADC: AN12 = %d",ADC1BUF4);
    write_string2(msg);
    sprintf(msg,"ADC: AN13 = %d",ADC1BUF5);
    write_string2(msg);
    sprintf(msg,"ADC: AN14 = %d",ADC1BUF6);
    write_string2(msg);
    sprintf(msg,"ADC: AN15 = %d",ADC1BUF7);
    write_string2(msg);
    
    // test EPS and battery telemetry
    write_string2("-----------------------------------");
    write_string2("Test: EPS / Battery Telemetry");
    write_string2("-----------------------------------");
    unsigned char eps_status = eps_get_status();
    sprintf(msg,"EPS status byte = 0x%02x", eps_status);
    write_string2(msg);
    unsigned char bat_status = bat_get_status();
    sprintf(msg,"Bat status byte = 0x%02x", bat_status);
    write_string2(msg);
    float batv = eps_get_batv();
    sprintf(msg,"Battery voltage = %.2f",batv);
    write_string2(msg);
    
    // test camera interfaces
    write_string2("-----------------------------------");
    write_string2("Test: Arducam Interfaces and Operation");
    write_string2("-----------------------------------");
    sprintf(msg,"Cameras power on status = 0x%02x",cameras_on_status);
    write_string2(msg);
    // test the arducam SPI interface with write/read
    int arducam_spi_iserror = test_arducam_spi();
    sprintf(msg,"Arducam SPI: Test is_error = %d", arducam_spi_iserror);
    write_string2(msg);
    // initialize arduchip and OV2640 sensor for both cameras
    int arducam_init_iserror = init_arducam();
    sprintf(msg,"Arducam Init: Test is_error = %d", arducam_init_iserror);
    write_string2(msg);
    // start the image capture test loop
    int arducam_capture_iserror = test_arducam_capture();
    sprintf(msg,"Arducam Capture: Test is_error = %d", arducam_capture_iserror);
    write_string2(msg);

#endif

    // hold here
    while (1);

}
