/*
 * File:       RamSat_MBM_test1_main.c
 * Author:     Peter Thornton
 * Purpose:    RamSat flight software
 * Created on: 12 May 2020
 */


#include "xc.h"
#include "clock.h"
#include "PIC_config.h"
#include "init.h"
#include "uart.h"
#include "preflight.h"
#include "rtc_user.h"
#include "datetime.h"
#include "adc.h"
#include "eps_bat.h"
#include "arducam_user.h"
//#include "imtq.h"
#include "ants.h"
#include "he100.h"
#include <stdio.h>
#include <string.h>
#include <math.h>

// Configuration for ground testing and flight configurations
// Input/Output configuration: Define USB or HE100, but not both
#define USB      // Enable USB I/O (ground testing only)
//#define HE100    // Enable He-100 transceiver I/O (ground testing or flight)

//#define INIT_RTC   // Pre-flight code to initialize RTC
//#define INIT_PREDEPLOY // Pre-flight code to set MUST_WAIT flag
//#define TEST_ARDUCAM
//#define TEST_IMTQ
//#define TEST_ANTS
//#define ANTS_DEPLOY  // just the arm/disarm steps
//#define ANTS_DEPLOY2 // include the actual deployment steps
//#define UART2_INTERRUPT  // test the use of interrupt handler for incoming UART data

// global variables accessed by interrupt service routines
// ISR variables for the He-100 interface on UART2
volatile unsigned char u2rx_char;
volatile int nhbytes = 0; // number of good header bytes already received
volatile int ndbytes = 0; // number of data payload bytes expected
volatile int data_byte;   // current data byte
volatile int ishead_flag = 0;
volatile int isdata_flag = 0;
volatile int he100_acknack = 0;
volatile int he100_receive = 0;
volatile unsigned char he100_head[8]; // command code, payload length, and checksum bytes
volatile unsigned char he100_data[256]; // data payload from He-100
volatile int data_bytes;
volatile unsigned char u2rx_flag;

#ifdef UART2_INTERRUPT
// test code for UART2 receive interrupt handler
void _ISR _U2RXInterrupt (void)
{
    // read a single character from the UART2 receive buffer
    u2rx_char = U2RXREG;
    // if a header has not been found, keep checking
    if (!ishead_flag)
    {
        // trap the start of an He-100 output on UART2
        if (u2rx_char == 'H')
        {
            // force the start of a new header 
            nhbytes = 0;
            he100_head[nhbytes++] = u2rx_char;
        }
        // trap the second byte of He-100 output
        else if (nhbytes == 1 && u2rx_char == 'e')
        {
            he100_head[nhbytes++] = u2rx_char;
        }
        // trap the third byte of He-100 output, and set flag to gather 
        // remaining header data
        else if (nhbytes == 2 && u2rx_char == 0x20)
        {
            he100_head[nhbytes++]=u2rx_char;
            ishead_flag = 1;
            isdata_flag = 0;
        }
        // any other condition means not a valid header, so reset
        else
        {
            nhbytes = 0;
            ndbytes = 0;
            ishead_flag = 0;
            isdata_flag = 0;
            he100_acknack = 0;
            he100_receive = 0;
        }
    }
    else if (!isdata_flag)
    {
        // gather remaining header data
        // trap the command code, payload size, and checksum bytes
        if (nhbytes < 8)
        {
            he100_head[nhbytes++] = u2rx_char;
        }
        // reached the end of the header, so compute checksum for bytes 2 through 5
        if (nhbytes == 8)
        {
            int i;
            unsigned char ck_a, ck_b;
            ck_a = 0;
            ck_b = 0;
            for (i=2 ; i<6 ; i++)
            {
                ck_a += he100_head[i];
                ck_b += ck_a;
            }
            // check checksum against bytes 6 and 7
            he100_head[6] = ck_a;
            he100_head[7] = ck_b;
            if (ck_a == he100_head[6] && ck_b == he100_head[7])
            {
                // checksum is good: if this is a packet receive, get length
                // from header bytes 4 and 5, set flag to start retrieving payload
                if (he100_head[3] == 0x04)
                {
                    ndbytes = (he100_head[4]<<8 | he100_head[5]);
                    data_byte = 0;
                    isdata_flag = 1;
                }
                // if not a receive packet, set flag that ack/nack is ready
                else
                {
                    he100_acknack = 1;
                }
            }
            // if checksum is bad, reject and reset traps
            else
            {
                nhbytes = 0;
                ndbytes = 0;
                ishead_flag = 0;
                isdata_flag = 0;
                he100_acknack = 0;
                he100_receive = 0;
            }
        }
    }
    // if ishead_flag and isdata_flag are both set, then fill the data array
    else
    {
        if (data_byte < ndbytes)
        {
            he100_data[data_byte++] = u2rx_char;
        }
        // this is the last byte, so set flag that receive is complete
        if (data_byte == ndbytes)
        {
            // set flag that data is ready
            he100_receive = 1;
        }
    }
    // reset the interrupt flag before exit
    _U2RXIF = 0;
}
#endif // UART_INTERRUPT

int main(void) {
    
    init_data_type init_data; // data structure for peripheral initialization
    
    // Initialize TIMER1 (16-bit), using system clock and 1:256 prescalar
    T1CON = 0x8030;
    long wait;          // Timer trigger
    int ui;             // user input, for ground testing
    int i;
    
    // Set desired baud rates for UART2 (He-100 radio or USB interfaces)
#ifdef HE100
    init_data.u2br_request = 9600;   // UART2 desired baud rate for radio
    unsigned char he100_response[8];
    unsigned char he100_telem[26];
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
    // Common header for ground testing output
    write_string2("---------------------------------------------");
    write_string2("RamSat flight software: ground testing output");
    write_string2("Press any key to continue...");
    ui = read_char2();
    write_string2("---------------------------------------------");
#endif
    
    // Initialize the hardware components that are integrated on Pumpkin MBM
    // (serial flash memory, SD card, and real-time clock)
    init_motherboard_components(&init_data);    

    // perform the initial deployment test, and wait if the MUST_WAIT flag is set
    init_wait(&init_data);

#ifdef INIT_RTC
    // This code is to be used pre-flight to set the date and time.
    // Should only be used with USB, not HE100.
    // Enter an infinite user-interface loop with options to display (d) and set (s)
    // clock, and other options to clear the HALT (c), STOP (t), and OF (f) flags.
    preflight_init_rtc();
#endif

#ifdef INIT_PREDEPLOY
    // THis code is to be used pre-flight to set the predeployment flag
    // on the serial flash memory. That flag is examined on powerup, and
    // if set to MUST_WAIT the code forces a 30-minute wait, as required by 
    // Nanoracks for deployment from the ISS.
    preflight_init_predeploy();
#endif    
    
#ifdef TEST_ARDUCAM
    // switch on power to the cameras
    unsigned char cameras_on_status = eps_cameras_on();
#endif

#ifdef TEST_ANTS
    // switch on power to the antenna (deploy and I2C)
    unsigned char antenna_on_status = eps_antenna_on();
#endif
    
#ifdef USB
    write_string2("-----------------------------------");
    write_string2("Test: PIC peripheral clock speeds and integrated devices");
    write_string2("-----------------------------------");
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
    sprintf(msg,"ADC: Initialize is_error = %d", init_data.adc_iserror);
    write_string2(msg);
    sprintf(msg,"SFM: Test is_error = %d", init_data.sfm_iserror);
    write_string2(msg);
    sprintf(msg,"SD: Test is_error = %d", init_data.sd_iserror);
    write_string2(msg);
    sprintf(msg,"RTC: Test flags is_error = %d", init_data.rtc_flags_iserror);
    write_string2(msg);
    sprintf(msg,"RTC: Flags = 0x%02x", init_data.rtc_flags);
    write_string2(msg);
    if (init_data.rtc_flags)
    {
        sprintf(msg,"RTC: Test clear flags is_error = %d", init_data.rtc_clear_iserror);
        write_string2(msg);
        sprintf(msg,"RTC: Test flags2 is_error = %d", init_data.rtc_flags2_iserror);
        write_string2(msg);
        sprintf(msg,"RTC: Flags2 = 0x%02x", init_data.rtc_flags2);
        write_string2(msg);
        sprintf(msg,"RTC: Last halt ISO8601 datetime = ");
        strcat(msg,init_data.rtc_halt_time);
        write_string2(msg);
    }
    sprintf(msg,"PDT: Wait status = %d", init_data.pdt_status);
    write_string2(msg);
    sprintf(msg,"PDT: Wait flag = 0x%02x", init_data.pdt_flag);
    write_string2(msg);
    
    // test the RTC and ISO8601 formatted datetime string function
    write_string2("-----------------------------------");
    write_string2("Test: Real Time Clock and Julian Date");
    write_string2("-----------------------------------");
    char isodatetime[25];
    double jdate;
    get_isodatetime(isodatetime);
    sprintf(msg,"RTC: ISO8601 datetime = ");
    strcat(msg,isodatetime);
    write_string2(msg);
    get_juliandate(&jdate);
    sprintf(msg,"RTC: Julian date = %.5lf",jdate);
    write_string2(msg);
    
    // test ADC read from 8 channels: AN9-AN15, AN17
    write_string2("-----------------------------------");
    write_string2("Test: Sun Sensor Analog to Digital Conversion");
    write_string2("-----------------------------------");
    adc_test_ssac();
    sprintf(msg,"ADC: AN17 = %d",ADC1BUF7);
    write_string2(msg);
    sprintf(msg,"ADC: AN9  = %d",ADC1BUF0);
    write_string2(msg);
    sprintf(msg,"ADC: AN10 = %d",ADC1BUF1);
    write_string2(msg);
    sprintf(msg,"ADC: AN11 = %d",ADC1BUF2);
    write_string2(msg);
    sprintf(msg,"ADC: AN12 = %d",ADC1BUF3);
    write_string2(msg);
    sprintf(msg,"ADC: AN13 = %d",ADC1BUF4);
    write_string2(msg);
    sprintf(msg,"ADC: AN14 = %d",ADC1BUF5);
    write_string2(msg);
    sprintf(msg,"ADC: AN15 = %d",ADC1BUF6);
    write_string2(msg);

    // hold here
    while (1);

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
    float bcr1v = eps_get_bcr1v();
    float bcr2v = eps_get_bcr2v();
    float bcr3v = eps_get_bcr3v();
    float bcroutv = eps_get_bcroutv();
    sprintf(msg,"BCR Voltages: %.2f %.2f %.2f %.2f",bcr1v, bcr2v, bcr3v, bcroutv);
    write_string2(msg);
    float bcr1ia = eps_get_bcr1ia();
    float bcr1ib = eps_get_bcr1ib();
    float bcr2ia = eps_get_bcr2ia();
    float bcr2ib = eps_get_bcr2ib();
    float bcr3ia = eps_get_bcr3ia();
    float bcr3ib = eps_get_bcr3ib();
    sprintf(msg,"BCR Currents: %.2f %.2f %.2f %.2f %.2f %.2f",bcr1ia, bcr1ib, bcr2ia, bcr2ib, bcr3ia, bcr3ib);
    write_string2(msg);

    float bati = bat_get_bati();
    int ischarging = bat_get_batischarging();
    sprintf(msg,"Bat current (is_charge) = %.2f, %d", bati, ischarging);
    write_string2(msg);


#ifdef TEST_ARDUCAM
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
    //int arducam_capture_iserror = test_arducam_capture();
    //sprintf(msg,"Arducam Capture: Test is_error = %d", arducam_capture_iserror);
    //write_string2(msg);
#endif
    
#ifdef TEST_IMTQ
    // Simple interface test for iMTQ (magnetorquer)
    write_string2("-----------------------------------");
    write_string2("Test: iMTQ interface (no-op command)");
    write_string2("-----------------------------------");
    imtq_resp_common imtq_common;       // iMTQ response from every command
    imtq_no_op(&imtq_common);
    sprintf(msg,"iMTQ response: command ID = 0x%02x", imtq_common.cc);
    write_string2(msg);
    sprintf(msg,"iMTQ response: status byte = 0x%02x", imtq_common.stat);
    write_string2(msg);
#endif

#ifdef TEST_ANTS    
    // Simple interface test for ANTs (dual dipole antenna module)
    write_string2("-----------------------------------");
    write_string2("Test: ANTs interface (deployment status command)");
    write_string2("-----------------------------------");
    sprintf(msg,"Antenna power on status = 0x%02x",antenna_on_status);
    write_string2(msg);
    unsigned char ants_response[2];      
    
#ifdef ANTS_DEPLOY
    write_string2("-----------------------------------");
    write_string2("ANTS Deployment: Arm");
    write_string2("-----------------------------------");
    ants_arm();
    ants_deploy_status(ants_response);
    sprintf(msg,"ANTs deploy status: byte 1 = 0x%02x", ants_response[0]);
    write_string2(msg);
    sprintf(msg,"ANTs deploy status: byte 2 = 0x%02x", ants_response[1]);
    write_string2(msg);
    
#ifdef ANTS_DEPLOY2    
    write_string2("-----------------------------------");
    write_string2("ANTS Deployment: Deploy all");
    write_string2("-----------------------------------");
    ants_deploy_all();
    ants_deploy_status(ants_response);
    sprintf(msg,"ANTs deploy status: byte 1 = 0x%02x", ants_response[0]);
    write_string2(msg);
    sprintf(msg,"ANTs deploy status: byte 2 = 0x%02x", ants_response[1]);
    write_string2(msg);
    
    wait = 1000 * DELAYMSEC;
    while (ants_response[0] & 0x08)
    {
        TMR1 = 0;
        while (TMR1 < wait);
        ants_deploy_status(ants_response);
        write_string2("---");
        sprintf(msg,"ANTs deploy status: byte 1 = 0x%02x", ants_response[0]);
        write_string2(msg);
        sprintf(msg,"ANTs deploy status: byte 2 = 0x%02x", ants_response[1]);
        write_string2(msg);
    }
    
    write_string2("-----------------------------------");
    write_string2("ANTS Deployment: Get deploy times");
    write_string2("-----------------------------------");
    ants_time_1(ants_response);
    sprintf(msg,"ANTs time 1: byte 1 = 0x%02x", ants_response[0]);
    write_string2(msg);
    sprintf(msg,"ANTs time 1: byte 2 = 0x%02x", ants_response[1]);
    write_string2(msg);
    ants_time_2(ants_response);
    sprintf(msg,"ANTs time 2: byte 1 = 0x%02x", ants_response[0]);
    write_string2(msg);
    sprintf(msg,"ANTs time 2: byte 2 = 0x%02x", ants_response[1]);
    write_string2(msg);
    ants_time_3(ants_response);
    sprintf(msg,"ANTs time 3: byte 1 = 0x%02x", ants_response[0]);
    write_string2(msg);
    sprintf(msg,"ANTs time 3: byte 2 = 0x%02x", ants_response[1]);
    write_string2(msg);
    ants_time_4(ants_response);
    sprintf(msg,"ANTs time 4: byte 1 = 0x%02x", ants_response[0]);
    write_string2(msg);
    sprintf(msg,"ANTs time 4: byte 2 = 0x%02x", ants_response[1]);
    write_string2(msg);
#endif // ANTS_DEPLOY2
    
    write_string2("-----------------------------------");
    write_string2("ANTS Deployment: Disarm");
    write_string2("-----------------------------------");
    ants_disarm();
    ants_deploy_status(ants_response);
    sprintf(msg,"ANTs deploy status: byte 1 = 0x%02x", ants_response[0]);
    write_string2(msg);
    sprintf(msg,"ANTs deploy status: byte 2 = 0x%02x", ants_response[1]);
    write_string2(msg);
    
    // look at the battery status after antenna deployment
    // test EPS and battery telemetry
    write_string2("-----------------------------------");
    write_string2("Test: EPS / Battery Telemetry");
    write_string2("-----------------------------------");
    eps_status = eps_get_status();
    sprintf(msg,"EPS status byte = 0x%02x", eps_status);
    write_string2(msg);
    bat_status = bat_get_status();
    sprintf(msg,"Bat status byte = 0x%02x", bat_status);
    write_string2(msg);
    batv = eps_get_batv();
    sprintf(msg,"Battery voltage = %.2f",batv);
    write_string2(msg);
    bcr1v = eps_get_bcr1v();
    bcr2v = eps_get_bcr2v();
    bcr3v = eps_get_bcr3v();
    bcroutv = eps_get_bcroutv();
    sprintf(msg,"BCR Voltages: %.2f %.2f %.2f %.2f",bcr1v, bcr2v, bcr3v, bcroutv);
    write_string2(msg);
    bcr1ia = eps_get_bcr1ia();
    bcr1ib = eps_get_bcr1ib();
    bcr2ia = eps_get_bcr2ia();
    bcr2ib = eps_get_bcr2ib();
    bcr3ia = eps_get_bcr3ia();
    bcr3ib = eps_get_bcr3ib();
    sprintf(msg,"BCR Currents: %.2f %.2f %.2f %.2f %.2f %.2f",bcr1ia, bcr1ib, bcr2ia, bcr2ib, bcr3ia, bcr3ib);
    write_string2(msg);
    bati = bat_get_bati();
    ischarging = bat_get_batischarging();
    sprintf(msg,"Bat current (is_charge) = %.2f, %d", bati, ischarging);
    write_string2(msg);

#endif // ANTS_DEPLOY
    
#endif // TEST_ANTS

#endif // USB

#ifdef UART2_INTERRUPT
    // Test the use of UART2 receive interrupt to handle incoming data.
    // Eventually this will be used to handle packets received by the radio
    // NB: This test block requires both WRITE_DIAG1 and USE_USB be defined.
    
    // enter the main loop, wait for interrupts
    char acknack_msg[255];
    char goodpacket_msg[255];
    float batv = eps_get_batv();
    // try to get telemetry
    //he100_telemetry(he100_telem);
    sprintf(acknack_msg,"Startup: BatV = %.2f, RSSI = %d",batv,he100_telem[15]);
    //he100_transmit_test_msg2(he100_response, acknack_msg);

    // clear the UART2 receive interrupt flag, and enable the interrupt source
    _U2RXIF = 0;
    _U2RXIE = 1;
    
    
    while (1)
    {
        // check for a valid ack/nack response on UART2
        if (he100_acknack)
        {
            // disable the interrupt source
            _U2RXIE = 0;
            
            sprintf(acknack_msg,"%02x %02x %02x %02x %02x %02x %02x %02x",he100_head[0],he100_head[1],he100_head[2],he100_head[3],he100_head[4],he100_head[5],he100_head[6],he100_head[7]);
            
            // transmit a diagnostic message
            he100_transmit_test_msg2(he100_response, acknack_msg);
            
            // reset the UART2 traps
            nhbytes = 0;
            ndbytes = 0;
            ishead_flag = 0;
            isdata_flag = 0;
            he100_acknack = 0;
            he100_receive = 0;
            // enable the interrupt source
            _U2RXIE = 1;
        }
        // check for valid data receive on UART2
        if (he100_receive)
        {
            // disable the interrupt source
            _U2RXIE = 0;
            
            // transmit a diagnostic message
            batv = eps_get_batv();
            he100_telemetry(he100_telem);
            sprintf(goodpacket_msg,"Valid packet: BatV = %.2f, RSSI = %d",batv,he100_telem[15]);
            he100_transmit_test_msg2(he100_response, goodpacket_msg);
            
            // reset the UART2 traps
            nhbytes = 0;
            ndbytes = 0;
            ishead_flag = 0;
            isdata_flag = 0;
            he100_acknack = 0;
            he100_receive = 0;
            // enable the interrupt source
            _U2RXIE = 1;
        }
    }
#endif
    
    // hold here
    while (1);

}
