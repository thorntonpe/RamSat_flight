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
#include "imtq.h"
#include "ants.h"
#include "he100.h"
#include "security.h"
#include "sd_test.h"
#include "telemetry.h"
#include "command.h"
#include "sgp4.h"
#include "wmm.h"
#include "position_attitude.h"
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <math.h>

// Configuration for ground testing and flight configurations
// Input/Output configuration: Define USB or HE100, but not both
//#define USB      // Enable USB I/O (ground testing only)
//#define RS232      // enable the RS232 serial interface
#define HE100    // Enable He-100 transceiver I/O (ground testing or flight)

//#define INIT_RTC   // Pre-flight code to initialize RTC
//#define INIT_PREDEPLOY // Pre-flight code to set MUST_WAIT flag
//#define TEST_ARDUCAM
//#define TEST_IMTQ
//#define TEST_ANTS
//#define ANTS_DEPLOY  // just the arm/disarm steps
//#define ANTS_DEPLOY2 // include the actual deployment steps
//#define BAT_TELEM  // include outputs for battery telemetry on RS232
#define UART2_INTERRUPT  // test the use of interrupt handler for incoming UART data

// global variable for initialization data
init_data_type init_data; // data structure for peripheral initialization
// global variable for the current TLE
tle_t tle;
// global variables for telemetry control
telem_control_type telem_lev0;   // Level 0 control data
telem_control_type telem_lev1;   // Level 1 control data
telem_control_type telem_lev2;   // Level 2 control data
// global variable for position and attitude data
position_attitude_type posatt;

// Global ISR variables for timed events
// This flag gets set as each minute elapses
volatile int minute_elapsed = 0;
// 32-bit timer (Timer2/3) interrupt handler, for watchdog timer resets
void _ISR _T3Interrupt (void)
{
    // set flag to execute watchdog resets in main loop
    minute_elapsed = 1;
    // reset the interrupt flag before exit
    _T3IF = 0;
}

// Global ISR variables for the He-100 interface on UART2
volatile unsigned char u2rx_char;
volatile int nhbytes = 0; // number of good header bytes already received
volatile int ndbytes = 0; // number of data payload bytes expected
volatile int data_byte;   // current data byte
volatile int ishead_flag = 0;
volatile int isdata_flag = 0;
volatile int he100_receive = 0;
volatile unsigned char he100_head[8]; // command code, payload length, and checksum bytes
volatile unsigned char he100_data[258]; // data payload from He-100
#ifdef UART2_INTERRUPT
// test code for UART2 receive interrupt handler
void _ISR _U2RXInterrupt (void)
{
    int i;
    unsigned char ck_a, ck_b; // checksum bytes
    
    // read a single character from the UART2 receive buffer
    u2rx_char = U2RXREG;
    
    // indicate received char by writing ASCII 1 to UART1 (RS-232)
    //while (U1STAbits.UTXBF);      // wait if the transmit buffer is full)
    //U1TXREG = 0x31;
    
    // if a header has not been found, keep checking
    if (!ishead_flag)
    {
        // trap the start of an He-100 output on UART2
        // All radio UART2 messages start with "He"
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
        // trap the third byte of He-100 output. 0x20 indicates a radio response.
        else if (nhbytes == 2 && u2rx_char == 0x20)
        {
            he100_head[nhbytes++] = u2rx_char;
        }
        // trap the fourth byte of He-100 output. If this is 0x04 it indicates a 
        // received packet message, so set flags to gather remaining header data
        else if (nhbytes == 3 && u2rx_char == 0x04)
        {
            he100_head[nhbytes++] = u2rx_char;
            ishead_flag = 1;
            isdata_flag = 0;
        }
        // any other condition means not a valid receive packet header, so reset
        else
        {
            nhbytes = 0;
            ndbytes = 0;
            ishead_flag = 0;
            isdata_flag = 0;
            he100_receive = 0;
        }
    }
    // There is the start of a valid header, but not yet in the data segment
    else if (!isdata_flag)
    {
        // gather remaining header data
        // trap the payload size and checksum bytes
        if (nhbytes < 8)
        {
            he100_head[nhbytes++] = u2rx_char;
        }
        // reached the end of the header, so compute checksum for bytes 2 through 5
        if (nhbytes == 8)
        {
            ck_a = 0;
            ck_b = 0;
            for (i=2 ; i<6 ; i++)
            {
                ck_a += he100_head[i];
                ck_b += ck_a;
            }
            // check checksum against bytes 6 and 7
            if (ck_a == he100_head[6] && ck_b == he100_head[7])
            {
                // checksum is good: if this is a packet receive, get length
                // from header bytes 4 and 5, set flag to start retrieving payload.
                // Adding two additional bytes for the checksum after payload.
                ndbytes = (he100_head[4]<<8 | he100_head[5]) + 2;
                data_byte = 0;
                isdata_flag = 1;
                // as a diagnostic to indicate a good header and the start of a
                // data payload, write an ASCII 2 to UART1.
                //while (U1STAbits.UTXBF);
                //U1TXREG = 0x32;
            }
            // if checksum is bad, reject and reset traps
            else
            {
                nhbytes = 0;
                ndbytes = 0;
                ishead_flag = 0;
                isdata_flag = 0;
                he100_receive = 0;
                // as a diagnostic to indicate a bad header checksum
                // on received data, write an ASCII 3 to UART1.
                //while (U1STAbits.UTXBF);
                //U1TXREG = 0x33;
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
        // this is the last byte, so test the checksum
        if (data_byte == ndbytes)
        {
            ck_a = 0;
            ck_b = 0;
            // Skip the He on header, but include the other 6 header bytes and
            // the payload
            for (i=2 ; i<8 ; i++)
            {
                ck_a += he100_head[i];
                ck_b += ck_a;
            }
            for (i=0 ; i<(ndbytes-2) ; i++)
            {
                ck_a += he100_data[i];
                ck_b += ck_a;
            }
            if (ck_a == he100_data[ndbytes-2] && ck_b == he100_data[ndbytes-1])
            {
                // set flag that a received data packet is ready
                he100_receive = 1;
                // as a diagnostic to indicate a good data checksum
                // on received data, write an ASCII 4 to UART1.
                //while (U1STAbits.UTXBF);
                //U1TXREG = 0x34;
            }
            else
            {
                // bad checksum for received data, reject and reset traps 
                nhbytes = 0;
                ndbytes = 0;
                ishead_flag = 0;
                isdata_flag = 0;
                he100_receive = 0;
                // as a diagnostic to indicate a bad data checksum
                // on received data, write an ASCII 5 to UART1.
                //while (U1STAbits.UTXBF);
                //U1TXREG = 0x35;
            }
        }
    }
    // reset the interrupt flag before exit
    _U2RXIF = 0;
}
#endif // UART_INTERRUPT

int main(void) {
    // data structures for iMTQ (magnetorquer)
    imtq_resp_common imtq_common;       // iMTQ response from every command
    imtq_resp_state imtq_state;         // iMTQ state data
    imtq_resp_mtm imtq_calib_mtm;       // iMTQ calibrated magnetometer data
    imtq_resp_mtm imtq_raw_mtm;         // iMTQ raw magnetometer data
    imtq_resp_integ imtq_integ;         // iMTQ MTM integration time parameter

    // flags controlling the main program loop
    int isNewTLE = 0;   // flag is set immediately after a new TLE is uplinked
    int isGoodTLE = 0;  // flag is set when a current TLE is available
    
    // Variables used in orbital prediction
    double jd;          // The current julian date, from RTC
    double t_since;     // Time since epoch of current TLE, in minutes
    double sat_params[N_SAT_PARAMS]; // parameters needed by the SGP4 code
    double pos[3];      // Satellite position vector 
    double lat;         // current latitude (radians)
    double lon;         // current longitude (radians)
    double elev;        // current elevation of orbit (km)
    int sgp4_ret;       // return value for the main SGP4 call
    double f, f2;       // temporary variables for ellipsoid correction
    double cor_lat;     // correct latitude for ellipsoid
    double lst;         // local sidereal time
    double b_locx, b_locy, b_locz;  // magnetic field in local tangential coords
    double Bx, By, Bz;  // Magnetic field in ECI coordinates
    double coslat,sinlat,coslst,sinlst; // for intermediate calculations
    double t1;          // temporary variable
    
    char isodatetime[25];  // for test code reporting of time
    int loop_counter;

    // precalculate ellipsoid flattening parameter
    f = 0.003352811;    // flattening parameter for ellipsoid
    f2 = (1.0-f)*(1.0-f);
    
    // Initialize the PIC24 timers
    // Initialize TIMER1 (16-bit), using system clock and 1:256 prescalar
    // This is a general purpose timer used in multiple routines for device delays
    T1CON = 0x8030;
    
    // Initialize TIMER2/3 (32-bit), using system clock and 1:256 prescalar
    // also set the period register for timer 2/3 for 3,750,000 counts, which is
    // one minute. This timer is used for watchdog resets and other periodic
    // operations, and has its period interrupt enabled.
    T2CON = 0x8038;
    //PR3 = 0x0039;
    //PR2 = 0x3870;
    PR3 = 0x0000;
    PR2 = 0xF424;
    // reset the timer
    TMR3 = 0x0000;
    TMR2 = 0x0000;
    // set a low priority, clear flag, and enable the Timer2/3 interrupt
    _T3IP = 0x01;
    _T3IF = 0;
    _T3IE = 1;
    
    long wait;          // Timer trigger
    int ui;             // user input, for ground testing
    int i;
    char msg[128];      // character string for messages to user via COM port
    
    // Set desired baud rates for UART2 (He-100 radio or USB interfaces)
#ifdef HE100
    init_data.u2br_request = 9600;   // UART2 desired baud rate for radio
    unsigned char he100_response[8];
    char downlink_msg[260]; // Response/message to be downlinked by RamSat
    unsigned char he100_telem[26];
#endif
    
#ifdef USB    
    init_data.u2br_request = 9600; // UART2 desired baud rate for COM port
#endif

#ifdef RS232    
    init_data.u1br_request = 115200; // UART1 desired baud rate
    // Initialize the UART1 peripheral (RS-232)
    // this is done outside the init_peripherals() routine since it isn't used 
    // for flight configuration
    init_data.u1br_actual = init_uart1(FOSC, init_data.u1br_request);
    write_string1("---------------------------------------------");
    write_string1("RamSat flight software: ground testing output (RS232)");
    sprintf(msg,"test %03d",1);
    write_string1(msg);
    write_string1("---------------------------------------------");

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
    // includes resets for the EPS WDT during post-deploy wait period
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
    unsigned char cameras_on_status = 1;
    cameras_on_status = eps_cameras_on();
#endif

#ifdef TEST_ANTS
    // switch on power to the antenna (deploy and I2C)
    unsigned char antenna_on_status;
    unsigned char antenna_off_status;
    //antenna_on_status = eps_antenna_on();
    antenna_off_status = eps_antenna_off();
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
    adc_scan_all();
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
    
    // test EPS and battery telemetry
    int adc;  // raw telemetry output
    write_string2("-----------------------------------");
    write_string2("Test: EPS / Battery Telemetry");
    write_string2("-----------------------------------");
    unsigned char eps_status = eps_get_status();
    sprintf(msg,"EPS status byte = 0x%02x", eps_status);
    write_string2(msg);
    
    unsigned char bat_status = bat_get_status();
    sprintf(msg,"Bat status byte = 0x%02x", bat_status);
    write_string2(msg);
    float batv = eps_get_batv(&adc);
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
    
    // Simple interface test for iMTQ (magnetorquer)
    write_string2("-----------------------------------");
    write_string2("Test: iMTQ interface (no-op command)");
    write_string2("-----------------------------------");
    imtq_no_op(&imtq_common);
    sprintf(msg,"iMTQ response: command ID = 0x%02x", imtq_common.cc);
    write_string2(msg);
    sprintf(msg,"iMTQ response: status byte = 0x%02x", imtq_common.stat);
    write_string2(msg);
    //imtq_start_actpwm(&imtq_common,0,0,500,1000);
    
    // hold here
    while (1);

#endif // end of USB
    
#ifdef TEST_ARDUCAM
    // test camera interfaces
    //write_string1("-----------------------------------");
    //write_string1("Test: Arducam Interfaces and Operation");
    //write_string1("-----------------------------------");
    sprintf(downlink_msg,"Cameras power on status = 0x%02x",cameras_on_status);
    //write_string1(msg);
    he100_transmit_packet(he100_response, downlink_msg);
    // test the arducam SPI interface with write/read
    int arducam_spi_iserror = test_arducam_spi();
    sprintf(downlink_msg,"Arducam SPI: Test is_error = %d", arducam_spi_iserror);
    he100_transmit_packet(he100_response, downlink_msg);
    //write_string1(msg);
    // initialize arduchip and OV2640 sensor for both cameras
    int arducam_init_iserror = init_arducam();
    sprintf(downlink_msg,"Arducam Init: Test is_error = %d", arducam_init_iserror);
    he100_transmit_packet(he100_response, downlink_msg);
    //write_string1(msg);
    
    while(1);
    
    // start the image capture test loop
    int arducam_capture_iserror = test_arducam_capture();
    sprintf(msg,"Arducam Capture: Test is_error = %d", arducam_capture_iserror);
    write_string1(msg);
#endif // TEST_ARDUCAM

#ifdef TEST_ANTS    
    // Simple interface test for ANTs (dual dipole antenna module)
    write_string1("-----------------------------------");
    write_string1("Test: ANTs interface (deployment status command)");
    write_string1("-----------------------------------");
    sprintf(msg,"Antenna power on status = 0x%02x",antenna_off_status);
    write_string1(msg);
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
    
#ifdef UART2_INTERRUPT
    // Test the use of UART2 receive interrupt to handle incoming data packets
    
    // enter the main loop, wait for interrupts
    char uplink_cmd[255];   // holds the latest uplinked command and parameters
    char cmd_idstr[3];      // null-terminated string for the command ID
    cmd_idstr[2]=0;         // null termination
    char cmd_paramstr[257]; // parameters passed in uplink command
    int cmd_id;             // integer value for command ID
    int cmd_err;            // return value for command handlers
    

    // Retrieve battery telemetry for startup message
    int adc;  // raw telemetry output
    float batv = eps_get_batv(&adc);
    
    // Retrieve He-100 telemetry for startup message
    union he100_telem_union telem_union;
    int he100_telem_iserror = he100_telemetry(telem_union.raw);
    
    // Downlink the startup message
    sprintf(downlink_msg,"RamSat: Startup BatV = %.2f, RSSI = %d",batv,telem_union.telem.rssi);
    he100_transmit_packet(he100_response, downlink_msg);
    
    // test the overrun status of UART2 buffer, report and reset
    if (U2STAbits.OERR)
    {
        sprintf(downlink_msg,"RamSat: Startup UART2 buffer overflow error!");
        he100_transmit_packet(he100_response, downlink_msg);
        U2STAbits.OERR = 0;
    }

    // initialize the elapsed time counters for multi-level telemetry
    int telem_lev0_elapsed = 0;
    int telem_lev1_elapsed = 0;
    
    // initialize telemetry control data structures 
    // Level 0 telemetry control
    telem_lev0.record_period = 1;     // 1-minute intervals per record
    telem_lev0.rec_per_page = 60;     // one page for each hour
    telem_lev0.page_per_block = 24;   // 24 pages (hours) between timestamps
    telem_lev0.first_sector = 1;      // first sector to use for this telemetry level
    telem_lev0.num_sectors = 10;      // number of sectors to use for this telemetry level
    telem_lev0.record_count = 0;      // record counter
    telem_lev0.page_count = 0;        // page counter (includes timestamp pages)
    telem_lev0.first_timestamp[0]=0;  // initialize timestamps as null
    telem_lev0.last_timestamp[0]=0;   // initialize timestamps as null
    
    // Level 1 telemetry control
    telem_lev1.record_period = 1;     // 1-minute intervals per record
    telem_lev1.rec_per_page = 60;     // one page for each hour
    telem_lev1.page_per_block = 24;   // 24 pages (hours) between timestamps
    telem_lev1.first_sector = 11;      // first sector to use for this telemetry level
    telem_lev1.num_sectors = 10;      // number of sectors to use for this telemetry level
    telem_lev1.record_count = 0;      // record counter
    telem_lev1.page_count = 0;        // page counter (includes timestamp pages)
    telem_lev1.first_timestamp[0]=0;  // initialize timestamps as null
    telem_lev1.last_timestamp[0]=0;   // initialize timestamps as null

    // Level 2 telemetry control
    telem_lev2.record_period = 1;     // 1-minute intervals per record
    telem_lev2.rec_per_page = 60;     // one page for each hour
    telem_lev2.page_per_block = 24;   // 24 pages (hours) between timestamps
    telem_lev2.first_sector = 1;      // first sector to use for this telemetry level
    telem_lev2.num_sectors = 10;      // number of sectors to use for this telemetry level
    telem_lev2.record_count = 0;      // record counter
    telem_lev2.page_count = 0;        // page counter (includes timestamp pages)
    telem_lev2.first_timestamp[0]=0;  // initialize timestamps as null
    telem_lev2.last_timestamp[0]=0;   // initialize timestamps as null
    
    // set a high interrupt priority for uplink, clear the UART2 receive
    // interrupt flag, and enable the interrupt source
    _U2RXIP = 0x07;
    _U2RXIF = 0;
    _U2RXIE = 1;
        
    // Some initialization for iMTQ
    // set the mtm time integration parameter
    // (new value is 6, which corresponds to 80 ms)
    imtq_set_mtm_integ(&imtq_common, &imtq_integ, 6);
    
    // set watchdog timer (currently 32 minutes)
    eps_set_watchdog();
    
    // Enter the main program loop
    while (1)
    {
        // If the 1-minute flag is set (from Timer2/3 interrupt)
        // cycle through timed events (watchdog reset, telemetry gathering)
        if (minute_elapsed)
        {
            // Reset watchdog timers on each minute boundary
            eps_reset_watchdog();
            
            // track elapsed time for different levels of telemetry
            telem_lev0_elapsed++;
            telem_lev1_elapsed++;
            
            // check if any telemetry levels are triggered
            if (telem_lev0_elapsed == telem_lev0.record_period)
            {
                // perform level-0 telemetry operations
                telem_gather_lev0(&telem_lev0);
                // reset elapsed counter
                telem_lev0_elapsed = 0;
            }
            if (telem_lev1_elapsed == telem_lev1.record_period)
            {
                // perform level-0 telemetry operations
                telem_gather_lev1(&telem_lev1);
                // reset elapsed counter
                telem_lev1_elapsed = 0;
            }
            
            // reset the global 1-minute flag
            minute_elapsed = 0;
        }
        
        // test the overrun status of UART2 buffer, report and reset
        if (U2STAbits.OERR)
        {
            sprintf(downlink_msg,"RamSat: Main loop UART2 buffer overflow error!");
            he100_transmit_packet(he100_response, downlink_msg);
            U2STAbits.OERR = 0;
        }

        // check for valid data receive on UART2
        if (he100_receive)
        {
            // disable the UART2 interrupt source
            // this prevents new uplink from interrupting the command handler
            _U2RXIE = 0;
            
            // disable the Timer 2/3 interrupt source
            // this prevents interference with downlink
            _T3IE = 0;

            // parse the packet to look for a command
            // discard extra bytes at beginning and end of packet
            // calculate the length of entire uplink command, and the parameter part
            int uplink_nbytes = ndbytes - (HEAD_NBYTES+TAIL_NBYTES);
            // make sure the uplinked data is at least long enough for 
            // security key and command ID before proceeding
            if (uplink_nbytes > NKEY+1)
            {
                sprintf(downlink_msg,"RamSat: Packet received, entering command interpreter.");
                he100_transmit_packet(he100_response, downlink_msg);
                // copy the good part of uplink packet into local array
                memcpy(uplink_cmd,&he100_data[HEAD_NBYTES],uplink_nbytes);
                // check for the uplink command security key
                if (!memcmp(uplink_cmd,seckey,NKEY))
                {
                    // good security key, continue processing as a valid command
                    // strip the command ID and any parameters out of the 
                    // uplinked packet as separate pieces
                    memcpy(cmd_idstr,uplink_cmd+NKEY,2);
                    // calculate the length of the parameter part of command
                    int param_nbytes = uplink_nbytes-(NKEY+2);
                    memcpy(cmd_paramstr,uplink_cmd+NKEY+2,param_nbytes);
                    // make cmd_paramstr a null-terminated string
                    cmd_paramstr[param_nbytes]=0;
                    // make the command ID a number for case statement
                    cmd_id = atoi(cmd_idstr);

                    // the main switch-case statement that processes commands
                    switch(cmd_id)
                    {
                        case 0: // No-Op command - just send acknowledgment message 
                            CmdNoOp();
                            break;
                        
                        case 1: // return the number of files on SD card
                            cmd_err = CmdFileCount();
                            break;
                        
                        case 2: // list details for each file on SD card
                            cmd_err = CmdFileList();
                            break;
                        
                        case 3: // dump the contents of a named file
                            cmd_err = CmdFileDump(cmd_paramstr);
                            break;
                        
                        case 4: // uplink a new Two-Line Element (TLE)
                            cmd_err = CmdNewTLE(cmd_paramstr,param_nbytes, &isNewTLE);
                            break;
                        
                        case 5: // return the current date and time from RTC
                            cmd_err = CmdGetDateTime();
                            break;
                        
                        case 6: // Set the date and time on RTC
                            cmd_err = CmdSetDateTime(cmd_paramstr, param_nbytes);
                            break;
                        
                        case 7: // Erase one 64KB sector on the SFM
                            cmd_err = CmdEraseSector(cmd_paramstr);
                            break;
                        
                        case 8: // Write one page within one sector on the SFM
                            cmd_err = CmdWritePage(cmd_paramstr);
                            break;
                        
                        case 9: // Read one page from SFM and downlink
                            cmd_err = CmdDownlinkPage(cmd_paramstr);
                            break;
                            
                        case 10: // Read telemetry control data for a specified level
                            cmd_err = CmdGetTelemControl(cmd_paramstr);
                            break;
                        
                        case 11: // Read telemetry data for sector and page range
                            cmd_err = CmdGetTelemData(cmd_paramstr);
                            break;
                        
                        case 12: // Capture image on both cameras, store to SD card
                            cmd_err = CmdCaptureImage(cmd_paramstr);
                            break;

                        case 13: // Turn camera power on (1) or off (0)
                            cmd_err = CmdCameraPower(cmd_paramstr);
                            break;
                        
                        case 14: // dump single packet from named file
                            cmd_err = CmdFileDumpOnePacket(cmd_paramstr);
                            break;
                        
                        case 15: // Return current telemetry for one system 
                            cmd_err = CmdCurrentTelemetry(cmd_paramstr);
                            break;
                        
                        case 90: // Set post-deployment timer flag (pre-flight)
                            CmdSetPDT();
                            break;
                        
                        case 98: // Reset the flight radio
                            CmdResetHe100();
                            break;

                        case 99: // Reset the flight computer
                            CmdResetPIC();
                            break;

                        default:
                            sprintf(msg,"Received an invalid command ID");
                            //write_string1(msg);
                            sprintf(downlink_msg,"RamSat: %d is an invalid command ID.", cmd_id);
                            he100_transmit_packet(he100_response, downlink_msg);
                    }
                } // good seckey
                else
                {
                    // bad or missing security key
                    sprintf(msg,"Invalid security key in received packet");
                    //write_string1(msg);
                    sprintf(downlink_msg,"RamSat: Invalid security key.");
                    he100_transmit_packet(he100_response, downlink_msg);
                }
            } // meets minimum packet length for valid command
            else
            {
                sprintf(msg,"Packet too short");
                //write_string1(msg);
                sprintf(downlink_msg,"RamSat: ERROR - received packet too short.");
                he100_transmit_packet(he100_response, downlink_msg);
            }
            
            // reset the UART2 receive traps
            nhbytes = 0;
            ndbytes = 0;
            ishead_flag = 0;
            isdata_flag = 0;
            he100_receive = 0;

            // re-enable the UART2 receive and Timer 2/3 interrupts
            _U2RXIE = 1;
            _T3IE = 1;
        }
        
        // After any outstanding uplink commands have been processed,
        // enter the main work segment of the program loop
        
        // If the TLE was just uplinked, initialize the SGP4 parameters
        if (isNewTLE)
        {
            SGP4_init(sat_params, &tle);
            // clear the new TLE flag, set the good TLE flag
            isNewTLE = 0;
            isGoodTLE = 1;
            // test code to get timing for the main loop
            // on the first pass with a new TLE, write out the isodatetime
            get_isodatetime(isodatetime);
            _U2RXIE = 0;
            sprintf(downlink_msg,"RamSat: New TLE, ISO datetime: %s",isodatetime);
            he100_transmit_packet(he100_response, downlink_msg);
            _U2RXIE = 1;
            loop_counter = 0;
        }
        
        // If there is a good TLE, use it and RTC data to make an orbital prediction
        if (isGoodTLE)
        {
            // Read the RTC and format as a julian date.
            get_juliandate(&jd);
            // Calculate the time since epoch of current TLE, in minutes.
            t_since = (jd - tle.epoch) * 1440.;
            // call the SGP4 routine to calculate position (not calculating velocity)
            sgp4_ret = SGP4(t_since, &tle, sat_params, pos, NULL);
            // if no error in SGP4 call, continue with orbital prediction
            if (!sgp4_ret)
            {
                // estimate satellite's groundtrack longitude, latitude, and orbit elevation
                // lat and lon are in radians, elev is meters above mean radius
                // lst (local sidereal time) is in [units]
                sat_lon_lat_elev(jd, pos, &lon, &lat, &elev, &lst);
                // latitude corrected for ellipsoid (also in radians))
                cor_lat = atan(tan(lat)/f2);
                
                // estimate Earth's magnetic field vector at this location
                // returns values in local tangential coordinates
                // Note: latitude is geocentric, not geodetic
                // Note: elevation is distance from center of Earth
                calc_WMM(2020.8, lon, lat, elev+6378.135, &b_locx, &b_locy, &b_locz);
                
                // convert magnetic field local tangential coordinates to ECI coordinates
                coslat = cos(lat);
                sinlat = sin(lat);
                coslst = cos(lst);
                sinlst = sin(lst);
                t1 = b_locz*coslat + b_locx*sinlat;
                Bx = t1*coslst - b_locy*sinlst;
                By = t1*sinlst + b_locy*coslst;
                Bz = b_locz*sinlat + b_locx*coslat;
                
                // gather MTM data in satellite frame coordinates
                // start the MTM measurement
                imtq_start_mtm(&imtq_common);
                // delay for MTM integration
                TMR1 = 0;
                while (TMR1 <= 82 * TMR1MSEC);
                // get the calibrated MTM data
                imtq_get_calib_mtm(&imtq_common, &imtq_calib_mtm);
                
                // add the sun angle calculations here...
                
                // once all orbital and attitude calculations are complete
                // pause the interrupt handler and copy the results to a data structure
                // all angles are converted from radians to degrees
                _U2RXIE = 0;
                posatt.jd = jd;
                posatt.t_since = t_since;
                posatt.x_eci = pos[0];
                posatt.y_eci = pos[1];
                posatt.z_eci = pos[2];
                posatt.lon = lon * (180.0/pi);
                posatt.lat = lat * (180.0/pi);
                posatt.cor_lat = cor_lat * (180.0/pi);
                posatt.lst = lst * (180.0/pi);
                posatt.B_locx = b_locx;
                posatt.B_locy = b_locy;
                posatt.B_locz = b_locz;
                posatt.B_x = Bx;
                posatt.B_y = By;
                posatt.B_z = Bz;
                posatt.B_fx = imtq_calib_mtm.x;
                posatt.B_fy = imtq_calib_mtm.y;
                posatt.B_fz = imtq_calib_mtm.z;
                // restart the interrupt handler
                _U2RXIE = 1;
                
                // test code for timing of main loop
                loop_counter++;
                if (loop_counter = 1000)
                {
                    get_isodatetime(isodatetime);
                    _U2RXIE = 0;
                    sprintf(downlink_msg,"RamSat: completed 1000 cycles, ISO datetime: %s",isodatetime);
                    he100_transmit_packet(he100_response, downlink_msg);
                    _U2RXIE = 1;
                    // reset the loop counter, to get timing reports every 1000 cycles
                    loop_counter = 0;
                }
            }   // end of no error on sgp4
            else
            {
                // will need some error handling here for SGP4 error
            }
        }   // end of isGoodTLE
    }       // end of main program loop
#endif      // use UART2 interrupt
    
}
