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
#include "datetime.h"
#include "eps_bat.h"
#include "imtq.h"
#include "ants.h"
#include "he100.h"
#include "security.h"
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
#define HE100    // Enable He-100 transceiver I/O (ground testing or flight)

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
void __attribute((interrupt,no_auto_psv)) _T3Interrupt (void)
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

// UART2 receive interrupt handler
void __attribute((interrupt,shadow,no_auto_psv)) _U2RXInterrupt (void)
{
    int i;
    unsigned char ck_a, ck_b; // checksum bytes
    
    // read a single character from the UART2 receive buffer
    u2rx_char = U2RXREG;
    
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
            }
            // if checksum is bad, reject and reset traps
            else
            {
                nhbytes = 0;
                ndbytes = 0;
                ishead_flag = 0;
                isdata_flag = 0;
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
            }
            else
            {
                // bad checksum for received data, reject and reset traps 
                nhbytes = 0;
                ndbytes = 0;
                ishead_flag = 0;
                isdata_flag = 0;
                he100_receive = 0;
            }
        }
    }
    // reset the interrupt flag before exit
    _U2RXIF = 0;
}

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
    PR3 = 0x0039;
    PR2 = 0x3870;
    // reset the timer
    TMR3 = 0x0000;
    TMR2 = 0x0000;
    // set a low priority, clear flag, and enable the Timer2/3 interrupt
    _T3IP = 0x01;
    _T3IF = 0;
    _T3IE = 1;
    
    // Set desired baud rates for UART2 (He-100 radio or USB interfaces)
#ifdef HE100
    init_data.u2br_request = 9600;   // UART2 desired baud rate for radio
    unsigned char he100_response[8];
    char downlink_msg[260]; // Response/message to be downlinked by RamSat
#endif
    
#ifdef USB    
    init_data.u2br_request = 9600; // UART2 desired baud rate for COM port
#endif

    // set desired clock speeds for SPI peripherals
    init_data.spi1_fsck = 250000;  // SD card, initial speed (250kHz) 
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
    
    // Initialize the EPS watchdog timer - the system-level watchdog
    unsigned char watchdog_minutes = 2;
    eps_set_watchdog(watchdog_minutes);

    // perform the initial deployment test, and wait if the MUST_WAIT flag is set
    // includes resets for the EPS WDT during post-deploy wait period
    init_wait(&init_data);
    
    // if this is the initial startup after deployment, release antenna
    // pdt_status of 1 indicates that a deploy wait was completed during init_wait()
    if (init_data.pdt_status == 1)
    {
        unsigned char status_byte;
        // turn on 3.3V power to the antenna via switched PDM #8 on EPS
        status_byte = eps_antenna_on();
        // check for error condition from power-on command
        if (status_byte == 0xff)
        {
            init_data.eps_antenna_on_iserror = eps_get_last_error();
        }
        else
        {
            init_data.eps_antenna_on_iserror = 0;
        }
        // wait one second to allow antenna power to stabilize
        TMR1=0;
        while (TMR1 < 1000*TMR1MSEC);
        // verify that PDM #8 is on (could shut itself off due to current limit)
        init_data.antenna_on_status = eps_antenna_status();
        // if PDM #8 is off, try the other power circuit (PDM #10)
        if (init_data.antenna_on_status == 0)
        {
            // try the secondary power circuit
        }
        // if the power to antenna is on, proceed with arm and deploy sequence
        if (init_data.antenna_on_status == 1)
        {
            unsigned char ants_response[2]; // holds response from antenna commands
            // get the initial antenna deployment status
            ants_deploy_status(ants_response);
            init_data.ants0_deploy_status_msb = ants_response[0];
            init_data.ants0_deploy_status_lsb = ants_response[1];
            // arm the antenna deployment mechanism
            ants_arm();
            // wait one second after arm
            TMR1=0;
            while (TMR1 < 1000*TMR1MSEC);
            // get the post-armed antenna deployment status
            ants_deploy_status(ants_response);
            init_data.ants1_deploy_status_msb = ants_response[0];
            init_data.ants1_deploy_status_lsb = ants_response[1];
            // attempt to deploy all four antennas
            //ants_deploy_all();
            // enter a status polling loop, checking every second until
            // all antennas are showing status deployed
            ants_deploy_status(ants_response);
            while ((ants_response[0] & 0x88) || (ants_response[1] & 0x88))
            {
                TMR1 = 0;
                while (TMR1 < 1000*TMR1MSEC);
                ants_deploy_status(ants_response);
            }
            // save the final status 
            init_data.ants2_deploy_status_msb = ants_response[0];
            init_data.ants2_deploy_status_lsb = ants_response[1];
            // save the deployment times for each antenna
            ants_time_1(ants_response);
            init_data.ants_deploy_time1_msb = ants_response[0];
            init_data.ants_deploy_time1_lsb = ants_response[1];
            ants_time_2(ants_response);
            init_data.ants_deploy_time2_msb = ants_response[0];
            init_data.ants_deploy_time2_lsb = ants_response[1];
            ants_time_3(ants_response);
            init_data.ants_deploy_time3_msb = ants_response[0];
            init_data.ants_deploy_time3_lsb = ants_response[1];
            ants_time_4(ants_response);
            init_data.ants_deploy_time4_msb = ants_response[0];
            init_data.ants_deploy_time4_lsb = ants_response[1];
            // disarm antenna system
            ants_disarm();
            // wait one second after disarm
            TMR1=0;
            while (TMR1 < 1000*TMR1MSEC);
            // capture the post-disarm deployment status
            ants_deploy_status(ants_response);
            init_data.ants3_deploy_status_msb = ants_response[0];
            init_data.ants3_deploy_status_lsb = ants_response[1];
            
            // turn off 3.3V power to the antenna via switched PDM #8 on EPS
            status_byte = eps_antenna_off();
            // check for error condition from power-on command
            if (status_byte == 0xff)
            {
                init_data.eps_antenna_off_iserror = eps_get_last_error();
            }
            else
            {
                init_data.eps_antenna_off_iserror = 0;
            }
            // wait one second after power off
            TMR1=0;
            while (TMR1 < 1000*TMR1MSEC);
            // verify that PDM #8 is off
            init_data.antenna_off_status = eps_antenna_status();
        }
        // get initial battery voltage for startup telemetry
        init_data.batv = eps_get_batv();
        
        // write the initial data structure to SFM
        
    }

    // Retrieve battery telemetry and downlink startup message
    float batv = eps_get_batv();
    int size_init_data = sizeof(init_data);
    sprintf(downlink_msg,"RamSat: Startup BatV = %.2f",batv);
    he100_transmit_packet(he100_response, downlink_msg);
    
    // wait one second
    TMR1 = 0;
    while (TMR1 < 1000L*TMR1MSEC);
    
    // test the overrun status of UART2 buffer, report and reset
    if (U2STAbits.OERR)
    {
        sprintf(downlink_msg,"RamSat: Startup UART2 buffer overflow error!");
        he100_transmit_packet(he100_response, downlink_msg);
        // wait one second
        TMR1 = 0;
        while (TMR1 < 1000L*TMR1MSEC);
        // clear buffer overflow
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
    
    // Some initialization for iMTQ
    // set the mtm time integration parameter
    // (new value is 6, which corresponds to 80 ms)
    imtq_set_mtm_integ(&imtq_common, &imtq_integ, 6);
    
    // variables used by the radio command and control interface
    char uplink_cmd[255];   // holds the latest uplinked command and parameters
    char cmd_idstr[3];      // null-terminated string for the command ID
    cmd_idstr[2]=0;         // null termination
    char cmd_paramstr[257]; // parameters passed in uplink command
    int cmd_id;             // integer value for command ID
    int cmd_err;            // return value for command handlers
    
    // Initiate the radio command and control interface:
    // set a high interrupt priority for uplink, clear the UART2 receive
    // interrupt flag, and enable the interrupt source. 
    _U2RXIP = 0x07;
    _U2RXIF = 0;
    _U2RXIE = 1;
        
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
        
        // test the overrun status of UART2 buffer
        // if overflow error, reset the interrupt handler, clear overflow, and report
        if (U2STAbits.OERR)
        {
            // disable the UART2 receive interrupt
            _U2RXIE = 0;
            // report overflow
            sprintf(downlink_msg,"RamSat: Main loop UART2 buffer overflow (clearing)");
            he100_transmit_packet(he100_response, downlink_msg);
            // wait one second
            TMR1 = 0;
            while (TMR1 < 1000L*TMR1MSEC);
            // reset the UART2 receive traps
            nhbytes = 0;
            ndbytes = 0;
            ishead_flag = 0;
            isdata_flag = 0;
            he100_receive = 0;
            // clear the overflow error, which also clears the receive buffer
            U2STAbits.OERR = 0;
            // restart the interrupt handler
            _U2RXIE = 1;
        }

        // check for valid data receive on UART2
        if (he100_receive)
        {
            // disable the UART2 interrupt source
            // this prevents new uplink from interrupting the command handler
            _U2RXIE = 0;
            
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
                            
                        case 16: // Delete one named file
                            cmd_err = CmdFileDelete(cmd_paramstr);
                            break;
                            
                        case 17: // Downlink a range of packets from one file
                            cmd_err = CmdFileDumpRange(cmd_paramstr);
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
                            sprintf(downlink_msg,"RamSat: %d is an invalid command ID.", cmd_id);
                            he100_transmit_packet(he100_response, downlink_msg);
                    }
                } // good seckey
                else
                {
                    // bad or missing security key
                    sprintf(downlink_msg,"RamSat: Invalid security key.");
                    he100_transmit_packet(he100_response, downlink_msg);
                }
            } // meets minimum packet length for valid command
            else
            {
                sprintf(downlink_msg,"RamSat: ERROR - received packet too short.");
                he100_transmit_packet(he100_response, downlink_msg);
            }
            
            // reset the UART2 receive traps
            nhbytes = 0;
            ndbytes = 0;
            ishead_flag = 0;
            isdata_flag = 0;
            he100_receive = 0;

            // re-enable the UART2 receive interrupt
            _U2RXIE = 1;
        }
        
        // After any new uplink commands have been processed,
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
                // all angles are converted from radians to degrees
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
                
                // test code for timing of main loop
                //loop_counter++;
                //if (loop_counter == 100)
                //{
                //    get_isodatetime(isodatetime);
                //    _U2RXIE = 0;
                //    sprintf(downlink_msg,"RamSat: completed 1000 cycles, ISO datetime: %s",isodatetime);
                //    he100_transmit_packet(he100_response, downlink_msg);
                //    _U2RXIE = 1;
                //    // reset the loop counter, to get timing reports every 1000 cycles
                //    loop_counter = 0;
                //}
            }   // end of no error on sgp4
            else
            {
                // will need some error handling here for SGP4 error
            }
        }   // end of isGoodTLE
    }       // end of main program loop
}
