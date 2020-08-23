/*
 * File:       preflight.c
 * Author:     Peter Thornton
 * Purpose:    Routines used in preflight tests and prep
 * Created on: 2 August 2020
 *  
*/

#include "xc.h"
#include "uart.h"
#include "rtc.h"
#include "rtc_user.h"
#include "sfm.h"
#include "preflight.h"
#include <stdio.h>

#define STOP_REG 0x01
#define STOP_BIT 7
#define HALT_REG 0x0c
#define HALT_BIT 6
#define OF_REG 0x0f
#define OF_BIT 2

char msg[128];  // output message string

void preflight_init_rtc()
{
    int ui;         // user input at USB
    int i;          // loop control
    int nbytes;     // number of bytes to read from device
    unsigned char firstbyte;     // first byte to read from device
    unsigned char rtc_data[20];  // RTC register data (max length is 20)
    int err = 0;
    
    // Read all 20 bytes of data from the RTC registers and write to USB
    nbytes = 20;
    firstbyte = 0x00;
    write_string2("-----------------------------------");
    write_string2("Pre-flight: Initialize RTC");
    write_string2("-----------------------------------");
    write_string2("Initial contents of RTC registers:");
    rtc_read_nbytes(nbytes, firstbyte, rtc_data);
    for (i=firstbyte ; i<nbytes ; i++)
    {
        sprintf(msg,"byte #0x%02x = 0x%02x",i,rtc_data[i]);
        write_string2(msg);
        
    }
    
    // wait for user input
    // if 'd', display the RTC
    // if 's', set the RTC
    // if 'c', clear the HALT flag (set when going to battery power))
    // if 't', clear the STOP flag (set on first startup)
    // if 'f', clear the OF flag   (set on first startup)
    while(1)
    {
        ui = read_char2();
        sprintf(msg,"%c", ui);
        write_string2(msg);
        
        if (ui == 'd')
        {
            nbytes = 20;
            firstbyte = 0x00;
            write_string2("--------------------------------------");
            write_string2("Displaying RTC:");
            rtc_read_nbytes(nbytes, firstbyte, rtc_data);
            write_string2("--------------------------------------");
            for (i=firstbyte ; i<nbytes ; i++)
            {
                sprintf(msg,"byte #0x%02x = 0x%02x",i,rtc_data[i]);
                write_string2(msg);
            }
        }
        
        if (ui == 's')
        {
            // the STOP, OF, and HALT bits should be clear before setting clock
            write_string2("--------------------------------------");
            write_string2("Setting RTC time (UTC) and date:");
            write_string2("--------------------------------------");
            firstbyte = 0x00;
            nbytes = 8;
            rtc_data[0]=0x00;   // decimal seconds (00-99)
            rtc_data[1]=0x00;   // seconds (0-59)  
            rtc_data[2]=0x10;   // minutes (0-59)
            rtc_data[3]=0x20;   // hour (00-23)
            rtc_data[4]=0x07;   // day of week (1-7)
            rtc_data[5]=0x22;   // day of month (1-31)
            rtc_data[6]=0x08;   // month (1-12)
            rtc_data[7]=0x20;   // year (00-99)
            rtc_write_nbytes(nbytes, firstbyte, rtc_data);
        }
        
        if (ui == 'c')
        {
            write_string2("--------------------------------------");
            write_string2("Clearing RTC HALT flag:");
            write_string2("--------------------------------------");
            //err = rtc_write_bit(HALT_REG, HALT_BIT, 0);
            err = rtc_clearhalt();
            sprintf(msg,"err = 0x%02x",err);
            write_string2(msg);
            
        }
        
        if (ui == 't')
        {
            write_string2("--------------------------------------");
            write_string2("Clearing RTC STOP flag:");
            write_string2("--------------------------------------");
            err = rtc_write_bit(STOP_REG, STOP_BIT, 0);
            
        }
        
        if (ui == 'f')
        {
            write_string2("--------------------------------------");
            write_string2("Clearing RTC OF flag:");
            write_string2("--------------------------------------");
            rtc_write_bit(OF_REG, OF_BIT, 0);
        }
    }
}

void preflight_init_predeploy()
{
    write_string2("-----------------------------------");
    write_string2("Pre-flight: Initialize predeployment flag on SFM");
    write_string2("-----------------------------------");
    clear_pdt_flag();
    write_string2("Predeployment flag set to MUST_WAIT");
    while(1);
}

