/*
 * File:       datetime.c
 * Author:     Peter Thornton
 * Purpose:    Functions to generate formatted date and time from RTC
 * Created on: 19 May 2020
*/

#include "xc.h"
#include "rtc.h"
#include "datetime.h"
#include <stdio.h>

// RTC is assumed to be M41T81S, or equivalent
// get_* assumes the RTC stop and halt bits have been checked and cleared
// Time on RTC is assumed to be in UTC (Z)

// Read RTC and give date in ISO 8601 extended format
int get_isodate(char *isodate)
{
    int err = 0;
    int firstreg = 0x05;
    int nregs = 3;
    unsigned char rtc_data[3];
    int year, mon, day;
    
    // read the day, month, and year from RTC //
    err = rtc_read_data(firstreg, nregs, rtc_data);
    if (!err)
    {
        // format the ISO 8601 date
        year = 2000 + (10 * (rtc_data[2]>>4 & 0xf)) + (rtc_data[2] & 0x0f);
        mon  = (10 * (rtc_data[1]>>4 & 0x1)) + (rtc_data[1] & 0x0f);
        day  = (10 * (rtc_data[0]>>4 & 0x3)) + (rtc_data[0] & 0x0f);
        sprintf(isodate,"%4d-%02d-%02d",year,mon,day);
    }
    else
    {
        // on error, return an obviously bogus date string
        sprintf(isodate,"xxxx-xx-xx");
    }
    return err;
}

// Read RTC and give time in ISO 8601 extended format
int get_isotime(char *isotime)
{
    int err = 0;
    int firstreg = 0x00;
    int nregs = 4;
    unsigned char rtc_data[4];
    int hour, min, sec, hsec;
    
    // read the hour, minutes, seconds, and fractional seconds from RTC //
    err = rtc_read_data(firstreg, nregs, rtc_data);
    if (!err)
    {
        // format the ISO 8601 time
        hour = (10 * (rtc_data[3]>>4 & 0x3)) + (rtc_data[3] & 0x0f);
        min  = (10 * (rtc_data[2]>>4 & 0x7)) + (rtc_data[2] & 0x0f);
        sec =  (10 * (rtc_data[1]>>4 & 0x7)) + (rtc_data[1] & 0x0f);
        hsec = (10 * (rtc_data[0]>>4 & 0xf)) + (rtc_data[0] & 0x0f);
        sprintf(isotime,"%02d:%02d:%02d.%02dZ",hour,min,sec,hsec);
    }
    else
    {
        // on error, return an obviously bogus time string
        sprintf(isotime,"xx:xx:xx.xxZ");
    }
    return err;
}

// Read RTC and give datetime in ISO 8601 extended format
int get_isodatetime(char *isodatetime)
{
    int err = 0;
    int firstreg = 0x00;
    int nregs = 8;
    unsigned char rtc_data[8];
    int year, mon, day, hour, min, sec, hsec;
    
    // read the time and date registers from RTC //
    err = rtc_read_data(firstreg, nregs, rtc_data);
    if (!err)
    {
        // format the ISO 8601 date
        year = 2000 + (10 * (rtc_data[7]>>4 & 0xf)) + (rtc_data[7] & 0x0f);
        mon =  (10 * (rtc_data[6]>>4 & 0x1)) + (rtc_data[6] & 0x0f);
        day =  (10 * (rtc_data[5]>>4 & 0x3)) + (rtc_data[5] & 0x0f);
        hour = (10 * (rtc_data[3]>>4 & 0x3)) + (rtc_data[3] & 0x0f);
        min  = (10 * (rtc_data[2]>>4 & 0x7)) + (rtc_data[2] & 0x0f);
        sec =  (10 * (rtc_data[1]>>4 & 0x7)) + (rtc_data[1] & 0x0f);
        hsec = (10 * (rtc_data[0]>>4 & 0xf)) + (rtc_data[0] & 0x0f);
        sprintf(isodatetime,"%4d-%02d-%02dT%02d:%02d:%02d.%02dZ",
                year,mon,day,hour,min,sec,hsec);
    }
    else
    {
        // on error, return an obviously bogus date string
        sprintf(isodatetime,"xxxx-xx-xxTxx:xx:xx.xxZ");
    }
    return err;
}

// Read RTC and give date in FAT 16-bit format
int get_fatdatetime(unsigned short *date, unsigned short *time)
{
    int err = 0;
    int firstreg = 0x00;
    int nregs = 8;
    unsigned char rtc_data[8];
    int year, mon, day, hour, min, sec, sec2;
    
    // read the day, month, and year from RTC //
    err = rtc_read_data(firstreg, nregs, rtc_data);
    if (!err)
    {
        // format the ISO 8601 date
        year = (2000-1980) + (10 * (rtc_data[7]>>4 & 0xf)) + (rtc_data[7] & 0x0f);
        mon  = (10 * (rtc_data[6]>>4 & 0x1)) + (rtc_data[6] & 0x0f);
        day  = (10 * (rtc_data[5]>>4 & 0x3)) + (rtc_data[5] & 0x0f);
        hour = (10 * (rtc_data[3]>>4 & 0x3)) + (rtc_data[3] & 0x0f);
        min  = (10 * (rtc_data[2]>>4 & 0x7)) + (rtc_data[2] & 0x0f);
        sec =  (10 * (rtc_data[1]>>4 & 0x7)) + (rtc_data[1] & 0x0f);
        sec2 = sec/2;
        *date = (year<<9)|(mon<<5)|(day);
        *time = (hour<<11)|(min<<5)|(sec2);
    }
    else
    {
        // on error, return an obviously bogus date string
        *date = 0;
        *time = 0;
    }
    return err;
    
}

