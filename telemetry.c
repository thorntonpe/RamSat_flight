/*
 * File:       telemetry.c
 * Author:     Peter Thornton
 * Purpose:    Functions to gather, store, and retrieve telemetry data
 * Created on: 7 November 2020
 *  
*/

#include "xc.h"
#include "datetime.h"
#include "eps_bat.h"
#include "sfm.h"
#include "telemetry.h"
#include <stdio.h>
#include <string.h>

void telem_gather_level0(int* telem_count, int* page_count, char *telem_str)
{
    // as a first test, telem0 is hardwired for batv every minute, with 
    // 60 minutes of telemetry on each page, and 24 pages (hours) between
    // timestamp pages.
    int sector = 1;
    char isodatetime[128];
    char new_str[16];
    int len;
    float scaled_value;
    int raw_value;
    int minute_count, hour_count, day_count;
    
    // calculate minute, hour, and day count
    minute_count = *telem_count % 60;
    hour_count = (*telem_count/60) % 24;
    day_count = *telem_count/1440;
    // on count = 0, write a time stamp page
    if (minute_count == 0)
    {
        // if this is the start of a page period, initialize the page string
        sprintf(telem_str,"%02d,",hour_count);
        
        // if this is also the start of a new 24-hour period, write a timestamp page
        if (hour_count == 0)
        {
            // get the ISO-formatted date+time
            get_isodatetime(isodatetime);
            len = strlen(isodatetime);
            // write the timestamp page
            sfm_write_page(sector, *page_count, isodatetime, len);
            // increment the page count
            *page_count = *page_count+1;
        }
    }
    
    // gather telemetry, format, and concatenate to page string
    scaled_value = eps_get_batv(&raw_value);
    sprintf(new_str,"%03.0lf",scaled_value * 100.0);
    strcat(telem_str,new_str);
    // increment the telemetry counter
    *telem_count = *telem_count+1;
    
    // if this is the end of the hour, write page and increment page_count
    if (minute_count == 59)
    {
        len = strlen(telem_str);
        sfm_write_page(sector, *page_count, telem_str, len);
        page_count++;
    }
}
