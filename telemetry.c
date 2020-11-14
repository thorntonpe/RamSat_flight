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
#include <stdio.h>
#include <string.h>

void telem_get0(int count, char *out_str)
{
    char isodatetime[128];
    float scaled_value;
    int raw_value;
    
    // get the ISO-formatted date+time
    get_isodatetime(isodatetime);
    // get EPS telemetry: Battery voltage
    scaled_value = eps_get_batv(&raw_value);
    sprintf(out_str,"%s,%04.2f,%04d",isodatetime,scaled_value,raw_value);
}
