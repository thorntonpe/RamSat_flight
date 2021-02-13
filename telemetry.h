/* 
 * File:       .h
 * Author:     Peter Thornton
 * Purpose:  
 * Created on:  
 */

// telemetry control data structure
typedef struct
{
    int record_period;    // number of minutes between each telemetry record
    int rec_per_page;     // number of records per page on SFM
    int page_per_block;   // number of pages between each timestamp
    int first_sector;     // first sector on SFM to store this telemetry
    int num_sectors;      // how many sectors to use for this telemetry before wrapping
    int record_count;     // the current number of records written
    int page_count;       // the current number of pages written
    char first_timestamp[30]; // first timestamp written for this telemetry
    char last_timestamp[30];  // latest timestamp written for this telemetry
    char pagedata[256]; // the current page of telemetry data 
} telem_control_type;

// function prototypes 
void telem_form_beacon(char *beacon_str);
void telem_gather_lev0(telem_control_type* c);
void telem_gather_lev1(telem_control_type* c);





