/*
 * File:       telemetry.c
 * Author:     Peter Thornton
 * Purpose:    Functions to gather, store, and retrieve telemetry data
 * Created on: 7 November 2020
 *  
*/

#include "xc.h"
#include "clock.h"
#include "datetime.h"
#include "eps_bat.h"
#include "sfm.h"
#include "telemetry.h"
#include "adc.h"
#include "imtq.h"
#include "ants.h"
#include "position_attitude.h"
#include <stdio.h>
#include <string.h>

#define PAGES_PER_SECTOR 256    // number of 256-byte pages per 64KB sector on SFM

// declare external global variable for position and attitude data, defined in Ramsat_flight_main.c
extern position_attitude_type posatt;

// external variable for current beacon message, defined in Ramsat_flight_main.c
extern char beacon_msg[260];

void telem_form_beacon(char *beacon_str)
{
    // variables to store telemetry data
    char isodatetime[30];
    float batv, bati;
    int bat_ischarging;
    float eps_bcr1v, eps_bcr1ai, eps_bcr1bi;
    float eps_bcr2v, eps_bcr2ai, eps_bcr2bi;
    float eps_bcr3v, eps_bcr3bi;
    float eps_bcroutv, eps_bcrouti;
    float eps_batbusv, eps_batbusi;
    float eps_33busv, eps_33busi;
    float eps_5busv, eps_5busi;
    float eps_mbt, bat_mbt, bat_dbt;
    float eps_sapxt, eps_sanxt, eps_sapyt, eps_sanyt;
    int ss_px1, ss_px2, ss_nx1, ss_nx2, ss_py1, ss_py2, ss_ny1, ss_ny2;
    imtq_resp_common imtq_common;       // iMTQ response from every command
    imtq_resp_mtm imtq_calib_mtm;       // iMTQ calibrated magnetometer data
    unsigned char ants_response[2];    // holds response from antenna commands
    
    // gather all beacon telemetry elements
    get_isodatetime(isodatetime);
    batv = bat_get_batv();
    bati = bat_get_bati();
    bat_ischarging = bat_get_batischarging();
    eps_bcr1v = eps_get_bcr1v();
    eps_bcr1ai = eps_get_bcr1ia();
    eps_bcr1bi = eps_get_bcr1ib();
    eps_bcr2v = eps_get_bcr2v();
    eps_bcr2ai = eps_get_bcr2ia();
    eps_bcr2bi = eps_get_bcr2ib();
    eps_bcr3v = eps_get_bcr3v();
    eps_bcr3bi = eps_get_bcr3ib();
    eps_bcroutv = eps_get_bcroutv();
    eps_bcrouti = eps_get_bcrouti();
    eps_batbusv = eps_get_batv();
    eps_batbusi = eps_get_bati();
    eps_33busv = eps_get_bus33v();
    eps_33busi = eps_get_bus33i();
    eps_5busv = eps_get_bus5v();
    eps_5busi = eps_get_bus5i();
    eps_mbt = eps_get_mbt();
    bat_mbt = bat_get_mbt();
    bat_dbt = bat_get_dbt();
    eps_sanxt = eps_get_sa1at();
    eps_sapxt = eps_get_sa1bt();
    eps_sanyt = eps_get_sa2at();
    eps_sapyt = eps_get_sa2bt();
    // fill ADC buffer with sun sensor data
    adc_scan_all();
    ss_px1 = ADC1BUF4;
    ss_px2 = ADC1BUF0;
    ss_nx1 = ADC1BUF6;
    ss_nx2 = ADC1BUF2;
    ss_py1 = ADC1BUF3;
    ss_py2 = ADC1BUF7;
    ss_ny1 = ADC1BUF5;
    ss_ny2 = ADC1BUF1;
    // start the iMTQ magnetometer measurement
    imtq_start_mtm(&imtq_common);
    // delay for MTM integration
    TMR1 = 0;
    while (TMR1 <= 82 * TMR1MSEC);
    // get the calibrated MTM data
    imtq_get_calib_mtm(&imtq_common, &imtq_calib_mtm);
    // power on the antenna
    eps_antenna_on();
    // a delay to let the antenna circuitry initialize
    TMR1 = 0;
    while (TMR1 < 100*TMR1MSEC);
    // read antenna deployment status
    ants_deploy_status(ants_response);
    // power off the antenna
    eps_antenna_off();
    
    // format message string
    // see google drive speadsheet for beacon message details
    sprintf(beacon_str,"RSBeac:,%s,%3.0f,%4.0f,%1d,%4.0f,%4.0f,%4.0f,%4.0f,%4.0f,%4.0f,%4.0f,%4.0f,%4.0f,%4.0f,%3.0f,%4.0f,%3.0f,%4.0f,%3.0f,%4.0f,%4.0f,%4.0f,%4.0f,%4.0f,%4.0f,%4.0f,%4.0f,%4d,%4d,%4d,%4d,%4d,%4d,%4d,%4d,%6.0ld,%6.0ld,%6.0ld,%02x%02x,%6.0lf,%6.0lf,%4.0lf",
            isodatetime, batv*100.0, bati, bat_ischarging, eps_bcr1v*100.0, eps_bcr1ai, eps_bcr1bi,
            eps_bcr2v*100.0, eps_bcr2ai, eps_bcr2bi, eps_bcr3v*100.0, eps_bcr3bi, eps_bcroutv*100.0, eps_bcrouti,
            eps_batbusv*100.0, eps_batbusi, eps_33busv*100.0, eps_33busi, eps_5busv*100.0, eps_5busi,
            eps_mbt*10.0, bat_mbt*10.0, bat_dbt*10.0, 
            eps_sapxt*10.0, eps_sanxt*10.0, eps_sapyt*10.0, eps_sanyt*10.0,
            ss_px1, ss_px2, ss_nx1, ss_nx2, ss_py1, ss_py2, ss_ny1, ss_ny2,
            imtq_calib_mtm.x, imtq_calib_mtm.y, imtq_calib_mtm.z, 
            ants_response[0], ants_response[1],
            posatt.lon*100.0, posatt.cor_lat*100.0, posatt.elev*10.0);
}

void telem_lev0_read_metadata(telem_control_type* c)
{
    int page1[256];
    int page2[256];
    char meta_data[256];
    char page_data[256];
    int i;

    // read metadata from SFM for this telemetry level
    sfm_read_page(TM0_ADDR1, TM0_ADDR2, page1);
    // copy from data (int array) to meta_data (char array)
    for (i=0 ; i<255 ; i++)
    {
        meta_data[i] = page1[i] & 0x00ff;
    }
    // force null-termination in last place, for safety
    meta_data[255]=0;

    // read pagedata from SFM for this telemetry level
    sfm_read_page(TM0_ADDR1, TM0_ADDR2+1, page2);
    // copy from data (int array) to page_data (char array)
    for (i=0 ; i<255 ; i++)
    {
        page_data[i] = page2[i] & 0x00ff;
    }
    // force null-termination in last place, for safety
    page_data[255]=0;
    
    // scan meta data from string into data struct
    sscanf(meta_data,"%d %d %d %d %d %ld %d %s %s",&c->record_period, &c->rec_per_page, 
            &c->page_per_block, &c->first_sector, &c->num_sectors, &c->record_count, &c->page_count,
            c->first_timestamp, c->last_timestamp);
    
    // scan page_data from string into data struct
    sscanf(page_data,"%s",c->pagedata);
    // if this is the special string, then replace with null
    if (strcmp(c->pagedata,"X") == 1) strcpy(c->pagedata,"");
}

void telem_lev1_read_metadata(telem_control_type* c)
{
    int page1[256];
    int page2[256];
    char meta_data[256];
    char page_data[256];
    int i;
   
    // read metadata from SFM for this telemetry level
    sfm_read_page(TM1_ADDR1, TM1_ADDR2, page1);
    // copy from data (int array) to meta_data (char array)
    for (i=0 ; i<255 ; i++)
    {
        meta_data[i] = page1[i] & 0x00ff;
    }
    // force null-termination in last place, for safety
    meta_data[255]=0;

    // read pagedata from SFM for this telemetry level
    sfm_read_page(TM1_ADDR1, TM1_ADDR2+1, page2);
    // copy from data (int array) to page_data (char array)
    for (i=0 ; i<255 ; i++)
    {
        page_data[i] = page2[i] & 0x00ff;
    }
    // force null-termination in last place, for safety
    page_data[255]=0;
    
    // scan meta data from string into data struct
    sscanf(meta_data,"%d %d %d %d %d %ld %d %s %s",&c->record_period, &c->rec_per_page, 
            &c->page_per_block, &c->first_sector, &c->num_sectors, &c->record_count, &c->page_count,
            c->first_timestamp, c->last_timestamp);
    
    // scan page_data from string into data struct
    sscanf(page_data,"%s",c->pagedata);
    // if this is the special string, then replace with null
    if (strcmp(c->pagedata,"X") == 1) strcpy(c->pagedata,"");
}

void telem_lev2_read_metadata(telem_control_type* c)
{
    int page1[256];
    int page2[256];
    char meta_data[256];
    char page_data[256];
    int i;

    // read metadata from SFM for this telemetry level
    sfm_read_page(TM2_ADDR1, TM2_ADDR2, page1);
    // copy from data (int array) to meta_data (char array)
    for (i=0 ; i<255 ; i++)
    {
        meta_data[i] = page1[i] & 0x00ff;
    }
    // force null-termination in last place, for safety
    meta_data[255]=0;

    // read pagedata from SFM for this telemetry level
    sfm_read_page(TM2_ADDR1, TM2_ADDR2+1, page2);
    // copy from data (int array) to page_data (char array)
    for (i=0 ; i<255 ; i++)
    {
        page_data[i] = page2[i] & 0x00ff;
    }
    // force null-termination in last place, for safety
    page_data[255]=0;
    
    // scan meta data from string into data struct
    sscanf(meta_data,"%d %d %d %d %d %ld %d %s %s",&c->record_period, &c->rec_per_page, 
            &c->page_per_block, &c->first_sector, &c->num_sectors, &c->record_count, &c->page_count,
            c->first_timestamp, c->last_timestamp);
    
    // scan page_data from string into data struct
    sscanf(page_data,"%s",c->pagedata);
    // if this is the special string, then replace with null
    if (strcmp(c->pagedata,"X") == 1) strcpy(c->pagedata,"");
}

void telem_gather_lev0(telem_control_type* c)
{
    // In the following, a block is a group of data pages written between timestamp pages
    
    char isodatetime[30];
    char new_str[16];
    int len;
    float scaled_value;
    long int pagerec_count;   // the current record count on current page
    long int blockpage_count; // the current page count in current block
    long int block_count;     // the current block count
    int sector, sector_page;
    char meta_data[256];
    char page_data[256];
    
    // read the metadata from SFM
    telem_lev0_read_metadata(c);
    
    // calculate minute, hour, and day count from telem_count
    pagerec_count = c->record_count % c->rec_per_page;
    blockpage_count = (c->record_count/c->rec_per_page) % c->page_per_block;
    block_count = c->record_count/(c->rec_per_page * c->page_per_block);
    
    // calculate the current sector and page from page_count
    // includes wrapping when sector goes beyond num_sectors
    sector = ((c->page_count / PAGES_PER_SECTOR) % c->num_sectors) + c->first_sector;
    sector_page = c->page_count % PAGES_PER_SECTOR;

    if (sector_page == 0)
    {
        // first page in new sector, so do a sector erase
        sfm_erase_64k(sector);
    }
    
    // on count = 0, write a time stamp page
    if (pagerec_count == 0)
    {
        // if this is the start of a page period, initialize the page string
        sprintf(c->pagedata,"T0%02ld",blockpage_count);
        
        // if this is also the start of a new block, write a timestamp page
        if (blockpage_count == 0)
        {
            // get the ISO-formatted date+time
            get_isodatetime(isodatetime);
            len = strlen(isodatetime);
            // write the timestamp page
            sfm_write_page(sector, sector_page, isodatetime, len+1);
            // save this timestamp info to control structure
            // update first once, update last always
            if (c->first_timestamp[0]=='X')
            {
                strcpy(c->first_timestamp, isodatetime);
            }
            strcpy(c->last_timestamp, isodatetime);
            // increment the page count
            c->page_count = c->page_count+1;
            
            // page_count updated, so recalculate sector and sector_page
            // sector wrapping if needed, and erase new sector, if needed
            sector = ((c->page_count / PAGES_PER_SECTOR) % c->num_sectors) + c->first_sector;
            sector_page = c->page_count % PAGES_PER_SECTOR;
            if (sector_page == 0)
            {
                // first page in new sector, so do a sector erase
                sfm_erase_64k(sector);
            }
        }
    }
    
    // gather telemetry, format, and concatenate to page string
    scaled_value = bat_get_batv();
    // voltage is reported in V, so multiply by 100 to get value that captures 
    // down to hundredths of a volt.
    sprintf(new_str,",%3.0lf",scaled_value * 100.0);
    strcat(c->pagedata,new_str);
    // increment the telemetry counter
    c->record_count = c->record_count + 1;
    
    // if this is the end of the page, write page and increment page_count
    if (pagerec_count == (c->rec_per_page - 1))
    {
        len = strlen(c->pagedata);
        sfm_write_page(sector, sector_page, c->pagedata, len+1);
        c->page_count = c->page_count + 1;
        // reset pagedata with null
        c->pagedata[0] = 0;
    }
    
    // erase the target 4k block for meta-data
    sfm_erase_4k(TM0_ADDR1, TM0_ADDR2, TM0_ADDR3);
    // save the current meta data to SFM
    sprintf(meta_data,"%d %d %d %d %d %ld %d %s %s",c->record_period, c->rec_per_page,
            c->page_per_block, c->first_sector, c->num_sectors, c->record_count,
            c->page_count, c->first_timestamp, c->last_timestamp);
    len = strlen(meta_data);
    sfm_write_page(TM0_ADDR1, TM0_ADDR2, meta_data, len);
    
    // save the current page data to SFM to allow seamless restart
    if (c->pagedata[0]==0)
    {
        strcpy(page_data,"X");
    }
    else
    {
        strcpy(page_data,c->pagedata);
    }
    len = strlen(page_data);
    sfm_write_page(TM0_ADDR1, TM0_ADDR2+1, page_data, len);
}

void telem_gather_lev1(telem_control_type* c)
{
    // In the following, a block is a group of data pages written between timestamp pages
    
    char isodatetime[30];
    int len;
    long int pagerec_count;   // the current record count on current page
    long int blockpage_count; // the current page count in current block
    long int block_count;     // the current block count
    int sector, sector_page;
    char meta_data[256];
    char page_data[256];
    
     // read the metadata from SFM
    telem_lev1_read_metadata(c);
   
    // calculate minute, hour, and day count from telem_count
    pagerec_count = c->record_count % c->rec_per_page;
    blockpage_count = (c->record_count/c->rec_per_page) % c->page_per_block;
    block_count = c->record_count/(c->rec_per_page * c->page_per_block);
    
    // calculate the current sector and page from page_count
    // includes wrapping when sector goes beyond num_sectors
    sector = ((c->page_count / PAGES_PER_SECTOR) % c->num_sectors) + c->first_sector;
    sector_page = c->page_count % PAGES_PER_SECTOR;

    if (sector_page == 0)
    {
        // first page in new sector, so do a sector erase
        sfm_erase_64k(sector);
    }
    
    // on count = 0, write a time stamp page
    if (pagerec_count == 0)
    {
        // if this is the start of a page period, initialize the page string
        sprintf(c->pagedata,"T1%02ld",blockpage_count);
        
        // if this is also the start of a new block, write a timestamp page
        if (blockpage_count == 0)
        {
            // get the ISO-formatted date+time
            get_isodatetime(isodatetime);
            len = strlen(isodatetime);
            // write the timestamp page
            sfm_write_page(sector, sector_page, isodatetime, len+1);
            // save this timestamp info to control structure
            // update first once, update last always
            if (c->first_timestamp[0]=='X')
            {
                strcpy(c->first_timestamp, isodatetime);
            }
            strcpy(c->last_timestamp, isodatetime);
            // increment the page count
            c->page_count = c->page_count+1;
            
            // page_count updated, so recalculate sector and sector_page
            // sector wrapping if needed, and erase new sector, if needed
            sector = ((c->page_count / PAGES_PER_SECTOR) % c->num_sectors) + c->first_sector;
            sector_page = c->page_count % PAGES_PER_SECTOR;
            if (sector_page == 0)
            {
                // first page in new sector, so do a sector erase
                sfm_erase_64k(sector);
            }
        }
    }
    
    // gather telemetry, format, and concatenate to page string
    strcat(c->pagedata,beacon_msg);
    // increment the telemetry counter
    c->record_count = c->record_count + 1;
    
    // if this is the end of the page, write page and increment page_count
    if (pagerec_count == (c->rec_per_page - 1))
    {
        len = strlen(c->pagedata);
        sfm_write_page(sector, sector_page, c->pagedata, len+1);
        c->page_count = c->page_count + 1;
        // reset pagedata with null
        c->pagedata[0] = 0;
    }
    
    // erase the target 4k block for meta-data
    sfm_erase_4k(TM1_ADDR1, TM1_ADDR2, TM1_ADDR3);
    // save the current meta data to SFM
    sprintf(meta_data,"%d %d %d %d %d %ld %d %s %s",c->record_period, c->rec_per_page,
            c->page_per_block, c->first_sector, c->num_sectors, c->record_count,
            c->page_count, c->first_timestamp, c->last_timestamp);
    len = strlen(meta_data);
    sfm_write_page(TM1_ADDR1, TM1_ADDR2, meta_data, len);
    
    // save the current page data to SFM to allow seamless restart
    if (c->pagedata[0]==0)
    {
        strcpy(page_data,"X");
    }
    else
    {
        strcpy(page_data,c->pagedata);
    }
    len = strlen(page_data);
    sfm_write_page(TM1_ADDR1, TM1_ADDR2+1, page_data, len);
}

void telem_gather_lev2(telem_control_type* c)
{
    // In the following, a block is a group of data pages written between timestamp pages
    
    char isodatetime[30];
    int len;
    long int pagerec_count;   // the current record count on current page
    long int blockpage_count; // the current page count in current block
    long int block_count;     // the current block count
    int sector, sector_page;
    char meta_data[256];
    char page_data[256];

    // read the metadata from SFM
    telem_lev2_read_metadata(c);
    
    // calculate minute, hour, and day count from telem_count
    pagerec_count = c->record_count % c->rec_per_page;
    blockpage_count = (c->record_count/c->rec_per_page) % c->page_per_block;
    block_count = c->record_count/(c->rec_per_page * c->page_per_block);
    
    // calculate the current sector and page from page_count
    // includes wrapping when sector goes beyond num_sectors
    sector = ((c->page_count / PAGES_PER_SECTOR) % c->num_sectors) + c->first_sector;
    sector_page = c->page_count % PAGES_PER_SECTOR;

    if (sector_page == 0)
    {
        // first page in new sector, so do a sector erase
        sfm_erase_64k(sector);
    }
    
    // on count = 0, write a time stamp page
    if (pagerec_count == 0)
    {
        // if this is the start of a page period, initialize the page string
        sprintf(c->pagedata,"T2%02ld",blockpage_count);
        
        // if this is also the start of a new block, write a timestamp page
        if (blockpage_count == 0)
        {
            // get the ISO-formatted date+time
            get_isodatetime(isodatetime);
            len = strlen(isodatetime);
            // write the timestamp page
            sfm_write_page(sector, sector_page, isodatetime, len+1);
            // save this timestamp info to control structure
            // update first once, update last always
            if (c->first_timestamp[0]=='X')
            {
                strcpy(c->first_timestamp, isodatetime);
            }
            strcpy(c->last_timestamp, isodatetime);
            // increment the page count
            c->page_count = c->page_count+1;
            
            // page_count updated, so recalculate sector and sector_page
            // sector wrapping if needed, and erase new sector, if needed
            sector = ((c->page_count / PAGES_PER_SECTOR) % c->num_sectors) + c->first_sector;
            sector_page = c->page_count % PAGES_PER_SECTOR;
            if (sector_page == 0)
            {
                // first page in new sector, so do a sector erase
                sfm_erase_64k(sector);
            }
        }
    }
    
    // gather telemetry, format, and concatenate to page string
    strcat(c->pagedata,",Lev2_test");
    // increment the telemetry counter
    c->record_count = c->record_count + 1;
    
    // if this is the end of the page, write page and increment page_count
    if (pagerec_count == (c->rec_per_page - 1))
    {
        len = strlen(c->pagedata);
        sfm_write_page(sector, sector_page, c->pagedata, len+1);
        c->page_count = c->page_count + 1;
        // reset pagedata with null
        c->pagedata[0] = 0;
    }
    
    // erase the target 4k block for meta-data
    sfm_erase_4k(TM2_ADDR1, TM2_ADDR2, TM2_ADDR3);
    // save the current meta data to SFM
    sprintf(meta_data,"%d %d %d %d %d %ld %d %s %s",c->record_period, c->rec_per_page,
            c->page_per_block, c->first_sector, c->num_sectors, c->record_count,
            c->page_count, c->first_timestamp, c->last_timestamp);
    len = strlen(meta_data);
    sfm_write_page(TM2_ADDR1, TM2_ADDR2, meta_data, len);
    
    // save the current page data to SFM to allow seamless restart
    if (c->pagedata[0]==0)
    {
        strcpy(page_data,"X");
    }
    else
    {
        strcpy(page_data,c->pagedata);
    }
    len = strlen(page_data);
    sfm_write_page(TM2_ADDR1, TM2_ADDR2+1, page_data, len);
}
