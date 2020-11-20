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

#define PAGES_PER_SECTOR 256    // number of 256-byte pages per 64KB sector on SFM

void telem_gather_lev0(telem_control_type* c)
{
    // In the following, a block is a group of data pages written between timestamp pages
    
    char isodatetime[128];
    char new_str[16];
    int len;
    float scaled_value;
    int pagerec_count;   // the current record count on current page
    int blockpage_count; // the current page count in current block
    int block_count;     // the current block count
    int sector, sector_page;
    
    // calculate minute, hour, and day count from telem_count
    pagerec_count = c->record_count % c->rec_per_page;
    blockpage_count = (c->record_count/c->rec_per_page) % c->page_per_block;
    block_count = c->record_count/(c->rec_per_page * c->page_per_block);
    
    // calculate the current sector and page from page_count
    sector = (c->page_count / PAGES_PER_SECTOR) + c->first_sector;
    sector_page = c->page_count % PAGES_PER_SECTOR;
    
    // wrap back to first sector if the number of sectors goes above limit
    if (sector > (c->first_sector + c->num_sectors))
    {
        sector = c->first_sector;
    }
    if (sector_page == 0)
    {
        // first page in new sector, so do a sector erase
        sfm_erase_64k(sector);
    }
    
    // on count = 0, write a time stamp page
    if (pagerec_count == 0)
    {
        // if this is the start of a page period, initialize the page string
        sprintf(c->pagedata,"%02d",blockpage_count);
        
        // if this is also the start of a new block, write a timestamp page
        if (blockpage_count == 0)
        {
            // get the ISO-formatted date+time
            get_isodatetime(isodatetime);
            len = strlen(isodatetime);
            // write the timestamp page
            sfm_write_page(sector, sector_page, isodatetime, len+1);
            // increment the page count
            c->page_count = c->page_count+1;
            
            // page_count updated, so recalculate sector and sector_page
            // and erase new sector, if needed
            sector = (c->page_count / PAGES_PER_SECTOR) + c->first_sector;
            sector_page = c->page_count % PAGES_PER_SECTOR;
            // wrap back to first sector if the number of sectors goes above limit
            if (sector > (c->first_sector + c->num_sectors))
            {
                sector = c->first_sector;
            }
            if (sector_page == 0)
            {
                // first page in new sector, so do a sector erase
                sfm_erase_64k(sector);
            }
        }
    }
    
    // gather telemetry, format, and concatenate to page string
    scaled_value = bat_get_batv();
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
    }
}
