/*
 * File:       command.c
 * Author:     Peter Thornton
 * Purpose:    top-level handlers for uplink commands
 * Created on: 14 October 2020
 *  
*/

#include "xc.h"
#include "command.h"
#include "sd_card.h"  // includes FError as a global mailbox for SD errors
#include "he100.h"
#include "hex_lut.h"
#include "sgp4.h"
#include "parse_tle.h"
#include "datetime.h"
#include "sfm.h"
#include "pdt.h"
#include "rtc.h"
#include "rtc_user.h"
#include <stdlib.h>
#include <stdio.h>
#include <string.h>

#define B_SIZE 10            // buffer size for reading files
#define B_SIZE2 (B_SIZE*2)+1  // buffer to hold the ascii hex equivalent

// a common array for downlink data
char downlink_data[260];
// a common array for He-100 response to transmit command
unsigned char he100_response[8];

// declare a global variable for the TLE data, defined (for now) in RamSat_flight_main.c
extern tle_t tle;

// A No-Op command to verify 2-way radio connection
void CmdNoOp(void)
{
    sprintf(downlink_data,"RamSat: No-Op command acknowledged.");
    he100_transmit_packet(he100_response, downlink_data);
}

// Return the number of files on SD card
int CmdFileCount(void)
{
    int err = 0;
    unsigned nfiles;
    MEDIA * sd_dat;    // pointer to SD card data structure

    // attempt to mount SD card
    sd_dat = SD_mount();
    if (!sd_dat)
    {
        sprintf(downlink_data,"RamSat: CmdFileCount->SD_mount Error: %d",FError);
        he100_transmit_packet(he100_response, downlink_data);                
        return FError;
    }
    
    // get the number of files on SD card
    err = CountDIR(&nfiles);
    if (!err)
    {
        // form data portion of downlink packet, pass to transmit routine
        sprintf(downlink_data,"RamSat: Number of files = %u",nfiles);
        he100_transmit_packet(he100_response, downlink_data);
    }
    else
    {
        // special message if error getting file count
        sprintf(downlink_data,"RamSat: CmdFileCount->CountDIR Error: %d",err);
        he100_transmit_packet(he100_response, downlink_data);                
    }
    
    // unmount the SD card and free memory
    SD_umount();
    return err;
}

// List all files on SD card
int CmdFileList(void)
{
    int err = 0;
    MEDIA * sd_dat;    // pointer to SD card data structure
    MFILE * fp1;       // pointer to file data structure
    int i;
    unsigned nfiles;
    char name[9];      // file name
    char ext[4];       // file extension
    name[8]=0;         // null termination
    ext[3]=0;          // null termination

    // attempt to mount SD card
    sd_dat = SD_mount();
    if (!sd_dat)
    {
        sprintf(downlink_data,"RamSat: CmdFileList->SD_mount Error: %d",FError);
        he100_transmit_packet(he100_response, downlink_data);                
        return FError;
    }
    
    // allocate a MFILE structure on the heap
    fp1 = (MFILE *) malloc( sizeof( MFILE));
    if ( fp1 == NULL)            // report an error  
    {   
        FError = FE_MALLOC_FAILED;
        sprintf(downlink_data,"RamSat: CmdFileList->malloc Error: %d",FError);
        he100_transmit_packet(he100_response, downlink_data);                
        return FError;
    }
    
    // get the number of files on SD card
    err = CountDIR(&nfiles);
    if (!err)
    {
        // send a first packet that reports number of files
        sprintf(downlink_data,"RamSat: Number of files = %u",nfiles);
        he100_transmit_packet(he100_response, downlink_data);
        
        // loop through files, transmit filename, size, and creation date
        for (i=0 ; i<nfiles ; i++)
        {
            err = flistM(fp1, i);
            if (!err)
            {
                // extract file info
                memcpy(name,fp1->name,8);
                memcpy(ext,fp1->name+8,3);
                int year = (fp1->date>>9  & 0x7f) + 1980;
                int mon  = fp1->date>>5  & 0x0f;
                int day  = fp1->date     & 0x1f;
                int hrs  = fp1->time>>11 & 0x1f;
                int min  = fp1->time>>5  & 0x3f;
                int sec  = (fp1->time     & 0x1f) *2;
                sprintf(downlink_data,"RamSat: File# %d: Name: %s.%s Size: %ld Date: %d-%d-%d Time: %d:%d:%d",
                        i,name,ext,fp1->size,year,mon,day,hrs,min,sec);
                he100_transmit_packet(he100_response, downlink_data);
            }
            else
            {
                // special message if error getting file details
                sprintf(downlink_data,"RamSat: CmdFileList->File# %d: Error: %d",i,err);
                he100_transmit_packet(he100_response, downlink_data);                
            }
        }
    }
    else
    {
        // special message if error getting file count
        sprintf(downlink_data,"RamSat: CmdFileList->CountDIR Error: %d",err);
        he100_transmit_packet(he100_response, downlink_data);                
    }
    
    // unmount the SD card, free memory, and return
    SD_umount();
    free(fp1);
    return err;            
}

// Downlink the contents of a named file in a series of numbered packets
int CmdFileDump(char *paramstr)
{
    int err = 0;
    MEDIA * sd_dat;    // pointer to SD card data structure
    MFILE * fp1;       // pointer to file data structure
    int i;
    char fname[13];
    fname[12]=0;
    char file_data[B_SIZE];   // the latest chunk of data read from file
    char hex_data[B_SIZE2];   // the two-byte hex equivalent of the file data
    unsigned bytes_read;      // the number of bytes in most recent read
    unsigned long total_bytes;// total bytes read for the file
    int npackets;             // number of packets that will be sent
    int packet_num;           // current packet
    
    // assumes that the parameter passed to CmdFileDump is an 8.3 filename
    memcpy(fname,paramstr,12);

    // attempt to mount SD card
    sd_dat = SD_mount();
    if (!sd_dat)
    {
        sprintf(downlink_data,"RamSat: CmdFileDump->SD_mount Error: %d",FError);
        he100_transmit_packet(he100_response, downlink_data);                
        return FError;
    }
    
    // open specified file for reading
    fp1 = fopenM(fname, "r");
    if (!fp1)
    {
        sprintf(downlink_data,"RamSat: CmdFileDump->fopenM Error: %d",FError);
        he100_transmit_packet(he100_response, downlink_data);                
        return FError;
    }
    
    // first calculate the expected number of packets, adding one for a short packet
    npackets = fp1->size/B_SIZE;
    if (fp1->size%B_SIZE) npackets++;
    // send a message with filename, size, and number of packets to expect
    sprintf(downlink_data,"RamSat: %s is open to read: Size=%ld: npackets=%u",fname,fp1->size,npackets);
    he100_transmit_packet(he100_response, downlink_data);
    packet_num = 0;
    total_bytes = 0;
    // loop until the file is empty
    do
    {
        // read a chunk of data from the file
        bytes_read = freadM(file_data,B_SIZE,fp1);
        // if any bytes were read, form a packet and send
        if (bytes_read)
        {
            // sum all bytes read
            total_bytes += bytes_read;
            // loop through the bytes and convert to hex equivalent
            for (i=0 ; i<bytes_read ; i++)
            {
                memcpy(&hex_data[i*2],&hex_lut[file_data[i]*2],2);
            }
            // null terminate the hex_data
            hex_data[bytes_read*2]=0;
            sprintf(downlink_data,"RamSat:%6d %s",packet_num,hex_data);
            he100_transmit_packet(he100_response, downlink_data);            
        }
        packet_num++;
    } while (bytes_read == B_SIZE);
    
    // check the packet number and write a final line to report completion
    if (packet_num == npackets)
    {
        sprintf(downlink_data,"RamSat: %s dump complete, correct packet count: %d",fname, packet_num);
        he100_transmit_packet(he100_response, downlink_data);
        err = 0;
    }
    else
    {
        sprintf(downlink_data,"RamSat: %s dump complete, incorrect packet count: %d",fname, packet_num);
        he100_transmit_packet(he100_response, downlink_data);
        err = 0;
    }
    
    // unmount the SD card, free memory, and return
    SD_umount();
    free(fp1);
    return err;            
}

// Uplink a new Two-Line Element (TLE) from ground station, update current TLE
int CmdNewTLE(char *paramstr, int param_nbytes, int *good_tle)
{
    int err = 0;
    
    // check for the right amount of data in parameter
    if (param_nbytes != 138)
    {
        // clear flag and downlink error message
        *good_tle = 0;
        sprintf(downlink_data,"RamSat: NewTLE ERROR - Received %d bytes, expecting 138", param_nbytes);
        he100_transmit_packet(he100_response, downlink_data);
        err = 1;
    }
    else
    {
        // good size, split out the two lines
        char line1[70]; // TLE Line 1 (first 69 characters)
        char line2[70]; // TLE Line 2 (second 69 characters)
        memcpy(line1,paramstr,69);
        memcpy(line2,paramstr+69,69);
        // line1 and line2 need to be null-terminated strings
        line1[69]=0;
        line2[69]=0;

        // call function to parse elements from TLE
        err = parse_elements(line1, line2, &tle);
        if (err)
        {
            // clear flag and downlink error message
            *good_tle = 0;
            sprintf(downlink_data,"RamSat: NewTLE ERROR - parse_elements() err = %d",err);
            he100_transmit_packet(he100_response, downlink_data);
        }
        else
        {
            // TLE parsed without error
            // set flag and downlink success message
            *good_tle = 1;
            sprintf(downlink_data,"RamSat: NewTLE successful, TLE updated");
            he100_transmit_packet(he100_response, downlink_data);
        }
    }
    return err;
}

// return the current date and time from RTC
int CmdGetDateTime(void)
{
    char datetime[128];
    int err = 0;
    
    err = get_isodatetime(datetime);
    sprintf(downlink_data,"RamSat: ISO DateTime = %s",datetime);
    he100_transmit_packet(he100_response, downlink_data);
    return err;
}

// set the date and time on RTC
int CmdSetDateTime(char* paramstr, int param_nbytes)
{
    int err = 0;
    int rtc_flags;
    int nbytes;
    unsigned char rtc_data[8];
    unsigned char firstbyte;
    char numstr[3];
    int num1, num10;
    
    // null-terminated string to hold two-digit int
    numstr[2]=0;
    
    // check for the right amount of data in parameter
    if (param_nbytes != 24)
    {
        // clear flag and downlink error message
        sprintf(downlink_data,"RamSat: Set Date/Time Error - Received %d bytes, expecting 24.",param_nbytes);
        he100_transmit_packet(he100_response, downlink_data);
        err = 1;
    }
    else if (paramstr[10] != 'T' || paramstr[22] != 'Z')
    {
        // basic format check fails
        sprintf(downlink_data,"RamSat: Set Date/Time Error - incorrect format");
        he100_transmit_packet(he100_response, downlink_data);
        err = 1;
    }
    else
    {
        // good size and format, split out the elements needed to set RTC
        // Note that the rtc_data values are decimal-coded binary, which is
        // why there are funny-looking translations between the input date/time 
        // values and the values sent to RTC.
        firstbyte = 0x00;
        nbytes = 8;
        
        // hundredths of seconds (00-99)
        num10=paramstr[20]-48;
        num1 =paramstr[21]-48;
        rtc_data[0]=num10*16+num1;
        
        // seconds (0-59)  
        num10=paramstr[17]-48;
        num1 =paramstr[18]-48;
        rtc_data[1]=num10*16+num1;
        
        // minutes (0-59)
        num10=paramstr[14]-48;
        num1 =paramstr[15]-48;
        rtc_data[2]=num10*16+num1;
        
        // hour (00-23)
        num10=paramstr[11]-48;
        num1 =paramstr[12]-48;
        rtc_data[3]=num10*16+num1;
        
        // day of week (1-7)
        num10=0;
        num1 =paramstr[23]-48;
        rtc_data[4]=num10*16+num1;
        
        // day of month (1-31)
        num10=paramstr[8]-48;
        num1 =paramstr[9]-48;
        rtc_data[5]=num10*16+num1;
        
        // month (1-12)
        num10=paramstr[5]-48;
        num1 =paramstr[6]-48;
        rtc_data[6]=num10*16+num1;
        
        // year (00-99) e.g. "20" for 2020
        num10=paramstr[2]-48;
        num1 =paramstr[3]-48;
        rtc_data[7]=num10*16+num1;
        
        // Make sure the HALT, STOP, and OF bits are clear before setting RTC
        // read flags first
        err = rtc_read_flags(&rtc_flags);
        if (err)
        {
            // problem reading flags from RTC
            sprintf(downlink_data,"RamSat: Set Date/Time Error - RTC flag read error");
            he100_transmit_packet(he100_response, downlink_data);
        }
        else
        {
            // clear any flags that are set
            err = rtc_clear_flags(rtc_flags);
            if (err)
            {
                // problem cldearing RTC flags
                sprintf(downlink_data,"RamSat: Set Date/Time Error - RTC flag clearing error");
                he100_transmit_packet(he100_response, downlink_data);
            }
            else
            {
                // write the RTC data to device, acknowledge wia downlink
                rtc_write_nbytes(nbytes, firstbyte, rtc_data);
                sprintf(downlink_data,"RamSat: Set Date/Time completed. Reporting current date/time...");
                he100_transmit_packet(he100_response, downlink_data);
                err = CmdGetDateTime();
                
            }
        }
    }
    return err;
}

// Erase specified 64KB sector on Serial Flash Memory
int CmdEraseSector(char* paramstr)
{
    int err = 0;
    int sector;
    sector = atoi(paramstr);
    // make sure this is in a valid range, and prevent erasing sector 0
    if (sector == 0 || sector > 127)
    {
        err = 1;
    }
    else
    {
        sfm_erase_64k(sector);
    }
    return err;
}

// write one 256-byte page within one sector on SFM
int CmdWritePage(char* paramstr)
{
    int err = 0;
    int sector, page, fill_value;
    int n_param;
    char data[256];
    int i;
    
    // read three parameters from parameter string
    n_param = sscanf(paramstr,"%d %d %d",&sector, &page, &fill_value);
    
    // error checking
    if (n_param != 3 || sector < 1 || sector > 127 || page < 0 || page > 255
            || fill_value < 0 || fill_value > 255)
    {
        err = 1;
    }
    else
    {
        sprintf(downlink_data,"RamSat: test write page fill value = %d",fill_value);
        he100_transmit_packet(he100_response, downlink_data);
        // fill the test array
        for (i=0 ; i<254 ; i++)
        {
            data[i]=fill_value;
        }
        // set a null terminator in the last place
        data[254]=0;
        
        // write the page
        sfm_write_page(sector, page, data, 256);
    }
    return err;
}

// read one 256-byte page within one sector on SFM, downlink as a string
int CmdDownlinkPage(char* paramstr)
{
    int err = 0;
    int sector, page;
    int n_param;
    int data[256];
    int i;
    
    // read two parameters from parameter string
    n_param = sscanf(paramstr,"%d %d",&sector, &page);
    
    // error checking
    if (n_param != 2 || sector < 0 || sector > 127 || page < 0 || page > 255)
    {
        err = 1;
    }
    else
    {
        // read the page into data
        sfm_read_page(sector, page, data);
        // copy from data (int array) to downlink_data (char array)
        for (i=0 ; i<254 ; i++)
        {
            downlink_data[i] = data[i] & 0x00ff;
        }
        // force null-termination in last place, for safety
        downlink_data[254]=0;
        // downlink the page as a packet payload string
        he100_transmit_packet(he100_response, downlink_data);
    }
    return err;
}

// Set the MUST_WAIT flag for post-deployment timer, in SFM
void CmdSetPDT(void)
{
    // set flag that forces code to wait on reset
    sfm_write_1byte(PDT_ADR1, PDT_ADR2, PDT_ADR3, MUST_WAIT);

    // downlink status message
    sprintf(downlink_data,"RamSat: MUST_WAIT flag for post-deployment timer is set.");
    he100_transmit_packet(he100_response, downlink_data);
}

// Reset the flight computer. Try to resolve stuck code or frozen peripherals.
void CmdReset(void)
{
    sprintf(downlink_data,"RamSat: Attempting flight computer RESET...");
    he100_transmit_packet(he100_response, downlink_data);

    exit(0);
}