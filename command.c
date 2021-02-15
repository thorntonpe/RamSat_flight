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
#include "telemetry.h"
#include "arducam.h"
#include "arducam_user.h"
#include "spi.h"
#include "eps_bat.h"
#include "clock.h"
#include "adc.h"
#include "init.h"
#include "imtq.h"
#include "ants.h"
#include "position_attitude.h"
#include <stdlib.h>
#include <stdio.h>
#include <string.h>

#define B_SIZE 120            // buffer size for reading files
#define B_SIZE2 (B_SIZE*2)+1  // buffer to hold the ascii hex equivalent

// common arrays for downlink data and he100 response
char downlink_data[260];
char downlink_data2[260];
unsigned char he100_response[8];

// declare external global variable for initialization data, defined in Ramsat_flight_main.c
extern init_data_type init_data;

// declare external global variable for position and attitude data, defined in Ramsat_flight_main.c
extern position_attitude_type posatt;

// declare external global variable for the TLE data, defined (for now) in RamSat_flight_main.c
extern tle_t tle;

// declare external global variables for telemetry control, defined in RamSat_flight_main.c
extern telem_control_type telem_lev0;   // Level 0 control data
extern telem_control_type telem_lev1;   // Level 1 control data
extern telem_control_type telem_lev2;   // Level 2 control data

// declare external global variable for watchdog timer interrupt
extern volatile int minute_elapsed;

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
    unsigned char file_data[B_SIZE];   // the latest chunk of data read from file
    unsigned char hex_data[B_SIZE2];   // the two-byte hex equivalent of the file data
    unsigned bytes_read;      // the number of bytes in most recent read
    int npackets;             // number of packets that will be sent
    int packet_num;           // current packet
    int n_param;              // number of parameters passed to command
    int n_minutes_elapsed;    // a counter for minutes elapsed during downlink
    
    n_minutes_elapsed = 0;
    // read parameter string, return if too few parameters
    n_param = sscanf(paramstr,"%s",fname);
    if (n_param != 1)
    {
        sprintf(downlink_data,"RamSat: CmdFileDump->wrong n_param: %d",n_param);
        he100_transmit_packet(he100_response, downlink_data);                
        return 1;
    }
    
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
        SD_umount();
        return FError;
    }
    
    // first calculate the expected number of packets, adding one for a short packet
    npackets = fp1->size/B_SIZE;
    if (fp1->size%B_SIZE) npackets++;
    
    // send a message with filename, size, and number of packets to expect
    sprintf(downlink_data,"RamSat: %s is open to read: Size=%ld: npackets=%d",fname,fp1->size,npackets);
    he100_transmit_packet(he100_response, downlink_data);

    // loop until the file is empty
    packet_num = 0;
    do
    {
        // check the watchdog timer interrupt, reset watchdog if needed
        // required here because the image dump can take longer than the normal
        // watchdog period (4 minutes)
        if (minute_elapsed)
        {
            eps_reset_watchdog();
            minute_elapsed = 0;
            n_minutes_elapsed++;
        }
        
        // read a chunk of data from the file
        bytes_read = freadM(file_data,B_SIZE,fp1);

        // if any bytes were read, form a packet and send
        if (bytes_read)
        {
            // loop through the bytes and convert to hex equivalent
            for (i=0 ; i<bytes_read ; i++)
            {
                memcpy(&hex_data[i*2],&hex_lut[file_data[i]*2],2);
            }

            // null terminate the hex_data and form packet with header and data
            hex_data[bytes_read*2]=0;
            sprintf(downlink_data,"RS: %4d %s",packet_num,hex_data);
            he100_transmit_packet(he100_response, downlink_data);            

            // 100 msec delay to prevent overrunning transmit buffer
            TMR1 = 0;
            while (TMR1 < 100*TMR1MSEC);
            
            packet_num++;
        }
    } while (bytes_read == B_SIZE);
    
    // check the packet number and write a final line to report completion
    if (packet_num == npackets)
    {
        sprintf(downlink_data,"RamSat: %s dump complete, correct packet count: %d (%d minutes)",fname, packet_num, n_minutes_elapsed);
        he100_transmit_packet(he100_response, downlink_data);
        err = 0;
    }
    else
    {
        sprintf(downlink_data,"RamSat: %s dump complete, incorrect packet count: %d",fname, packet_num);
        he100_transmit_packet(he100_response, downlink_data);
        err = 1;
    }
    
    // unmount the SD card, free memory, and return
    SD_umount();
    free (fp1->buffer);
    free(fp1);
    return err;            
}

// Downlink the contents of a named file in a series of numbered packets
// user specifies the first and last packet numbers in a range to downlink
int CmdFileDumpRange(char *paramstr)
{
    int err = 0;
    MEDIA * sd_dat;    // pointer to SD card data structure
    MFILE * fp1;       // pointer to file data structure
    int i;
    char fname[13];
    fname[12]=0;
    int first_packet, last_packet;     // user-specified range of packets (base-zero)
    unsigned char file_data[B_SIZE];   // the latest chunk of data read from file
    unsigned char hex_data[B_SIZE2];   // the two-byte hex equivalent of the file data
    unsigned bytes_read;      // the number of bytes in most recent read
    int npackets;             // number of packets that will be sent
    int packet_num;           // current packet
    int n_param;              // number of parameters passed to command
    int n_minutes_elapsed;    // a counter for minutes elapsed during downlink
    
    n_minutes_elapsed = 0;
    // read parameter string, return if too few parameters
    n_param = sscanf(paramstr,"%s %d %d",fname, &first_packet, &last_packet);
    if (n_param != 3)
    {
        sprintf(downlink_data,"RamSat: CmdFileDumpRange->wrong n_param: %d",n_param);
        he100_transmit_packet(he100_response, downlink_data);                
        return 1;
    }
    
    // attempt to mount SD card
    sd_dat = SD_mount();
    if (!sd_dat)
    {
        sprintf(downlink_data,"RamSat: CmdFileDumpRange->SD_mount Error: %d",FError);
        he100_transmit_packet(he100_response, downlink_data);                
        return FError;
    }
    
    // open specified file for reading
    fp1 = fopenM(fname, "r");
    if (!fp1)
    {
        sprintf(downlink_data,"RamSat: CmdFileDumpRange->fopenM Error: %d",FError);
        he100_transmit_packet(he100_response, downlink_data); 
        SD_umount();
        return FError;
    }
    
    // first calculate the expected number of packets, adding one for a short packet
    npackets = fp1->size/B_SIZE;
    if (fp1->size%B_SIZE) npackets++;
    
    // send a message with filename, size, and number of packets to expect
    sprintf(downlink_data,"RamSat: %s is open to read: Size=%ld: npackets=%d first_packet=%d last_packet=%d",
            fname,fp1->size,npackets,first_packet,last_packet);
    he100_transmit_packet(he100_response, downlink_data);

    // loop until the file is empty
    packet_num = 0;
    do
    {
        // check the watchdog timer interrupt, reset watchdog if needed
        // required here because the image dump can take longer than the normal
        // watchdog period (4 minutes)
        if (minute_elapsed)
        {
            eps_reset_watchdog();
            minute_elapsed = 0;
            n_minutes_elapsed++;
        }
        
        // read a chunk of data from the file
        bytes_read = freadM(file_data,B_SIZE,fp1);

        // if any bytes were read, form a packet and send
        if (bytes_read && (packet_num >= first_packet) && (packet_num <= last_packet))
        {
            // loop through the bytes and convert to hex equivalent
            for (i=0 ; i<bytes_read ; i++)
            {
                memcpy(&hex_data[i*2],&hex_lut[file_data[i]*2],2);
            }
            
            // null terminate the hex_data and form packet with header and data
            hex_data[bytes_read*2]=0;
            sprintf(downlink_data,"RS: %4d %s",packet_num,hex_data);
            he100_transmit_packet(he100_response, downlink_data);            

            // 100 msec delay to prevent overrunning transmit buffer
            TMR1 = 0;
            while (TMR1 < 100*TMR1MSEC);
            
            //for (k=0 ; k<10 ; k++)
            //{
            //    TMR1=0;
            //    while(TMR1 < 1000*TMR1MSEC);
            //}
        }
        if (packet_num == last_packet) break;
        packet_num++;
    } while (bytes_read == B_SIZE);
    
    // check the packet number and write a final line to report completion
    if (packet_num == last_packet)
    {
        sprintf(downlink_data,"RamSat: %s dump complete, correct last_packet: %d (%d minutes)",fname, packet_num, n_minutes_elapsed);
        he100_transmit_packet(he100_response, downlink_data);
        err = 0;
    }
    else
    {
        sprintf(downlink_data,"RamSat: %s dump complete, incorrect last_packet: %d",fname, packet_num);
        he100_transmit_packet(he100_response, downlink_data);
        err = 1;
    }
    
    // unmount the SD card, free memory, and return
    SD_umount();
    free (fp1->buffer);
    free(fp1);
    return err;            
}

// Downlink a single numbered packet from a named file
int CmdFileDumpOnePacket(char *paramstr)
{
    int err = 0;
    MEDIA * sd_dat;    // pointer to SD card data structure
    MFILE * fp1;       // pointer to file data structure
    int i;
    char fname[13];
    fname[12]=0;
    unsigned char file_data[B_SIZE];   // the latest chunk of data read from file
    unsigned char hex_data[B_SIZE2];   // the two-byte hex equivalent of the file data
    unsigned bytes_read;      // the number of bytes in most recent read
    int npackets;             // number of packets that will be sent
    int packet_num;           // current packet
    int req_pacnum;           // the index for single packet requested to downlink
    int n_param;              // number of parameters passed to command
    
    // read parameter string, return if too few parameters
    n_param = sscanf(paramstr,"%s %d",fname, &req_pacnum);
    if (n_param != 2)
    {
        sprintf(downlink_data,"RamSat: CmdFileDumpOnePacket->wrong n_param: %d",n_param);
        he100_transmit_packet(he100_response, downlink_data);                
        return 1;
    }

    // attempt to mount SD card
    sd_dat = SD_mount();
    if (!sd_dat)
    {
        sprintf(downlink_data,"RamSat: CmdFileDumpOnePacket->SD_mount Error: %d",FError);
        he100_transmit_packet(he100_response, downlink_data);                
        return FError;
    }
    
    // open specified file for reading
    fp1 = fopenM(fname, "r");
    if (!fp1)
    {
        sprintf(downlink_data,"RamSat: CmdFileDumpOnePacket->fopenM Error: %d",FError);
        he100_transmit_packet(he100_response, downlink_data);                
        SD_umount();
        return FError;
    }
    
    // first calculate the expected number of packets, adding one for a short packet
    npackets = fp1->size/B_SIZE;
    if (fp1->size%B_SIZE) npackets++;
    // send a message with filename, size, and number of packets to expect
    sprintf(downlink_data,"RamSat: %s is open to read packet: Size=%ld: npackets=%u",fname,fp1->size,npackets);
    he100_transmit_packet(he100_response, downlink_data);
    
    packet_num = 0;
    // loop until the file is empty
    do
    {
        // read a chunk of data from the file
        bytes_read = freadM(file_data,B_SIZE,fp1);
        // if any bytes were read, form a packet and send
        if (bytes_read && packet_num == req_pacnum)
        {
            // loop through the bytes and convert to hex equivalent
            for (i=0 ; i<bytes_read ; i++)
            {
                memcpy(&hex_data[i*2],&hex_lut[file_data[i]*2],2);
            }
            // null terminate the hex_data
            hex_data[bytes_read*2]=0;
            sprintf(downlink_data,"RS: %4d %s", packet_num, hex_data);
            he100_transmit_packet(he100_response, downlink_data); 
            break;
        }
        packet_num++;
    } while (bytes_read == B_SIZE);
    
    // check the packet number and write a final line to report completion
    if (packet_num == req_pacnum)
    {
        sprintf(downlink_data,"RamSat: %s dump complete, packet %d found",fname, req_pacnum);
        he100_transmit_packet(he100_response, downlink_data);
        err = 0;
    }
    else
    {
        sprintf(downlink_data,"RamSat: %s dump complete, packet %d NOT found %d",fname, req_pacnum, packet_num);
        he100_transmit_packet(he100_response, downlink_data);
        err = 0;
    }
    
    // unmount the SD card, free memory, and return
    SD_umount();
    free (fp1->buffer);
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
                // problem clearing RTC flags
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

// downlink telemetry control data for requested telemetry level (0, 1, or 2))
int CmdGetTelemControl(char* paramstr)
{
    int err = 0;
    int telem_level;
    telem_control_type* p;
    
    // read one parameters from parameter string
    telem_level = atoi(paramstr); 
    
    // error checking on available telemetry levels
    if (telem_level < 0 || telem_level > 2)
    {
        err = 1;
    }
    else
    {
        // select which telemetry level control data to report
        switch (telem_level)
        {
            case 0:  // Level 0 telemetry control
                p = &telem_lev0;
                break;
                
            case 1:  // Level 1 telemetry control
                p = &telem_lev1;
                break;
                
            case 2:  // Level 2 telemetry control
                p = &telem_lev2;
                break;
        }
        
        // using the assigned pointer, downlink requested telemetry control data
        sprintf(downlink_data,"RamSat: Reporting telemetry control data for Level %d",telem_level);
        he100_transmit_packet(he100_response, downlink_data);
        sprintf(downlink_data,"Lev0: First timestamp = %s, last timestamp = %s", 
                p->first_timestamp, p->last_timestamp);
        he100_transmit_packet(he100_response, downlink_data);
        sprintf(downlink_data,"Lev0: First sector = %d, num_sectors = %d, page_count = %d, record_count = %d",
                p->first_sector, p->num_sectors, p->page_count, p->record_count);
        he100_transmit_packet(he100_response, downlink_data);
        sprintf(downlink_data,"Lev0: record_period = %d, rec_per_page = %d, page_per_block = %d",
                p->record_period, p->rec_per_page, p->page_per_block);
        he100_transmit_packet(he100_response, downlink_data);
        if (p->pagedata[0] != 0)
        {
            sprintf(downlink_data,"Lev0: Unwritten page data follows...");
            he100_transmit_packet(he100_response, downlink_data);
            sprintf(downlink_data,"%s",p->pagedata);
            he100_transmit_packet(he100_response, downlink_data);
            
        }
        else
        {
            sprintf(downlink_data,"Lev0: No unwritten page data");
            he100_transmit_packet(he100_response, downlink_data);
        }
    }
    return err;
}

// downlink telemetry data by sector and page range
int CmdGetTelemData(char* paramstr)
{
    int err = 0;
    int n_param;
    int sector, page;
    int start_page, stop_page;
    int data[256];
    int i;
    
    n_param = sscanf(paramstr,"%d %d %d",&sector, &start_page, &stop_page);
    
    // error checking
    if (n_param != 3 || sector > 127 || start_page > 255 || stop_page > 255)
    {
        err = 1;
    }
    else
    {
        sprintf(downlink_data,"RamSat: Sending telemetry data for sector %d, start_page = %d, stop_page = %d",
                sector, start_page, stop_page);
        he100_transmit_packet(he100_response, downlink_data);
        
        for (page=start_page ; page<= stop_page ;  page++)
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
    }
    
    return err;
}

// Return the current telemetry for one system, specified by the index value in paramstr
int CmdCurrentTelemetry(char* paramstr)
{
    int err = 0;
    int n_param;
    int index = 0;
    unsigned char bat_status, eps_status;
    int pdm_initial, pdm_expected, pdm_actual;
    int ischarging, bat_nbr, bat_nar, bat_nmr;
    float batv, bati, bat_mbt, bat_dbt;
    float eps_bcr1v, eps_bcr2v, eps_bcr3v, eps_bcroutv;
    float eps_bcr1ia, eps_bcr1ib, eps_bcr2ia, eps_bcr2ib, eps_bcr3ia, eps_bcr3ib;
    float eps_bati, eps_bus12i, eps_bus5i, eps_bus33i, eps_eps5i, eps_eps33i, eps_mbt;
    imtq_resp_common imtq_common;       // iMTQ response from every command
    imtq_resp_mtm imtq_calib_mtm;       // iMTQ calibrated magnetometer data

    // get the index parameter
    n_param = sscanf(paramstr, "%d", &index);
    
    // return current telemetry depending on index
    switch(index)
    {
        case 1:  // battery telemetry
            // response header
            sprintf(downlink_data,"RamSat: CmdCurrentTelemetry->Retrieving battery telemetry, index %d", index);
            he100_transmit_packet(he100_response, downlink_data);
            // a delay to let the transmitter complete before getting telemetry
            TMR1 = 0;
            while (TMR1 < 1000*TMR1MSEC);
            // gather telemetry
            bat_status = bat_get_status();
            batv = bat_get_batv();
            bati = bat_get_bati();
            ischarging = bat_get_batischarging();
            bat_mbt = bat_get_mbt();
            bat_dbt = bat_get_dbt();
            bat_nbr = bat_get_nbr();
            bat_nar = bat_get_nar();
            bat_nmr = bat_get_nmr();
            // format and send response
            sprintf(downlink_data,"Bat Telem: 0x%02x %.2f %.2f %d %.2f %.2f %d %d %d",
                    bat_status, batv, bati, ischarging, bat_mbt, bat_dbt, bat_nbr, bat_nar, bat_nmr);
            he100_transmit_packet(he100_response, downlink_data);
            break;
        
        case 2:  // EPS telemetry
            // response header
            sprintf(downlink_data,"RamSat: CmdCurrentTelemetry->Retrieving EPS telemetry, index %d", index);
            he100_transmit_packet(he100_response, downlink_data);
            // a delay to let the transmitter complete before getting telemetry
            TMR1 = 0;
            while (TMR1 < 1000*TMR1MSEC);
            // gather telemetry
            eps_status = eps_get_status();
            eps_bcr1v = eps_get_bcr1v();
            eps_bcr2v = eps_get_bcr2v();
            eps_bcr3v = eps_get_bcr3v();
            eps_bcroutv = eps_get_bcroutv();
            eps_bcr1ia = eps_get_bcr1ia();
            eps_bcr1ib = eps_get_bcr1ib();
            eps_bcr2ia = eps_get_bcr2ia();
            eps_bcr2ib = eps_get_bcr2ib();
            eps_bcr3ia = eps_get_bcr3ia();
            eps_bcr3ib = eps_get_bcr3ib();
            eps_bati = eps_get_bati();
            eps_bus12i = eps_get_bus12i();
            eps_bus5i = eps_get_bus5i();
            eps_bus33i = eps_get_bus33i();
            eps_eps5i = eps_get_eps5i();
            eps_eps33i = eps_get_eps33i();
            eps_mbt = eps_get_mbt();            
            // format and send response
            sprintf(downlink_data,"EPS Telem: 0x%02x %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f",
                    eps_status, eps_bcr1v, eps_bcr2v, eps_bcr3v, eps_bcroutv,
                    eps_bcr1ia, eps_bcr1ib, eps_bcr2ia, eps_bcr2ib, eps_bcr3ia, eps_bcr3ib,
                    eps_bati, eps_bus12i, eps_bus5i, eps_bus33i, eps_eps5i, eps_eps33i, eps_mbt);
            he100_transmit_packet(he100_response, downlink_data);
            break;
            
        case 3:  // sun sensor telemetry
            // response header
            sprintf(downlink_data,"RamSat: CmdCurrentTelemetry->Retrieving sun sensor telemetry, index %d", index);
            he100_transmit_packet(he100_response, downlink_data);
            // a delay to let the transmitter complete before getting telemetry
            TMR1 = 0;
            while (TMR1 < 1000*TMR1MSEC);
            // gather telemetry
            adc_scan_all();
            // format and send response
            sprintf(downlink_data,"SS Telem: +X=(%d, %d) -X=(%d, %d) +Y=(%d, %d) -Y=(%d, %d)",
                    ADC1BUF4, ADC1BUF0, ADC1BUF6, ADC1BUF2, ADC1BUF3, ADC1BUF7, ADC1BUF5, ADC1BUF1);
            he100_transmit_packet(he100_response, downlink_data);
            break;
            
        case 4:  // IMTQ magnetometer telemetry
            // response header
            sprintf(downlink_data,"RamSat: CmdCurrentTelemetry->Retrieving IMTQ magnetometer telemetry, index %d", index);
            he100_transmit_packet(he100_response, downlink_data);
            // a delay to let the transmitter complete before getting telemetry
            TMR1 = 0;
            while (TMR1 < 1000*TMR1MSEC);
            // gather telemetry: MTM data in frame coordinates
            // start the MTM measurement
            imtq_start_mtm(&imtq_common);
            // delay for MTM integration
            TMR1 = 0;
            while (TMR1 <= 82 * TMR1MSEC);
            // get the calibrated MTM data
            imtq_get_calib_mtm(&imtq_common, &imtq_calib_mtm);
            // format and send response
            sprintf(downlink_data,"MTM Telem: B_fx=%ld, B_fy=%ld, B_fz=%ld", 
                    imtq_calib_mtm.x, imtq_calib_mtm.y, imtq_calib_mtm.z);
            he100_transmit_packet(he100_response, downlink_data);
            break;
            
        case 5:  // Startup telemetry
            // response header
            sprintf(downlink_data,"RamSat: CmdCurrentTelemetry->Retrieving startup telemetry, index %d", index);
            he100_transmit_packet(he100_response, downlink_data);
            // telemetry is already in init_data structure from startup sequence
            sprintf(downlink_data,"Init Telem: %ld %ld %ld %d %d %d %d %d %d %d %d %d %d %d %d %d %d 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x %d %d %d %d %d %d %d %d %.2f %s %s",
                    init_data.u2br_actual, init_data.i2c1br, init_data.i2c2br,
                    init_data.adc_iserror, init_data.sfm_iserror, init_data.sd_iserror,
                    init_data.rtc_flags_iserror, init_data.rtc_flags2_iserror, init_data.rtc_clear_iserror,
                    init_data.rtc_flags, init_data.rtc_flags2, init_data.pdt_status, init_data.pdt_flag,
                    init_data.eps_antenna_on_iserror, init_data.antenna_on_status, 
                    init_data.eps_antenna_off_iserror, init_data.antenna_off_status,
                    init_data.ants0_deploy_status_msb, init_data.ants0_deploy_status_lsb,
                    init_data.ants1_deploy_status_msb, init_data.ants1_deploy_status_lsb,
                    init_data.ants2_deploy_status_msb, init_data.ants2_deploy_status_lsb,
                    init_data.ants3_deploy_status_msb, init_data.ants3_deploy_status_lsb,
                    init_data.ants_deploy_time1_msb, init_data.ants_deploy_time1_lsb,
                    init_data.ants_deploy_time2_msb, init_data.ants_deploy_time2_lsb,
                    init_data.ants_deploy_time3_msb, init_data.ants_deploy_time3_lsb,
                    init_data.ants_deploy_time4_msb, init_data.ants_deploy_time4_lsb,
                    init_data.batv, init_data.rtc_halt_time, init_data.rtc_init_time
                    );
            he100_transmit_packet(he100_response, downlink_data);
            break;
            
        case 6: // position and attitude telemetry
            // response header
            sprintf(downlink_data,"RamSat: CmdCurrentTelemetry->Retrieving position and attitude telemetry, index %d", index);
            he100_transmit_packet(he100_response, downlink_data);
            // telemetry is already in the posatt data structure, as long as 
            // a TLE has been uploaded.
            sprintf(downlink_data,"PosAtt Telem: %.2lf %.2lf %.2lf %.2lf %.2lf %.2lf %.2lf %.2lf %.2lf %.2lf %.2lf %.2lf %.2lf %.2lf %.2lf %.2lf %.2lf %.2lf %.2lf %.2lf %.2lf %.2lf %.2lf %.2lf %.2lf %.2lf %.2lf %.2lf %.2lf %.2lf %.2lf %.2lf",
                    posatt.jd, posatt.t_since, posatt.x_eci, posatt.y_eci, posatt.z_eci,
                    posatt.lon, posatt.lat, posatt.cor_lat, posatt.elev, posatt.lst,
                    posatt.B_x, posatt.B_y, posatt.B_z,
                    posatt.B_fx, posatt.B_fy, posatt.B_fz, posatt.b_x, posatt.b_y, posatt.b_z,
                    posatt.bf_x, posatt.bf_y, posatt.bf_z, posatt.s_x, posatt.s_y, posatt.s_z,
                    posatt.sf_x, posatt.sf_y, posatt.sf_z,
                    posatt.q0, posatt.q1, posatt.q2, posatt.q3);
            he100_transmit_packet(he100_response, downlink_data);
            break;
            
        case 7: // antenna telemetry
            // response header
            sprintf(downlink_data,"RamSat: CmdCurrentTelemetry->Retrieving antenna telemetry, index %d", index);
            he100_transmit_packet(he100_response, downlink_data);
            // a delay to let the transmitter complete before getting telemetry
            TMR1 = 0;
            while (TMR1 < 1000*TMR1MSEC);
            // power on the antenna
            eps_antenna_on();
            // a delay to let the antenna circuitry initialize
            TMR1 = 0;
            while (TMR1 < 100*TMR1MSEC);
            // read antenna deployment status
            unsigned char ants_response[2]; // holds response from antenna commands
            ants_deploy_status(ants_response);
            // power off the antenna
            eps_antenna_off();
            sprintf(downlink_data,"Antenna Telem: 0x%02x 0x%02x", ants_response[0],ants_response[1]);
            he100_transmit_packet(he100_response, downlink_data);
            break;
            
        case 8: // PDM switch telemetry (subset of EPS)
            // response header
            sprintf(downlink_data,"RamSat: CmdCurrentTelemetry->Retrieving PDM telemetry, index %d", index);
            he100_transmit_packet(he100_response, downlink_data);
            // a delay to let the transmitter complete before getting telemetry
            TMR1 = 0;
            while (TMR1 < 1000*TMR1MSEC);
            
            // test code to set initial state
            //eps_set_pdm_initial_off(1);
            //eps_set_pdm_initial_off(2);
            //eps_set_pdm_initial_off(3);
            //eps_set_pdm_initial_off(4);
            //eps_set_pdm_initial_off(5);
            //eps_set_pdm_initial_off(6);
            //eps_set_pdm_initial_off(7);
            //eps_set_pdm_initial_off(8);
            //eps_set_pdm_initial_off(9);
            //eps_set_pdm_initial_off(10);
            
            eps_set_pdm_all_off();
            
            // get the initial state of all PDM switches (on power-up)
            pdm_initial = eps_get_pdm_initial();
            // get the expected state of all PDM switches
            pdm_expected = eps_get_pdm_expected();
            // get the actual state of all PDM switches
            pdm_actual = eps_get_pdm_actual();
            sprintf(downlink_data,"PDM Telem: Initial=%d, Expected=%d, Actual=%d",
                    pdm_initial, pdm_expected, pdm_actual);
            he100_transmit_packet(he100_response, downlink_data);
            break;
            
        default:
            sprintf(downlink_data,"RamSat: CmdCurrentTelemetry->ERROR, invalid index (%d)", index);
            he100_transmit_packet(he100_response, downlink_data);
    }
    
    return err;
}

// capture image on CAM1 and CAM2
int CmdCaptureImage(char* paramstr)
{
    int err = 0;
    int image_number;         // user-defined index for commanded images (0-999)
    MEDIA * sd_dat;           // pointer to SD card data structure
    MFILE * fp;               // pointer to file data structure
    char fname[16];           // character string to hold filename
    long cap_msec_cam1, cap_msec_cam2;  // number of miliseconds required to capture image
    long fifo_length_cam1, fifo_length_cam2; // length of FIFO buffer on arduchip
    int len1, len2, len3;     // bytes read from length registers
    long totb;                // total bytes written to file
    long bcount;              // byte count for write
    char imgdata[SDBUFSIZE];  // temporary buffer to hold data between FIFO and SD file
    int byte1;                // single byte read from FIFO via SPI
    int i;                    // counter
    unsigned int nb;          //number of bytes written per chunk
    unsigned int nwrite;      // number of bytes requested to write per chunk

    // get the image number from command parameter
    image_number = atoi(paramstr);
    
    // attempt to mount SD card
    sd_dat = SD_mount();
    if (!sd_dat)
    {
        sprintf(downlink_data,"RamSat: CmdCaptureImage->SD_mount Error: %d",FError);
        he100_transmit_packet(he100_response, downlink_data);                
        err = FError;
    }
    else
    {
        // good SD card mount, so proceed with image capture on both cameras
        // clear the arducam output buffer for both cameras
        arduchip_clear_fifo(CAM1);
        arduchip_clear_fifo(CAM2);
        // capture an image from the camera, store time in msec fro each capture
        cap_msec_cam1 = arduchip_start_capture(CAM1);
        cap_msec_cam2 = arduchip_start_capture(CAM2);
        // read the size of camera fifo buffers
        fifo_length_cam1 = arduchip_fifo_length(&len1, &len2, &len3, CAM1);
        fifo_length_cam2 = arduchip_fifo_length(&len1, &len2, &len3, CAM2);
        
        // open a file in write mode, for image data output from camera 1
        sprintf(fname,"CMD1_%03d.JPG",image_number);
        fp = fopenM(fname, "w");
        if (!fp)
        {
            sprintf(downlink_data,"RamSat: CmdCaptureImage->File open error cam1 #%d",FError);
            he100_transmit_packet(he100_response, downlink_data);                
            err = FError;
        }
        else
        {
            // write image data from camera 1 to SD card
            // read the image data from the FIFO in burst mode
            CS_CAM1 = 0;         // select the device
            write_spi2(0x3c);   // start burst read mode
            totb = 0L;          // total bytes written
            bcount = fifo_length_cam1;  // counter to keep track of bytes to read
            while (bcount > 0)
            {
                if (bcount < SDBUFSIZE)
                    nwrite=bcount;
                else
                    nwrite=SDBUFSIZE;
                // read nwrite bytes from FIFO and store in imgdata buffer
                for (i=0 ; i<nwrite ; i++)
                {
                    byte1 = write_spi2(0x00);
                    imgdata[i] = byte1;
                }
                // write the bytes in imgdata buffer to file
                nb = fwriteM(imgdata, nwrite, fp);
                // update counters
                bcount -= nwrite;
                totb += nb;
            }
            CS_CAM1 = 1;  // deselect the device
            // close the file
            fcloseM(fp);
            sprintf(downlink_data,"RamSat: CmdCaptureImage->wrote file %s, size=%ld bytes",
                    fname, totb);
            he100_transmit_packet(he100_response, downlink_data);
            TMR1=0;
            while(TMR1 < 1000 * TMR1MSEC);
        }
        
        // open a file in write mode, for image data output from camera 2
        sprintf(fname,"CMD2_%03d.JPG",image_number);
        fp = fopenM(fname, "w");
        if (!fp)
        {
            sprintf(downlink_data,"RamSat: CmdCaptureImage->File open error cam2 #%d",FError);
            he100_transmit_packet(he100_response, downlink_data);                
            err = FError;
        }
        else
        {
            // write image data from camera 2 file to SD card
            // read the image data from the FIFO in burst mode
            CS_CAM2 = 0;         // select the device
            write_spi2(0x3c);   // start burst read mode
            totb = 0L;          // total bytes written
            bcount = fifo_length_cam2;  // counter to keep track of bytes to read
            while (bcount > 0)
            {
                if (bcount < SDBUFSIZE)
                    nwrite=bcount;
                else
                    nwrite=SDBUFSIZE;
                // read nwrite bytes from FIFO and store in imgdata buffer
                for (i=0 ; i<nwrite ; i++)
                {
                    byte1 = write_spi2(0x00);
                    imgdata[i] = byte1;
                }
                // write the bytes in imgdata buffer to file
                nb = fwriteM(imgdata, nwrite, fp);
                // update counters
                bcount -= nwrite;
                totb += nb;
            }
            CS_CAM2 = 1;  // deselect the device
            // close the file
            fcloseM(fp);
            sprintf(downlink_data,"RamSat: CmdCaptureImage->wrote file %s, size=%ld bytes",
                    fname, totb);
            he100_transmit_packet(he100_response, downlink_data);
            TMR1=0;
            while(TMR1 < 1000 * TMR1MSEC);
        }
        // unmount the SD card
        SD_umount();
    }
    return err;
}
    
// Turn power to camera board on (1) or off (0)
// When powering on, also initialize cameras
int CmdCameraPower(char* paramstr)
{
    int err = 0;
    int state = 0;
    int init_response;
    
    // get the state parameter
    state = atoi(paramstr);
    
    switch(state)
    {
        case 0:
            eps_cameras_off();
            sprintf(downlink_data,"RamSat: CmdCameraPower->OFF, Successful.");
            he100_transmit_packet(he100_response, downlink_data);
            break;
            
        case 1:
            eps_cameras_on();
            init_response = init_arducam();
            err = init_response;
            sprintf(downlink_data,"RamSat: CmdCameraPower->ON & INIT, Successful.");
            he100_transmit_packet(he100_response, downlink_data);
            break;
            
        default:
            sprintf(downlink_data,"RamSat: CmdCameraPower->ERROR, Invalid state.");
            he100_transmit_packet(he100_response, downlink_data);
            err = 1;
    }
    
    return err;
}

// Delete a named file from SD card
int CmdFileDelete(char *paramstr)
{
    int err = 0;
    MEDIA * sd_dat;    // pointer to SD card data structure
    char fname[13];
    fname[12]=0;
    int n_param;              // number of parameters passed to command
    
    // read parameter string, return if too few parameters
    n_param = sscanf(paramstr,"%s",fname);
    if (n_param != 1)
    {
        sprintf(downlink_data,"RamSat: CmdFileDelete->wrong n_param: %d",n_param);
        he100_transmit_packet(he100_response, downlink_data);                
        return 1;
    }
    
    // attempt to mount SD card
    sd_dat = SD_mount();
    if (!sd_dat)
    {
        sprintf(downlink_data,"RamSat: CmdFileDelete->SD_mount Error: %d",FError);
        he100_transmit_packet(he100_response, downlink_data);                
        return FError;
    }
    
    // delete specified file 
    err = fdeleteM(fname, "d");
    
    // unmount SD card
    SD_umount();
    
    if (err)
    {
        sprintf(downlink_data,"RamSat: CmdFileDelete->fdeleteM Error: %d",FError);
        he100_transmit_packet(he100_response, downlink_data);                
    }
    else
    {
        sprintf(downlink_data,"RamSat: %s deleted",fname);
        he100_transmit_packet(he100_response, downlink_data);                
    }
    
    return err;
}

// begin a detumble operation with iMTQ, specifying a number of seconds,
// a flag for whether or not to report results during the detumble, and the
// report frequency. Send 0 0 for the second and third parameter for no reporting
int CmdStartDetumble(char *paramstr)
{
    int err = 0;
    int n_param;              // number of parameters passed to command
    unsigned short nseconds;  // number of seconds to operate detumble
    int do_report;            // flag to downlink interim detumble data reports
    int report_freq;          // number of seconds between downlink reports
    int seconds_elapsed;      // detumble timer
    int report_count;         // current report number
    float batv, bati;         // additional battery telemetry during detumble
    imtq_resp_common imtq_common;       // iMTQ response from every command
    imtq_resp_detumble imtq_detumble;   // iMTQ detumble data
    
    // read parameter string, return if too few parameters
    n_param = sscanf(paramstr,"%hu %d %d",&nseconds, &do_report, &report_freq);
    if (n_param != 3)
    {
        sprintf(downlink_data,"RamSat: CmdStartDetumble->wrong n_param: %d",n_param);
        he100_transmit_packet(he100_response, downlink_data);                
        return 1;
    }
    
    // send iMTQ command to start the B-Dot detumble operation
    imtq_start_detumble(nseconds, &imtq_common);
    
    // check the response and report status
    if (imtq_common.cc == 0x09 && imtq_common.stat == 0x80)
    {
        // good iMTQ response
        sprintf(downlink_data,"RamSat: Detumbling for %d seconds...",nseconds);
        he100_transmit_packet(he100_response, downlink_data);
        
        // if user requested reporting, gather and downlink detumble data every second
        if (do_report)
        {
            report_count = 0;
            seconds_elapsed = 0;
            do {
                // wait one second, increment timer
                TMR1=0;
                while(TMR1 < 1000L*TMR1MSEC);
                seconds_elapsed++;

                // check if report period
                if (seconds_elapsed % report_freq == 0)
                {
                    // read detumble data
                    imtq_get_detumble_data(&imtq_common, &imtq_detumble);

                    // check response and downlink if no error
                    if (imtq_common.cc == 0x48 && imtq_common.stat == 0x80)
                    {
                        sprintf(downlink_data,"RamSat: DetumbleReport %d Cal(X,Y,Z)=(%ld,%ld,%ld) Filt(X,Y,Z)=(%ld,%ld,%ld) BDot(X,Y,Z)=(%ld,%ld,%ld)",
                                seconds_elapsed, 
                                imtq_detumble.cal_x, imtq_detumble.cal_y, imtq_detumble.cal_z,
                                imtq_detumble.filt_x, imtq_detumble.filt_y, imtq_detumble.filt_z,
                                imtq_detumble.bdot_x, imtq_detumble.bdot_y, imtq_detumble.bdot_z);
                        he100_transmit_packet(he100_response, downlink_data);
                        sprintf(downlink_data,"RamSat: DetumbleReport %d Dip(X,Y,Z)=(%d,%d,%d) CmdCur(X,Y,Z)=(%d,%d,%d) ActCur(X,Y,Z)=(%d,%d,%d)",
                                seconds_elapsed,
                                imtq_detumble.dip_x, imtq_detumble.dip_y, imtq_detumble.dip_z,
                                imtq_detumble.ccur_x, imtq_detumble.ccur_y, imtq_detumble.ccur_z,
                                imtq_detumble.cur_x, imtq_detumble.cur_y, imtq_detumble.cur_z);
                        he100_transmit_packet(he100_response, downlink_data);
                        batv = bat_get_batv();
                        bati = bat_get_bati();
                        sprintf(downlink_data,"RamSat: DetumbleReport %d Batv=%.2f Bati=%.2f",seconds_elapsed,batv, bati);
                        he100_transmit_packet(he100_response, downlink_data);
                    }
                    else
                    {
                        sprintf(downlink_data,"RamSat: DetumbleReport->iMTQ Error: cc=0x%02x, stat=0x%02x",
                                imtq_common.cc,imtq_common.stat);
                        he100_transmit_packet(he100_response, downlink_data);
                    }
                } // end report period
            } while (seconds_elapsed < nseconds);
        } // end do_report
    } // end good response to start detumble
    else
    {
        // error in iMTQ response
        sprintf(downlink_data,"RamSat: CmdStartDetumble->iMTQ Error: cc=0x%02x, stat=0x%02x",
                imtq_common.cc,imtq_common.stat);
        he100_transmit_packet(he100_response, downlink_data);
        err = 1;
    }
    return err;
}
    
// Set the MUST_WAIT flag for post-deployment timer, in SFM
void CmdSetPDT(void)
{
    // set flag that forces code to wait on reset
    sfm_write_1byte(PDT_ADR1, PDT_ADR2, PDT_ADR3, MUST_WAIT);

    // downlink status message
    sprintf(downlink_data,"RamSat: CmdSetPDT->MUST_WAIT flag for post-deployment timer is set.");
    he100_transmit_packet(he100_response, downlink_data);
}

// Power-cycle the flight radio, via EPS reset on the BatV bus (500 ms)
void CmdResetHe100(void)
{
    eps_batvbus_reset();
    // wait one second to allow bus and He-100 to reset after command
    long wait = 1000 * TMR1MSEC;
    TMR1 = 0;
    while (TMR1 < wait);
    sprintf(downlink_data,"RamSat: CmdResetHe100->Reset successful.");
    he100_transmit_packet(he100_response, downlink_data);
}
// Reset the flight computer. Try to resolve stuck code or frozen peripherals.
void CmdResetPIC(void)
{
    sprintf(downlink_data,"RamSat: Attempting flight computer RESET...");
    he100_transmit_packet(he100_response, downlink_data);

    exit(0);
}