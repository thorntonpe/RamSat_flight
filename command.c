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
#include <stdlib.h>
#include <stdio.h>
#include <string.h>

#define B_SIZE 100            // buffer size for reading files
#define B_SIZE2 (B_SIZE*2)+1  // buffer to hold the ascii hex equivalent

// a common array for downlink data
char downlink_data[255];
// a common array for He-100 response to transmit command
unsigned char he100_response[8];


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
    int i;
    unsigned nfiles;
    MEDIA * sd_dat;    // pointer to SD card data structure
    MFILE * fp1;       // pointer to file data structure
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
    int i;
    char fname[13];
    fname[12]=0;
    MEDIA * sd_dat;           // pointer to SD card data structure
    MFILE * fp1;              // pointer to file data structure
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
    }return err;
}
