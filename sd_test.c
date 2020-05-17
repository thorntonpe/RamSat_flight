/*
 * File:       sd_test.c
 * Author:     Peter Thornton
 * Purpose:    Test functions for SD card
 * Created on: 15 May 2020
 *  
*/

#include "xc.h"
#include "sd_card.h"
#include "uart.h"
#include "sd_test.h"
#include <stdio.h>
#include <string.h>

MEDIA * sd_dat; // pointer to SD card data structure
MFILE * fp1;    // pointer to file data structure

// test: mount then unmount the SD card, return error flag
// This routine can be used for testing in flight.
int test_sd_mount_umount(void)
{
    int err = 0;
    // 1. Attempt to mount the SD card
    sd_dat = SD_mount();
    if (!sd_dat) err = FError;
    
    // 2. Unmount the SD card
    if (!err) SD_umount();
    
    return err;
}

// test: mount SD card, write diagnostics to USB, unmount
// This routine is used for ground testing only.
void test_sd_mount_describe(void)
{
    char msg[128];      // character string for messages to user via COM port
    int err = 0;
    // Attempt to mount the SD card
    sd_dat = SD_mount();
    if (!sd_dat) err = FError;
    
    // write information about the storage device to UART2 (USB)
    sprintf(msg,"SD: Mount Test is_error = %d", err);
    write_string2(msg);
    if (!err)
    {
        sprintf(msg,"SD: FAT sector          = %lu", sd_dat->fat);
        write_string2(msg);
        sprintf(msg,"SD: Root sector         = %lu", sd_dat->root);
        write_string2(msg);
        sprintf(msg,"SD: Data sector         = %lu", sd_dat->data);
        write_string2(msg);
        sprintf(msg,"SD: max Root entries    = %hu", sd_dat->maxroot);
        write_string2(msg);
        sprintf(msg,"SD: max Clusters        = %hu", sd_dat->maxcls);
        write_string2(msg);
        sprintf(msg,"SD: FAT size (sectors)  = %hu", sd_dat->fatsize);
        write_string2(msg);
        sprintf(msg,"SD: number FAT copies   = %hu", sd_dat->fatcopy);
        write_string2(msg);
        sprintf(msg,"SD: Sectors per Cluster = %hu", sd_dat->sxc);
        write_string2(msg);
    }
    
    // Unmount the SD card
    if (!err) SD_umount();
}

// Mount SD card, write a short file, read the contents of the file
// and then delete the file.
// This routine can be used for testing in flight
int test_sd_write_read_delete(void)
{
    int err = 0;
    char test_text_w[15]; // character string to hold sample text written to file
    char test_text_r[15]; // character string to hold sample text read from file
    char fname[12];       // character string to hold filename, testing SD card
    unsigned int ntrans;  //number of bytes to write/read
    unsigned int nbw;     //number of bytes written to file
    unsigned int nbr;     //number of bytes read from file
    
    // make sure fname is a null terminated string
    memset(fname, '\0', 12);
    
    // Attempt to mount the SD card
    sd_dat = SD_mount();
    if (!sd_dat)
    {
        err = FError;
        return err;
    }
    
    // open a file for writing
    sprintf(fname,"SD_TEST.TXT");
    fp1 = fopenM(fname, "w");
    if (!fp1)
    {
        err = FError;
        return err;
    }
    
    // write ntrans bytes to file
    sprintf(test_text_w,"SD write test");
    ntrans = 13;
    nbw = fwriteM(test_text_w, ntrans, fp1);
    if (nbw != ntrans)
    {
        err = 1;
        return err;
    }
    
    // close the file
    fcloseM(fp1);
    
    // reopen the same file for reading
    fp1 = fopenM(fname, "r");
    if (!fp1)
    {
        err = FError;
        return err;
    }
    
    // read ntrans bytes from file
    nbr = freadM(test_text_r, ntrans, fp1);
    if (nbr != ntrans)
    {
        err = 1;
        return err;
    }
    
    // compare the write and read strings
    err = memcmp(test_text_w, test_text_r, ntrans);

    // close the file
    fcloseM(fp1);
    
    // delete the file
    if (!err)
    {
        err = fdeleteM(fname, "d");
    }
    
    // Unmount the SD card
    SD_umount();
    
    return err;
}

// test: mount SD card, write a short file, read the contents of the file
// This routine writes to USB, only for ground testing
int test_sd_delete(void)
{
    int err = 0;
    char fname[12];       // character string to hold filename, testing SD card
    
    // make sure fname is a null terminated string
    memset(fname, '\0', 12);
    
    // Attempt to mount the SD card
    sd_dat = SD_mount();
    if (!sd_dat)
    {
        err = FError;
        return err;
    }
    
    // specify a file to delete
    sprintf(fname,"SD_DEL.TXT");
    err = fdeleteM(fname, "d");
    
    // Unmount the SD card
    SD_umount();
    
    return err;
}
