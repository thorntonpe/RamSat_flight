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
#include <stdlib.h>     // malloc...
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
        SD_umount();
        err = FError;
        return err;
    }
    
    // write ntrans bytes to file
    sprintf(test_text_w,"SD write test");
    ntrans = 13;
    nbw = fwriteM(test_text_w, ntrans, fp1);
    if (nbw != ntrans)
    {
        fcloseM(fp1);
        fdeleteM(fname, "d");
        SD_umount();
        err = 1;
        return err;
    }
    
    // close the file
    fcloseM(fp1);
    
    // reopen the same file for reading
    fp1 = fopenM(fname, "r");
    if (!fp1)
    {
        SD_umount();
        err = FError;
        return err;
    }
    
    // read ntrans bytes from file
    nbr = freadM(test_text_r, ntrans, fp1);
    if (nbr != ntrans)
    {
        fcloseM(fp1);
        fdeleteM(fname, "d");
        err = 1;
        return err;
    }
    
    // compare the write and read strings
    err = memcmp(test_text_w, test_text_r, ntrans);

    // close file, then delete
    fcloseM(fp1);
    fdeleteM(fname, "d");
    
    // Unmount the SD card
    SD_umount();
    
    return err;
}

// Mount SD card, write a short file
// This routine can be used for testing in flight, but will leave the file on disk
int test_sd_write(void)
{
    int err = 0;
    char test_text_w[15]; // character string to hold sample text written to file
    char fname[12];       // character string to hold filename, testing SD card
    unsigned int ntrans;  //number of bytes to write/read
    unsigned int nbw;     //number of bytes written to file
    
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
    sprintf(fname,"SD_W2.TXT");
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

// test: mount SD card, list all files from FAT with output to RS232
int test_sd_list(void)
{
    int err = 0;
    int i;
    unsigned num_files = 0;
    char name[9];
    char ext[4];
    char msg[128];      // character string for messages to user via COM port
    name[8]=0;
    ext[3]=0;
    // attempt to mount SD card
    sd_dat = SD_mount();
    if (!sd_dat)
    {
        err = FError;
        return err;
    }
    
    // write out MEDIA metadata
    sprintf(msg,"SD Card: Max entries in root dir = %u",sd_dat->maxroot);
    write_string1(msg);
    sprintf(msg,"SD Card: Max clusters in partition = %u",sd_dat->maxcls);
    write_string1(msg);
    sprintf(msg,"SD Card: Number of sectors = %u",sd_dat->fatsize);
    write_string1(msg);
    sprintf(msg,"SD Card: Number of copies of FAT = %u",sd_dat->fatcopy);
    write_string1(msg);
    sprintf(msg,"SD Card: Number of sectors per cluster = %u",sd_dat->sxc);
    write_string1(msg);
    
    // Now write out metadata for all files
    // first get the count of files on the card
    err = CountDIR(&num_files);
    sprintf(msg,"SD Card: Number of files = %u",num_files);
    write_string1(msg);
    
    // now go through all files and get filenames and sizes
    // allocate a MFILE structure on the heap
    fp1 = (MFILE *) malloc( sizeof( MFILE));
    if ( fp1 == NULL)            // report an error  
    {   
        FError = FE_MALLOC_FAILED;
        return FError;
    }
    for (i=0 ; i<num_files ; i++)
    {
        err = flistM(fp1, i);
        if (err)
        {
            sprintf(msg,"SD File List: error = %d",err);
            write_string1(msg);
            continue;
        }
        // print file info
        memcpy(name,fp1->name,8);
        memcpy(ext,fp1->name+8,3);
        int year = (fp1->date>>9  & 0x7f) + 1980;
        int mon  = fp1->date>>5  & 0x0f;
        int day  = fp1->date     & 0x1f;
        int hrs  = fp1->time>>11 & 0x1f;
        int min  = fp1->time>>5  & 0x3f;
        int sec  = (fp1->time     & 0x1f) *2;
        sprintf(msg,"File# %d: Name: %s.%s Size: %ld Date: %d-%d-%d Time: %d:%d:%d",
                i,name,ext,fp1->size,year,mon,day,hrs,min,sec);
        write_string1(msg);
    }
    sprintf(msg,"SD Card: CountDIR iserror = %u",err);
    write_string1(msg);
    
    // unmount the SD card and free memory
    SD_umount();
    free(fp1);
    
    return err;            
}
