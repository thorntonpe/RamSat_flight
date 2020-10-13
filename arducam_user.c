/*
 * File:       arducam_user.c
 * Author:     Peter Thornton
 * Purpose:    High-level user interface for arducam functions
 * Created on: 4 June 2020
 *  
*/

#include "xc.h"
#include "arducam.h"
#include "ov2640_regs.h"
#include "sd_card.h"
#include "uart.h"
#include "spi.h"
#include "arducam_user.h"
#include <stdio.h>
#include <string.h>

int test_arducam_spi(void)
{
    // test the Arducam (arduchip) SPI interface
    // user can pick an arbitrary value in the range 0x00 to 0xFF
    // to send to arduchip_testreg. Should get the same value as return.
    int err = 0;
    int arduchip_testin = 0x7d;
    int arduchip_testout_cam1, arduchip_testout_cam2;
    arduchip_testout_cam1 = arduchip_testreg(arduchip_testin, CAM1);
    arduchip_testout_cam2 = arduchip_testreg(arduchip_testin, CAM2);

    // diagnostics for the SPI peripheral: cameras
    if (arduchip_testout_cam1 != arduchip_testin)
    {
        err = 1;
    }
    if (arduchip_testout_cam2 != arduchip_testin)
    {
        err = 1;
    }
    
    return err;
}

int init_arducam(void)
{
    int err = 0;
    // reset the arduchip on each camera (SPI)
    arduchip_reset(CAM1);
    arduchip_reset(CAM2);
    
    // reset the OV2640 sensor on each camera (I2C)
    reset_ov2640_regs_cam1();
    reset_ov2640_regs_cam2();
    
    // set the OV2640 registers for jpeg output, at 1600x1200 resolution
    init_ov2640_regs_cam1(OV2640_JPEG_1600x1200);
    init_ov2640_regs_cam2(OV2640_JPEG_1600x1200);
    
    // set for a single frame capture
    arduchip_set_nframes(1, CAM1);
    arduchip_set_nframes(1, CAM2);
    
    return err;
}

int test_arducam_capture(void)
{
    // take a series of pictures (Camera #1 or #2), triggered by user input on serial port
    // declare variables for camera capture loop
    int err = 0;
    long cap_msec_cam1, cap_msec_cam2;
    int len1_cam1, len2_cam1, len3_cam1;
    int len1_cam2, len2_cam2, len3_cam2;
    long fifo_length_cam1, fifo_length_cam2;
    int byte1;
    int i;    
    unsigned int nb; //number of bytes written per chunk
    unsigned int nwrite; // number of bytes requested to write per chunk
    long totb;   // total bytes written to file
    long bcount; // byte count for write
    char imgdata[SDBUFSIZE];
    MEDIA * sd_dat; // pointer to SD card data structure
    MFILE * fp1;    // pointer to file data structure
    int cam1_nimg=0;    // counter for the number of images captured by camera 1
    int cam2_nimg=0;    // counter for the number of images captured by camera 2
    int mounted = 0;    // keep track of whether the SD card is mounted
    int ui;             // user input
    char msg[128];      // character string for writing messages to user
    char fname[12];     // character string to hold filename

    // make sure fname is a null terminated string
    memset(fname, '\0', 12);

    while (1)
    {
        write_string1("----- Begin image capture test -----");
        write_string1("Press '1' or '2' to take a picture with camera 1 or 2.");
        write_string1("Press 'm' to mount, 'u' to unmount SD card.");
        write_string1("Press 'x' to exit test.");
        
        // get and echo a character of user input
        ui = read_char1();
        sprintf(msg,"%c", ui);
        write_string1(msg);

        // 'm' mount SD card
        if (ui == 'm')
        {
            // if SD card is already mounted, complain and restart loop
            if (mounted)
            {
                write_string1("Invalid mount command: SD card already mounted.");
                continue;                
            }
            // Attempt to mount the SD card, report result
            sd_dat = SD_mount();
            if (!sd_dat)
            {
                write_string1("Failed to mount SD card.");
                continue;
            }
            write_string1("Mounted SD card.");
            mounted = 1;
        }
        
        // 'u' unmount SD card
        if (ui == 'u')
        {
            // if SD card is not mounted, complain and restart loop
            if (!mounted)
            {
                write_string1("Invalid unmount command: SD card is not mounted.");
                continue;
            }
            // unmount the SD card
            SD_umount();
            write_string1("Unmounted SD card: can now be removed.");
            mounted = 0;
        }
        
        // '1' Capture image with camera 1
        if (ui == '1')
        {
            if (!mounted)
            {
                write_string1("Invalid capture command: SD card is not mounted.");
                write_string1("Make sure card is inserted, and then press 'm' to mount...");
                continue;
            }
            // open a file in write mode, for image data output
            sprintf(fname,"CAM1_%02d.JPG",cam1_nimg);
            fp1 = fopenM(fname, "w");
            if (!fp1)
            {
                sprintf(msg,"File open error #%d",FError);
                write_string1(msg);
                continue;
            }
            // increment the image number
            cam1_nimg++;
            // clear the arducam output buffer
            arduchip_clear_fifo(CAM1);
            // indicate start of image capture
            write_string1("--------------------------------------");
            sprintf(msg,"Begin capture for %s", fname);
            write_string1(msg);
            // capture an image from the camera
            cap_msec_cam1 = arduchip_start_capture(CAM1);
            // read the size of camera fifo buffer
            fifo_length_cam1 = arduchip_fifo_length(&len1_cam1, &len2_cam1, &len3_cam1, CAM1);
            sprintf(msg,"Capture time camera #1 = %ld msec", cap_msec_cam1);
            write_string1(msg);
            sprintf(msg,"FIFO length camera #1= %ld bytes", fifo_length_cam1);
            write_string1(msg);
            // start reading if the length is non-zero
            if (fifo_length_cam1)  
            {
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
                    nb = fwriteM(imgdata, nwrite, fp1);
                    // update counters
                    bcount -= nwrite;
                    totb += nb;
                }
                CS_CAM1 = 1;  // deselect the device
                sprintf(msg,"Wrote %ld bytes to file",totb);
                write_string1(msg);
                // clear the arducam output buffer
                arduchip_clear_fifo(CAM1);
            }
            else
            {
                write_string1("FIFO empty! No data written to file.");
            }
            write_string1("--------------------------------------");
            // close the file
            fcloseM(fp1);
        } // end if ui == '1'
        
        // '2' Capture image with camera 2
        if (ui == '2')
        {
            if (!mounted)
            {
                write_string1("Invalid capture command: SD card is not mounted.");
                write_string1("Make sure card is inserted, and then press 'm' to mount...");
                continue;
            }
            // open a file in write mode, for image data output
            sprintf(fname,"CAM2_%02d.JPG",cam2_nimg);
            fp1 = fopenM(fname, "w");
            if (!fp1)
            {
                sprintf(msg,"File open error #%d",FError);
                write_string1(msg);
                continue;
            }
            // increment the image number
            cam2_nimg++;
            // clear the arducam output buffer
            arduchip_clear_fifo(CAM2);
            // indicate start of image capture
            write_string1("--------------------------------------");
            sprintf(msg,"Begin capture for %s", fname);
            write_string1(msg);
            // capture an image from the camera
            cap_msec_cam2 = arduchip_start_capture(CAM2);
            // read the size of camera fifo buffer
            fifo_length_cam2 = arduchip_fifo_length(&len1_cam2, &len2_cam2, &len3_cam2, CAM2);
            sprintf(msg,"Capture time camera #2 = %ld msec", cap_msec_cam2);
            write_string1(msg);
            sprintf(msg,"FIFO length camera #2= %ld bytes", fifo_length_cam2);
            write_string1(msg);
            // start reading if the length is non-zero
            if (fifo_length_cam2)  
            {
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
                    nb = fwriteM(imgdata, nwrite, fp1);
                    // update counters
                    bcount -= nwrite;
                    totb += nb;
                }
                CS_CAM2 = 1;  // deselect the device
                sprintf(msg,"Wrote %ld bytes to file",totb);
                write_string1(msg);
                // clear the arducam output buffer
                arduchip_clear_fifo(CAM2);
            }
            else
            {
                write_string1("FIFO empty! No data written to file.");
            }
            write_string1("--------------------------------------");
            // close the file
            fcloseM(fp1);
        } // end if ui == '2'
        
        // option to break out of image capture loop
        if (ui == 'x')
        {
            write_string1("Exit image capture.");
            if (mounted)
            {
                write_string1("Un-mounting SD card.");
                SD_umount();
            }
            break;
        }
    } // end image capture loop
    
    return err;
}
