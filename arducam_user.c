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
#include "clock.h"
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

// capture auto image on CAM1 and CAM2
int arducam_auto_image(int image_number)
{
    // image_number is user-supplied integer 0-999
    int err = 0;
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

    // attempt to mount SD card
    sd_dat = SD_mount();
    if (!sd_dat)
    {
        err = FError;
    }
    else
    {
        // good SD card mount, so proceed with image capture on both cameras
        // clear the arducam output buffer for both cameras
        arduchip_clear_fifo(CAM1);
        arduchip_clear_fifo(CAM2);
        // capture an image from the camera, store time in msec for each capture
        cap_msec_cam1 = arduchip_start_capture(CAM1);
        cap_msec_cam2 = arduchip_start_capture(CAM2);
        // read the size of camera fifo buffers
        fifo_length_cam1 = arduchip_fifo_length(&len1, &len2, &len3, CAM1);
        fifo_length_cam2 = arduchip_fifo_length(&len1, &len2, &len3, CAM2);
        
        // open a file in write mode, for image data output from camera 1
        sprintf(fname,"AUT1_%03d.JPG",image_number);
        fp = fopenM(fname, "w");
        if (!fp)
        {
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
            TMR1=0;
            while(TMR1 < 100 * TMR1MSEC);
        }
        
        // open a file in write mode, for image data output from camera 2
        sprintf(fname,"AUT2_%03d.JPG",image_number);
        fp = fopenM(fname, "w");
        if (!fp)
        {
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
            TMR1=0;
            while(TMR1 < 100 * TMR1MSEC);
        }
        // unmount the SD card
        SD_umount();
    }
    return err;
}

