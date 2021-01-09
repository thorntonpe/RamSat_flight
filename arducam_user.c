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
