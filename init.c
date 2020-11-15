/*
 * File:       init.c
 * Author:     Peter Thornton
 * Purpose:    Initialize all required PIC24F peripherals
 * Created on: 13 May 2020
 *  
*/

#include "xc.h"
#include "clock.h"
#include "uart.h"
#include "spi.h"
#include "i2c.h"
#include "adc.h"
#include "sfm.h"
#include "sd_test.h"
#include "rtc_user.h"
#include "datetime.h"
#include "eps_bat.h"
#include "pdt.h"
#include "init.h"


void init_peripherals(init_data_type *init_data_p)
{
    // Initialize the UART2 peripheral (He-100 transceiver or USB))
    init_data_p->u2br_actual = init_uart2(FOSC, init_data_p->u2br_request);
    
    // Initialize the SPI peripherals:
    // SPI1: SD-card interface (on MBM)
    // SPI2: Arducam interfaces (camera #1 and camera #2)
    // SPI3: Serial Flash Memory interface (on PPM)
    init_data_p->spi1_prescalar = init_spi1(FOSC, init_data_p->spi1_fsck);
    init_data_p->spi2_prescalar = init_spi2(FOSC, init_data_p->spi2_fsck);
    init_data_p->spi3_prescalar = init_spi3(FOSC, init_data_p->spi3_fsck);
    
    // initialize the I2C peripherals
    // I2C 1 used for: RTC, iMTQ, EPS, Bat, ANTS, Arducam1
    // I2C 2 used for: Arducam2
    init_data_p->brg1_reload = init_i2c1(FOSC, init_data_p->i2c1br);
    init_data_p->brg2_reload = init_i2c2(FOSC, init_data_p->i2c2br);
    
    // initialize the 10-bit ADC peripheral, for sun sensor data
    init_data_p->adc_iserror = init_adc();

    // wait one second to allow devices to reset after power-on
    long wait = 1000 * TMR1MSEC;
    TMR1 = 0;
    while (TMR1 < wait);
}

// initialize and test the hardware modules that are integrated on the
// Pumpkin motherboard module
void init_motherboard_components(init_data_type *init_data_p)
{
    // test the serial flash memory (Atmel AT25DF641)
    init_data_p->sfm_iserror = test_sfm();
    
    // test the SD card 
    init_data_p->sd_iserror = test_sd_write_read_delete();
    
    // test the real-time clock (M41T81S) and read the flag values
    init_data_p->rtc_flags_iserror = rtc_read_flags(&(init_data_p->rtc_flags));
    
    // if the RTC HALT flag is set, read the clock registers and save the
    // date and time of previous halt
    if ((init_data_p->rtc_flags >> 2) & 0x01)
    {
        get_isodatetime(init_data_p->rtc_halt_time);
    }
    // if any RTC flags are set, clear them, then read again
    if (init_data_p->rtc_flags)
    {
        init_data_p->rtc_clear_iserror = rtc_clear_flags(init_data_p->rtc_flags);
        init_data_p->rtc_flags2_iserror = rtc_read_flags(&(init_data_p->rtc_flags2));
    }
}

// perform the initial deployment test, and wait a set number of seconds if
// the MUST_WAIT flag is set in serial flash memory. 
void init_wait(init_data_type *init_data_p)
{
    int minutes_elapsed = 0;
    unsigned int counts_per_sec = 1000 * TMR1MSEC;
    unsigned int seconds_elapsed;
    
    // read the post-deployment timer wait flag from SFM
    init_data_p->pdt_flag = sfm_read_1byte(PDT_ADR1, PDT_ADR2, PDT_ADR3);
    
    if (init_data_p->pdt_flag == MUST_WAIT)
    {
        // begin post-deploy wait loop
        // wait in one-minute increments with EPS WDT reset each minute
        while (minutes_elapsed < PDT_NMIN)
        {
            // reset the seconds counter
            seconds_elapsed = 0;
            while (seconds_elapsed < 60)
            {
                TMR1 = 0;
                while(TMR1 < counts_per_sec);
                seconds_elapsed++;
            }
            minutes_elapsed++;
            // Reset EPS watchdog timer on each minute boundary
            eps_reset_watchdog();
        }
        // Completed the required PDT wait period, so write DONT_WAIT flag to SFM.
        // This allows code to skip the PDT wait period on any subsequent resets.
        sfm_write_1byte(PDT_ADR1, PDT_ADR2, PDT_ADR3, DONT_WAIT);
        // set status to indicate wait
        init_data_p->pdt_status = 1;
    }
    else if (init_data_p->pdt_flag == DONT_WAIT)
    {
        // set status to indicate didn't wait
        init_data_p->pdt_status = 0;
    }
    else
    {
        // set status to indicate unrecognized PDT flag value read from SFM
        init_data_p->pdt_status = 2;
    }
}
