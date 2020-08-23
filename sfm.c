/*
 * File:       sfm.c
 * Author:     Peter Thornton
 * Purpose:    Functions test and use the integrated Serial Flash Memory (SFM)
 * Created on: 13 May 2020
 */

// Note: the chip-select and write-protect pins are defined in spi.h

#include "xc.h"
#include "clock.h"
#include "spi.h"  // low-level function prototypes and pin definitions for SPI3
#include "sfm.h"

// some basic SFM commands for AT25DF641
#define SFM_WRSR    0x01   // write status register
#define SFM_WRITE   0x02   // write command
#define SFM_READ    0x03   // read command
#define SFM_WDI     0x04   // write disable
#define SFM_RDSR    0x05   // read status register
#define SFM_WEN     0x06   // write enable
#define SFM_SECTUNP 0x39   // 64Kbyte sector unprotect (followed by 3-byte sector address)
#define SFM_SECTPRO 0x36   // 64Kbyte sector protect (followed by 3-byte sector address)
#define SFM_SECTRDR 0x3c   // read the sector protect register for following address
#define SFM_ER4K    0x20   // Erase a 4 KByte block at following address
#define SFM_ER32K   0x52   // Erase a 32 Kbyte block at following address
#define SFM_ER64K   0xd8   // Erase a 64 Kbyte block at following address

// address and flag values for the post-deployment timer
#define PDT_ADDR1 0x00     // Address on SFM for post-deploy timer flag:
#define PDT_ADDR2 0x10     // (00, 10, 00) is at the start of the second
#define PDT_ADDR3 0x00     // 4k block in sector 1.
#define MUST_WAIT 0x55     // Flag value: Post-deploy timer has not yet completed
#define DONT_WAIT 0xaa     // Flag value: Post-deploy timer has already completed

int ReadSR( void)
{
    // read the SFM status register and return its value
    int srbyte1;
    CS_SFM = 0;
    write_spi3(SFM_RDSR);
    srbyte1 = write_spi3(0);
    CS_SFM = 1;
    return srbyte1;
}

int test_sfm(void) 
{
    // do a test write and read operation to the SFM
    // return is_error = 0 (correct write/read), 1 (incorrect write/read)

    // set the 24-bit start address for write/read test
    int addr1 = 0x00;
    int addr2 = 0x00;
    int addr3 = 0x00;
        
    // before any data can be written to the chip, the sector protection
    // must be turned off for the target sector (turned on by default at power-up)
    // The device determines the correct sector based on 24-bit address sent
    // send write-enable command
    CS_SFM = 0;             // select the SFM
    write_spi3(SFM_WEN);    // send write enable command
    CS_SFM = 1;               // deselect, terminate command
    // send sector unprotect command
    CS_SFM = 0;
    write_spi3(SFM_SECTUNP);
    write_spi3(addr1);
    write_spi3(addr2);
    write_spi3(addr3);
    CS_SFM = 1;

    // Erase the first 4 KByte block in this sector, for write testing
    // The device determines the correct block based on 24-bit address sent
    // send write-enable command
    CS_SFM = 0;
    write_spi3(SFM_WEN);
    CS_SFM = 1;
    // send block erase command
    CS_SFM = 0;
    write_spi3(SFM_ER4K);
    write_spi3(addr1);
    write_spi3(addr2);
    write_spi3(addr3);
    CS_SFM = 1;
    // wait for the write operation to complete by monitoring bit 0 of the SR
    while (ReadSR() & 0x1);

    // Write a few bytes of test data starting at defined address
    // set test values for data bytes (use any non-zero values)
    int write_data1 = 0xaa;
    int write_data2 = 0xbb;
    int write_data3 = 0xcc;
    // send the write enable command
    CS_SFM = 0;
    write_spi3(SFM_WEN);
    CS_SFM = 1;
    // send the write command, address, and data
    CS_SFM = 0;
    write_spi3(SFM_WRITE);
    write_spi3(addr1);
    write_spi3(addr2);
    write_spi3(addr3);
    write_spi3(write_data1);
    write_spi3(write_data2);
    write_spi3(write_data3);
    CS_SFM = 1;
    // wait for the write operation to complete by monitoring bit 0 of the SR
    while (ReadSR() & 0x1);

    // Read back these bytes of data
    int read_data1 = 0;
    int read_data2 = 0;
    int read_data3 = 0;
    CS_SFM = 0;
    write_spi3(SFM_READ);
    write_spi3(addr1);
    write_spi3(addr2);
    write_spi3(addr3);
    read_data1 = write_spi3(0);
    read_data2 = write_spi3(0);
    read_data3 = write_spi3(0);
    CS_SFM = 1;
    
    // compare the write data with the read data, and return
    int is_error = (!(read_data1==write_data1 && read_data2==write_data2 &&
            read_data3==write_data3));

    return is_error;
}

// set the post-deploy timer flag to pre-deployed configuration
void clear_pdt_flag(void) 
{
    // write MUST_WAIT to the post-deploy timer flag address on SFM
    
    // before any data can be written to the chip, the sector protection
    // must be turned off for the target sector (turned on by default at power-up)
    // The device determines the correct sector based on 24-bit address sent
    // send write-enable command
    CS_SFM = 0;             // select the SFM
    write_spi3(SFM_WEN);    // send write enable command
    CS_SFM = 1;               // deselect, terminate command
    // send sector unprotect command
    CS_SFM = 0;
    write_spi3(SFM_SECTUNP);
    write_spi3(PDT_ADDR1);
    write_spi3(PDT_ADDR2);
    write_spi3(PDT_ADDR3);
    CS_SFM = 1;

    // Erase the designated 4 KByte block 
    // send write-enable command
    CS_SFM = 0;
    write_spi3(SFM_WEN);
    CS_SFM = 1;
    // send block erase command
    CS_SFM = 0;
    write_spi3(SFM_ER4K);
    write_spi3(PDT_ADDR1);
    write_spi3(PDT_ADDR2);
    write_spi3(PDT_ADDR3);
    CS_SFM = 1;
    // wait for the write operation to complete by monitoring bit 0 of the SR
    while (ReadSR() & 0x1);

    // Write the MUST_WAIT value to deploy timer flag address
    // send the write enable command
    CS_SFM = 0;
    write_spi3(SFM_WEN);
    CS_SFM = 1;
    // send the write command, address, and data
    CS_SFM = 0;
    write_spi3(SFM_WRITE);
    write_spi3(PDT_ADDR1);
    write_spi3(PDT_ADDR2);
    write_spi3(PDT_ADDR3);
    write_spi3(MUST_WAIT);
    CS_SFM = 1;
    // wait for the write operation to complete by monitoring bit 0 of the SR
    while (ReadSR() & 0x1);
}

int wait_pdt(unsigned int n_secs, int *out_flag)
{
    int status;
    unsigned int counts_per_sec = 1000 * TMR1MSEC;
    unsigned int seconds_waited = 0;
    
    // read the post-deploy timer flag from SFM
    // Read back these bytes of data
    int pdt_flag = 0;
    CS_SFM = 0;
    write_spi3(SFM_READ);
    write_spi3(PDT_ADDR1);
    write_spi3(PDT_ADDR2);
    write_spi3(PDT_ADDR3);
    pdt_flag = write_spi3(0);
    CS_SFM = 1;
    
    switch(pdt_flag)
    {
        case MUST_WAIT:  
            // pft wait period has not been satisfied, 
            // so wait the requred number of seconds
            while (seconds_waited < n_secs)
            {
                TMR1 = 0;
                while(TMR1 < counts_per_sec);
                seconds_waited++;
            }
            // turn off the write protect flag for sector
            CS_SFM = 0;             // select the SFM
            write_spi3(SFM_WEN);    // send write enable command
            CS_SFM = 1;               // deselect, terminate command
            // send sector unprotect command
            CS_SFM = 0;
            write_spi3(SFM_SECTUNP);
            write_spi3(PDT_ADDR1);
            write_spi3(PDT_ADDR2);
            write_spi3(PDT_ADDR3);
            CS_SFM = 1;
            // wait for the unprotect operation to complete by monitoring bit 0 of the SR
            while (ReadSR() & 0x1);
            // Erase the designated 4 KByte block (must be erased before writing)
            // send write-enable command
            CS_SFM = 0;
            write_spi3(SFM_WEN);
            CS_SFM = 1;
            // send block erase command
            CS_SFM = 0;
            write_spi3(SFM_ER4K);
            write_spi3(PDT_ADDR1);
            write_spi3(PDT_ADDR2);
            write_spi3(PDT_ADDR3);
            CS_SFM = 1;
            // wait for the erase operation to complete by monitoring bit 0 of the SR
            while (ReadSR() & 0x1);
            // write the DONT_WAIT flag to SFM, to prevent wait on next restart
            // send the write enable command
            CS_SFM = 0;
            write_spi3(SFM_WEN);
            CS_SFM = 1;
            // send the write command, address, and data
            CS_SFM = 0;
            write_spi3(SFM_WRITE);
            write_spi3(PDT_ADDR1);
            write_spi3(PDT_ADDR2);
            write_spi3(PDT_ADDR3);
            write_spi3(DONT_WAIT);
            CS_SFM = 1;
            // wait for the write operation to complete by monitoring bit 0 of the SR
            while (ReadSR() & 0x1);
            status = 1;  // return status: waited
            break;
            
        case DONT_WAIT:
            status = 0;  // return status: didn't wait
            break;
            
        default:
            status = 2;  // return status: unrecognized pdt_flag value
    } // end switch on pdt_flag
    
    *out_flag = pdt_flag;
    return status;
}
