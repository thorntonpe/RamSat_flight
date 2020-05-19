/*
 * File:       rtc.c
 * Author:     Peter Thornton
 * Purpose:    Functions to interface with the off-chip Real TIme Clock
 *             (Part number m41t81s, integrated on Pumpkin MBM)
 * Created on: 18 May 2020
*/

#include "xc.h"
#include "i2c.h"
#include "uart.h"  // temporary for test output to USB
#include "rtc.h"
#include <stdio.h> // temporary for output to USB

#define RTC_ADDR 0x68               // 7-bit address for RTC on I2C bus
#define RTC_DELAY 0                 // ms delay after RTC read or write command 

// common address values used by all commands
static int rtc_add_w = RTC_ADDR << 1;          // RTC address with write flag set (0)
static int rtc_add_r = (RTC_ADDR << 1) | 0b1;  // RTC address with read flag set (1)

// data array used to both read from and write to RTC registers
static unsigned char rtc_data[20];              // maximum size for any command

// write the specified number of bytes of data from rtc_data array to RTC,
// starting at the indicated byte
void rtc_write_data(int nbytes, unsigned char start_reg)
{
    int i;
    
    // ensure i2c bus is in idle state
    idle_i2c1();
    // initiate an i2c bus start, and wait for it to complete
    start_i2c1();
    // transmit device address byte with write flag
    transmit_i2c1(rtc_add_w);
    // transmit the address for first byte to write (0x00 - 0x13)
    transmit_i2c1(start_reg);
    // transmit the requested number of command bytes
    for (i=0 ; i<nbytes ; i++)
    {
        transmit_i2c1(rtc_data[i]);
    }
    // Initiate a stop and wait the specified time (ms)
    // the wait gives device time to prepare response
    stop_i2c1(RTC_DELAY);
}

// Read the specified number of bytes from the RTC device into rtc_data array.
// The device allows read to start from arbitrary position, but for now assume it
// starts in position 0x00.
void rtc_read_data(int nbytes)
{
    int i;
    int rcv;
    
    // ensure i2c bus is in idle state
    idle_i2c1();
    // initiate an i2c bus start, and wait for it to complete
    start_i2c1();
    // even though this is a read, begin with a write to send the starting register
    // transmit device address byte with write flag
    transmit_i2c1(rtc_add_w);
    transmit_i2c1(0x00);
    // send a new start signal to initiate the read operation
    start_i2c1();
    // transmit address byte with read flag
    transmit_i2c1(rtc_add_r);
    // receive the requested number of response bytes
    // receive with ack for first nbytes-1 bytes
    for (i=0 ; i<nbytes-1 ; i++)
    {
        receive_i2c1(&rcv);
        rtc_data[i] = rcv;
    }
    // receive with nack on last received byte
    receive_i2c1_nack(&rcv);
    rtc_data[i] = rcv;
    
    // initiate a stop and wait for it to complete
    stop_i2c1(RTC_DELAY);
}

// a test function that returns the first nbytes of data from RTC registers
void rtc_test_read(void)
{
    int i;
    int nbytes=20;
    char msg[128];      // character string for messages to user via COM port
    
    // read the first nbytes from RTC registers
    rtc_read_data(nbytes);
    // copy the local data into external array
    for (i=0 ; i<20 ; i++)
    {
        sprintf(msg,"RTC: reg %02d = 0x%02x",i, rtc_data[i]);
        write_string2(msg);
    }
}

// a test function to write 00 to the first two bytes of RTC register
// This should turn off the stop bit, allowing clock to run
void rtc_test_set(void)
{
    int nbytes=8;
    unsigned char start_reg;
    
    // first, clear the stop bit (reg 01.7)
    start_reg = 0x00;
    rtc_data[0]=0x00;   // decimal seconds (00-99)
    rtc_data[1]=0x00;   // seconds (0-59)  
    rtc_data[2]=0x05;   // minutes (0-59)
    rtc_data[3]=0x01;   // hour (00-23)
    rtc_data[4]=0x03;   // day of week (1-7)
    rtc_data[5]=0x19;   // day of month (1-31)
    rtc_data[6]=0x05;   // month (1-12)
    rtc_data[7]=0x20;   // year (00-99)
    rtc_write_data(nbytes, start_reg);
}

// clear the halt bit, which gets set if the RTC switches to battery power
void rtc_clearhalt(void)
{
    int nbytes = 1;
    unsigned char start_reg = 0x0c;
    rtc_data[0] = 0x00;
    rtc_write_data(nbytes, start_reg);
}

// clear the stop bit, which is set by default on first powerup
void rtc_clearstop(void)
{
    int nbytes = 1;
    unsigned char start_reg = 0x01;
    rtc_data[0] = 0x00;
    rtc_write_data(nbytes, start_reg);
}


// clear the oscillator fail bit, which is set by default on first powerup
void rtc_clearof(void)
{
    int nbytes = 1;
    unsigned char start_reg = 0x0f;
    rtc_data[0] = 0x00;
    rtc_write_data(nbytes, start_reg);
}