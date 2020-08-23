/*
 * File:       rtc.c
 * Author:     Peter Thornton
 * Purpose:    Low-level functions to interface with the off-chip Real TIme Clock
 *             (Part number m41t81s, integrated on Pumpkin MBM)
 * Created on: 18 May 2020
*/

#include "xc.h"
#include "i2c.h"
#include "rtc.h"

#define RTC_ADDR 0x68               // 7-bit address for RTC on I2C bus
#define RTC_DELAY 0                 // ms delay after RTC read or write command 

// common address values used by all commands
static int rtc_add_w = RTC_ADDR << 1;          // RTC address with write flag set (0)
static int rtc_add_r = (RTC_ADDR << 1) | 0b1;  // RTC address with read flag set (1)

// Write the specified number of bytes of data from in array to RTC,
// starting at start_reg register. The data to write are assumed to start 
// at position 0 in input array.
// Returns 1 for error, 0 for no error.
int rtc_write_nbytes(int nbytes, unsigned char firstreg, unsigned char *in)
{
    int err = 0;
    int i;
    
    // make sure the input parameters are in valid range for RTC
    if ((firstreg > 0x13) || (firstreg+nbytes > 0x14))
    {
        err = 1;
        return err;
    }
    
    // ensure i2c bus is in idle state
    idle_i2c1();
    // initiate an i2c bus start, and wait for it to complete
    start_i2c1();
    // transmit device address byte with write flag
    transmit_i2c1(rtc_add_w);
    // transmit the address for first byte to write (0x00 - 0x13)
    transmit_i2c1(firstreg);
    // transmit the requested number of command bytes
    for (i=0 ; i<nbytes ; i++)
    {
        transmit_i2c1(*in++);
    }
    // Initiate a stop and wait the specified time (ms)
    // the wait gives device time to prepare response
    stop_i2c1(RTC_DELAY);
    
    return err;
}

// Read the specified number of bytes from the RTC device into output array.
// The device allows read to start from arbitrary position, given as firstreg.
// Returns 1 for error, 0 for no error.
int rtc_read_nbytes(int nbytes, unsigned char firstreg, unsigned char *out)
{
    int err = 0;
    int i;
    int rcv;
    
    // make sure the input parameters are in valid range for RTC
    if ((firstreg > 0x13) || (firstreg+nbytes > 0x14))
    {
        err = 1;
        return err;
    }
    
    // ensure i2c bus is in idle state
    idle_i2c1();
    // initiate an i2c bus start, and wait for it to complete
    start_i2c1();
    // even though this is a read, begin with a write to send the starting register
    // transmit device address byte with write flag
    transmit_i2c1(rtc_add_w);
    transmit_i2c1(firstreg);
    // send a new start signal to initiate the read operation
    start_i2c1();
    // transmit address byte with read flag
    transmit_i2c1(rtc_add_r);
    // receive the requested number of response bytes
    // receive with ack for first nbytes-1 bytes
    for (i=0 ; i<nbytes-1 ; i++)
    {
        receive_i2c1(&rcv);
        *out++ = rcv;
    }
    // receive with nack on last received byte
    receive_i2c1_nack(&rcv);
    *out = rcv;
    
    // initiate a stop and wait for it to complete
    stop_i2c1(RTC_DELAY);
    
    return err;
}

// Set a single bit by first reading the register, then, if necessary, changing the
// specified bit and writing the register. bitn is the bit position (0-7). bitval is
// the desired bit value.
int rtc_write_bit(unsigned char reg, int bitn, int bitval)
{
    int err = 0;
    unsigned char regval;
    
    // check that bit position and bit value are valid
    if (bitn < 0 || bitn > 7 || bitval < 0 || bitval > 1)
    {
        err = 1;
        return err;
    }
    
    // read the register and return if error
    err = rtc_read_nbytes(1, reg, &regval);
    if (err)
    {
        return err;
    }
    
    
    // set or clear bit
    if (bitval)  // set bit
    {
        regval |= (1 << bitn);
    }
    else         // clear bit
    {
        regval &= ~(1 << bitn);
    }
    err = rtc_write_nbytes(1, reg, &regval);

    return err;
}

// read a single bit from a specified register. bitn is the bit position (0-7).
// bitval is the returned bit value.
int rtc_read_bit(unsigned char reg, int bitn, int *bitval)
{
    int err = 0;
    unsigned char regval;
    
    // check that bit position and bit value is valid
    if (bitn < 0 || bitn > 7)
    {
        err = 1;
        return err;
    }
    
    // read the register and return if error
    err = rtc_read_nbytes(1, reg, &regval);
    if (err)
    {
        return err;
    }
    
    // set the return bit value
    *bitval = (regval >> bitn) & 0x01;
    
    return err;
}