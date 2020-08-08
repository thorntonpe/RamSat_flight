/*
 * File:       rtc_user.c
 * Author:     Peter Thornton
 * Purpose:    User interface functions for the real-time clock
 * Created on: 6 August 2020
 *  
*/


#include "xc.h"
#include "rtc.h"
#include "rtc_user.h"

#define HALT_REG 0x0c
#define HALT_BIT 6
#define STOP_REG 0x01
#define STOP_BIT 7
#define OF_REG 0x0f
#define OF_BIT 2

// read the oscillator flags and store as the three low bytes of flags
// byte2 = STOP, byte1 = HALT, byte0 = OF (oscillator fail)
int rtc_read_flags(int *flags)
{
    int err = 0;
    int read_bit;
    int temp_flags = 0;
    
    // start by setting the flags values to 0
    *flags = 0;
    
    // read the HALT, STOP, and OF bits
    // set the value of HALT in temp_flags, byte 2
    err = rtc_read_bit(HALT_REG, HALT_BIT, &read_bit);
    if (err)
    {
        return err;
    }
    temp_flags = (read_bit << 2);
    
    // set the value of STOP in temp_flags, byte 1
    err = rtc_read_bit(STOP_REG, STOP_BIT, &read_bit);
    if (err)
    {
        return err;
    }
    if (read_bit)
    {
       temp_flags |= (read_bit << 1); 
    }
    else
    {
        temp_flags &= ~(read_bit << 1);
    }
    
    // set the value of OF in temp_flags, byte 0
    err = rtc_read_bit(OF_REG, OF_BIT, &read_bit);
    if (err)
    {
        return err;
    }
    if (read_bit)
    {
       temp_flags |= (read_bit); 
    }
    else
    {
        temp_flags &= ~(read_bit);
    }
    
    *flags = temp_flags;
    
    return err;
}