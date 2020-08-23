/*
 * File:       rtc_user.c
 * Author:     Peter Thornton
 * Purpose:    User interface functions for the real-time clock
 * Created on: 6 August 2020
 *  
*/


#include "xc.h"
#include "rtc.h"
#include "clock.h"
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

// clear the HALT flag
int rtc_clearhalt()
{
    int err = 0;
    err = rtc_write_bit(HALT_REG, HALT_BIT, 0);
    return err;
}

// clear the STOP flag
int rtc_clearstop()
{
    int err = 0;
    err = rtc_write_bit(STOP_REG, STOP_BIT, 0);
    return err;
}

// clear the Oscillator Fail flag
int rtc_clearof()
{
    int err = 0;
    err = rtc_write_bit(OF_REG, OF_BIT, 0);
    return err;
}

// restart the oscillator (if OF flag is set any time other than first power up)
// first set and then immediately clear the STOP bit to restart oscillator
int rtc_restartosc()
{
    int err = 0;
    if ((err = rtc_write_bit(STOP_REG, STOP_BIT, 1)))
    {
        return err;
    }
    if ((err = rtc_write_bit(STOP_REG, STOP_BIT, 0)))
    {
        return err;
    }
    return err;    
}

// clear flags, based on current flag settings
int rtc_clear_flags(int flags)
{
    int err = 0;
    int halt_flag, stop_flag, of_flag;
    int i;
    long wait;          // Timer trigger

    
    halt_flag = ((flags >> 2) & 0x01);
    stop_flag = ((flags >> 1) & 0x01);
    of_flag   = (flags & 0x01);
    
    if (halt_flag)
    {
        // clear the halt flag to restart clock, return if err
        if ((err = rtc_clearhalt()))
        {
            return err;
        }
    }
    
    if (stop_flag)
    {
        // clear the stop flag, return if err
        if ((err = rtc_clearstop()))
        {
            return err;
        }
    }
    
    if (of_flag)
    {
        // restart the oscillator (toggles the STOP bit), return if err
        if ((err = rtc_restartosc()))
        {
            return err;
        }
        // wait 4 seconds, then clear the OF flag (per RTC datasheet)
        wait = 1000 * TMR1MSEC;
        for (i=0 ; i<4 ; i++)
        {
            TMR1 = 0;
            while (TMR1 < wait);
        }
        if ((err = rtc_clearof()))
        {
            return err;
        }
    }
    
    return err;
}