/*
 * File:       i2c.c
 * Author:     Peter Thornton
 * Purpose:    Functions to initialize and use the I2C peripheral
 * Created on: 18 May 2020
 */


#include "xc.h"
#include "clock.h"
#include "i2c.h"

void idle_i2c1(void)
{
    // wait until start, stop, receive, acknowledge, and transmit operations
    // are all clear (bit value = 0)
    while(I2C1CONbits.SEN | I2C1CONbits.RSEN | I2C1CONbits.PEN | I2C1CONbits.RCEN | 
            I2C1CONbits.ACKEN | I2C1STATbits.TRSTAT);
}

void idle_i2c2(void)
{
    // wait until start, stop, receive, acknowledge, and transmit operations
    // are all clear (bit value = 0)
    while(I2C2CONbits.SEN | I2C2CONbits.RSEN | I2C2CONbits.PEN | I2C2CONbits.RCEN | 
            I2C2CONbits.ACKEN | I2C2STATbits.TRSTAT);
}

void start_i2c1(void)
{
    I2C1CONbits.SEN = 1;
    while (I2C1CONbits.SEN);
}

void start_i2c2(void)
{
    I2C2CONbits.SEN = 1;
    while (I2C2CONbits.SEN);
}

void restart_i2c1(void)
{
    I2C1CONbits.RSEN = 1;
    while (I2C1CONbits.RSEN);
}

void restart_i2c2(void)
{
    I2C2CONbits.RSEN = 1;
    while (I2C2CONbits.RSEN);
}

void stop_i2c1(int delay_msec)
{
    // delay_msec = delay in milliseconds after the stop
    // completes. set according to device requirements.
    // (delay_msec = 0 for no delay)
    
    int delay = delay_msec * DELAYMSEC;
    
    I2C1CONbits.PEN = 1;
    while (I2C1CONbits.PEN);
    
    // wait the specified number of milliseconds, allow device to update state
    TMR1 = 0;
    while (TMR1 <= delay);
}

void stop_i2c2(int delay_msec)
{
    // delay_msec = delay in milliseconds after the stop
    // completes. set according to device requirements.
    // (delay_msec = 0 for no delay)
    
    int delay = delay_msec * DELAYMSEC;
    
    I2C2CONbits.PEN = 1;
    while (I2C2CONbits.PEN);
    
    // wait the specified number of milliseconds, allow device to update state
    TMR1 = 0;
    while (TMR1 <= delay);
}

void transmit_i2c1(int data)
{
    do
    {
        I2C1TRN = data;
        while (I2C1STATbits.TRSTAT);
    } while (I2C1STATbits.ACKSTAT);
}

void transmit_i2c2(int data)
{
    do
    {
        I2C2TRN = data;
        while (I2C2STATbits.TRSTAT);
    } while (I2C2STATbits.ACKSTAT);
}

// a version of transmit that does not wait for ack from device
void transmit_i2c1_noack(int data)
{
    I2C1TRN = data;
    while (I2C1STATbits.TRSTAT);
}

// a version of transmit that does not wait for ack from device
void transmit_i2c2_noack(int data)
{
    I2C2TRN = data;
    while (I2C2STATbits.TRSTAT);
}

void ack_i2c1(void)
{
    I2C1CONbits.ACKDT = 0;
    I2C1CONbits.ACKEN = 1;
    while (I2C1CONbits.ACKEN);
}

void ack_i2c2(void)
{
    I2C2CONbits.ACKDT = 0;
    I2C2CONbits.ACKEN = 1;
    while (I2C2CONbits.ACKEN);
}

void nack_i2c1(void)
{
    I2C1CONbits.ACKDT = 1;
    I2C1CONbits.ACKEN = 1;
    while (I2C1CONbits.ACKEN);
}

void nack_i2c2(void)
{
    I2C2CONbits.ACKDT = 1;
    I2C2CONbits.ACKEN = 1;
    while (I2C2CONbits.ACKEN);
}

void receive_i2c1(int *data)
{
    I2C1CONbits.RCEN = 1;
    while (I2C1CONbits.RCEN);
    *data = I2C1RCV;
    // send ACK for this receive
    ack_i2c1();
}

void receive_i2c2(int *data)
{
    I2C2CONbits.RCEN = 1;
    while (I2C2CONbits.RCEN);
    *data = I2C2RCV;
    // send ACK for this receive
    ack_i2c2();
}

void receive_i2c1_nack(int *data)
{
    I2C1CONbits.RCEN = 1;
    while (I2C1CONbits.RCEN);
    *data = I2C1RCV;
    // send NACK for this receive
    nack_i2c1();
}

void receive_i2c2_nack(int *data)
{
    I2C2CONbits.RCEN = 1;
    while (I2C2CONbits.RCEN);
    *data = I2C2RCV;
    // send NACK for this receive
    nack_i2c2();
}

long init_i2c1(long fosc, long i2c1br)
{
    // long fosc = clock rate (Hz)
    // long i2c1br = intended baud rate for the I2C_1 clock

    long fcy;              // frequency for instruction cycle (Hz))
    long brg_reload;       // the calculated baud rate generator reload value 
    
    // disable the peripheral
    I2C1CONbits.I2CEN = 0;
    
    // both pins (Port A 14, 15) need to be set as inputs
    TRISAbits.TRISA14 = 1;  
    TRISAbits.TRISA15 = 1;  
    // set the Port A pins <14-15> to Open-drain configuration
    ODCAbits.ODA14 = 1;
    ODCAbits.ODA15 = 1;
    
    // calculate the baud rate generator reload value, based on eq 16-1 in the
    // PIC24F device data sheet. Set the special value register. This value is also
    // returned for diagnostic output.
    fcy = fosc/2;
    brg_reload = ((fcy / i2c1br) - (fcy / 10000000) - 1);
    I2C1BRG = brg_reload;
    
    // set any necessary I2C control register values
    I2C1CONbits.DISSLW = 1;   // slew control not needed for 100 kHz baud rate
    
    // enable the peripheral and idle until it enters ready state
    I2C1CONbits.I2CEN = 1;
    
    return brg_reload;
}

long init_i2c2(long fosc, long i2c2br)
{
    // long fosc = clock rate (Hz)
    // long i2c2br = intended baud rate for the I2C_2 clock

    long fcy;              // frequency for instruction cycle (Hz))
    long brg_reload;       // the calculated baud rate generator reload value 
    
    // disable the peripheral
    I2C2CONbits.I2CEN = 0;
    
    // both pins (Port A 2, 3) need to be set as inputs
    TRISAbits.TRISA2 = 1;  
    TRISAbits.TRISA3 = 1;  
    // set the Port A pins 2 and 3 to Open-drain configuration
    ODCAbits.ODA2 = 1;
    ODCAbits.ODA3 = 1;
    
    // calculate the baud rate generator reload value, based on eq 16-1 in the
    // PIC24F device data sheet. Set the special value register. This value is also
    // returned for diagnostic output.
    fcy = fosc/2;
    brg_reload = ((fcy / i2c2br) - (fcy / 10000000) - 1);
    I2C2BRG = brg_reload;
    
    // set any necessary I2C control register values
    I2C2CONbits.DISSLW = 1;   // slew control not needed for 100 kHz baud rate
    
    // enable the peripheral and idle until it enters ready state
    I2C2CONbits.I2CEN = 1;
    
    return brg_reload;
}


