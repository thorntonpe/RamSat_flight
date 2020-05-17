/*
 * File:       uart.c
 * Author:     Peter Thornton
 * Purpose:    Initialize and use the UART2 peripheral (USB or He-100 transceiver)
 * Created on: 13 May 2020
 *  
*/


#include "xc.h"
#include "uart.h"
#include <math.h>
#include <stdio.h>

// Initialize UART2 peripheral, and connect to IO.6 (tx) and IO.7 (Rx)
// on the CSKB. This is used for serial connection to the He-100 
// transceiver or the USB interface on MBM (but not both at once)
long init_uart2(long fosc, long u2br)
{
    // long fosc = clock speed, Hz
    // long u2br = requested baud rate for UART2
    
    int brgh = 0;          // high baud rate generator flag (1=high speed)
    long fcy;              // frequency for instruction cycle
    long brg, brg2;        // calculated value for U2BRG register
    long br;               // calculated baud rate
    float br_err, br_err2; // calculated baud rate error
    float max_br_err = 0.02; // maximum allowable baud rate error (2%))
    
    // Set the Peripheral Pin Selections
    ANSF = 0;      // set all Port F pins to digital mode
    _TRISF5 = 0;   // set Port F, pin 5 (RP17) as output (U2TX)
    _TRISF4 = 1;   // set Port F, pin 4 (RP10) as input (U2RX)
    PORTFbits.RF5 = 1;  // initially set bit 5 high (is this needed? PET 3/2/2020)

    __builtin_write_OSCCONL( OSCCON & 0xbf); // clear bit 6 to unlock pin remap
    RPINR19bits.U2RXR = 10; // set the UART2 input function (U2RX) on pin RP10
    RPOR8bits.RP17R = 5;    // set pin RP17 as the UART2 output function (U2TX = 5)
    __builtin_write_OSCCONL( OSCCON | 0x40); // set bit 6 to lock pin remap
    
    // Set baud rate and initialize UART1 module
    // estimate a value for the baud rate generator register.
    // First pass assumes the normal mode, with 16 instruction cycles per bit.
    fcy = fosc / 2;
    brg = (fcy / (16 * u2br)) - 1;
    // calculate the actual baud rate and error compared to
    // requested baud rate. If error exceeds maximum allowable error,
    // bump baud rate generator index up or down by 1, depending on 
    // sign of error. Then compare new errors and select the lowest
    // absolute error as final value for BRG.
    br = fcy / (16 * (brg+1));
    br_err = (float)(br - u2br)/(float)(u2br);
    if (fabs(br_err) > max_br_err)
    {
        if (br_err > 0)
        {
            brg2 = brg+1;
        } else brg2 = brg-1;
        br = fcy / (16 * (brg2+1));
        br_err2 = (float)(br - u2br)/(float)(u2br);
        if (fabs(br_err) > fabs(br_err2))
        {
            brg = brg2;
            br_err = br_err2;
        }
        
        // if baud rate error is still too high, switch to high-speed
        // generator mode, recalculate assuming 4 instruction cycles
        // per bit.
        if (fabs(br_err) > max_br_err)
        {
            brgh = 1;
            brg = (fcy / (4 * u2br)) - 1;
            br = fcy / (4 * (brg+1));
            br_err = (float)(br - u2br)/(float)(u2br);
            // check for high error again
            if (fabs(br_err) > max_br_err)
            {
                if (br_err > 0)
                {
                    brg2 = brg+1;
                } else brg2 = brg-1;
                // recalculate baud rate and error
                br = fcy / (4 * (brg2+1));
                br_err2 = (float)(br - u2br)/(float)(u2br);
                // if new error is lower, use the new baud rate
                if (fabs(br_err) > fabs(br_err2))
                {
                    brg = brg2;
                    br_err = br_err2;
                }
            }
        }
    }
    // set the BRG register for UART2 module
    U2BRG = brg;
    // set the BRGH bit in U2MODE register
    U2MODE = 0x0000;
    U2MODEbits.BRGH = brgh;
    
    // set the UART2 operating mode:
    U2MODEbits.UEN = 0;    // Tx and Rx pins enabled
    U2MODEbits.PDSEL = 0;  // 8-bit data, no parity
    U2MODEbits.STSEL = 0;  // 1 stop bit
    U2MODEbits.UARTEN = 1; // UART2 is enabled
    
    // set the status register and enable UART2 Tx
    U2STA = 0x0000;
    U2STAbits.UTXEN = 1;   // enable transmitter, only after UARTEN is set
    
    // return the actual baud rate
    return br;
}

// write one character to UART2
void write_char2(int c)
{
    while (U2STAbits.UTXBF);      // wait if the transmit buffer is full)
    U2TXREG = c;
}   

// write a full string to UART2 (for terminal output)
void write_string2(char *s)
{
    // Write characters from the message string to the UART2 serial port
    char crlf[2]={13,10};
    while (*s)                  // send characters until the null terminator
        write_char2(*s++);       // send one character to USB port, advance pointer
    write_char2(crlf[0]);          // end with CR/LF sequence
    write_char2(crlf[1]);          // CR/LF
}

// read one character from UART2
unsigned char read_char2()
{
    unsigned char c;
    // Read a character from the serial port
    while ( !U2STAbits.URXDA);   // wait until there is a character in the receive buffer
    c = U2RXREG;                 // get the character
    return c;                  
}        

