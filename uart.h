/* 
 * File:     uart.h
 * Author:   Peter Thornton
 * Purpose:  Initialize and use the UART peripherals
 * Created:  2 March 2020
 */

// function prototypes 
long init_uart2(long fosc, long u2br);
void write_char2(int c);
void write_string2(char *s);
unsigned char read_char2();








