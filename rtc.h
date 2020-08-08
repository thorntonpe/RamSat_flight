/* 
 * File:        rtc.h
 * Author:      Peter Thornton 
 * Purpose:     Low-level routines for the real-time clock
 * Created on:  18 May 2020
 */

int rtc_write_nbytes(int nbytes, unsigned char firstreg, unsigned char *in);
int rtc_read_nbytes(int nbytes, unsigned char firstreg, unsigned char *out);
int rtc_write_bit(unsigned char reg, int bitn, int bitval);
int rtc_read_bit(unsigned char reg, int bitn, int *bitval);
