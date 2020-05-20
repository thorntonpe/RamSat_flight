/* 
 * File:        rtc.h
 * Author:      Peter Thornton 
 * Purpose:     Public interface for rtc routines
 * Created on:  18 May 2020
 */

int rtc_read_data(int firstreg, int nbytes, unsigned char *out);
void rtc_test_read(void);
void rtc_test_set(void);
void rtc_clearhalt(void);
void rtc_clearstop(void);
void rtc_clearof(void);
