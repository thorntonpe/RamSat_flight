/* 
 * File:       .h
 * Author:     Peter Thornton
 * Purpose:  
 * Created on:  
 */

// function prototypes 
int rtc_read_flags(int *flags);
int rtc_clearhalt();
int rtc_clearstop();
int rtc_clearof();
int rtc_restartosc();
int rtc_clear_flags(int flags);
