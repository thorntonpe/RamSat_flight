/* 
 * File:       sfm.h
 * Author:     Peter Thornton
 * Purpose:    Prototypes for Serial Flash Memory routines
 * Created on: 13 May 2020
 */

int test_sfm(void);
void clear_pdt_flag(void);
int wait_pdt(unsigned int n_secs, int *out_flag);