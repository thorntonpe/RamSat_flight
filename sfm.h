/* 
 * File:       sfm.h
 * Author:     Peter Thornton
 * Purpose:    Prototypes for Serial Flash Memory routines
 * Created on: 13 May 2020
 */

int sfm_read_1byte(int adr1, int adr2, int adr3);
void sfm_write_1byte(int adr1, int adr2, int adr3, int data);
void sfm_erase_4k(int adr1, int adr2, int adr3);
void sfm_erase_64k(int sector);
void sfm_write_page(int sector, int page, char* data, int nbytes);
void sfm_read_page(int sector, int page, int* data);
int test_sfm(void);
