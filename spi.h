/* 
 * File:       spi.h
 * Author:     Peter Thornton
 * Purpose:    Support for SPI peripheral functions
 * Created on: 13 May 2020
 */

#define CS_SD   _RE5   // chip select for SD Card   , CSKB H1. 
#define CS_SFM  _RD13  // chip select for Serial Flash Memory (low = selected)
#define WP_SFM  _RD6   // write protect for Serial Flash Memory (low = protected)

long init_spi1(long fosc, long fsck);  // Initialize SPI1 with specified clock freq
long init_spi2(long fosc, long fsck);  // Initialize SPI2 with specified clock freq
long init_spi3(long fosc, long fsck);  // Initialize SPI3 with specified clock freq
int write_spi1(int data);              // one exchange through the shift register
int write_spi2(int data);              // one exchange through the shift register
int write_spi3(int data);              // one exchange through the shift register