/* 
 * File:    RamSat_MBM_test1_config.h
 * Author:  Peter Thornton
 * Purpose: Device settings for PIC24FJ256GB210
 * Created: 13 May 2020
 */

// This is a guard condition so that contents of this file are not included
// more than once.  
#ifndef XC_HEADER_TEMPLATE_H
#define	XC_HEADER_TEMPLATE_H

#include <xc.h> // include processor files - each processor file is guarded.  

// Configuration settings
#pragma config GCP = OFF         // memory protection: off
#pragma config GWRP = OFF        // program memory write protection: off
#pragma config FWDTEN = OFF      // watchdog timer: off
#pragma config JTAGEN = OFF      // JTAG port enabled: off
#pragma config IESO = OFF        // two-speed sstartup mode: off
#pragma config FCKSM = CSDCMD    // clock switching and clock monitor: disabled
#pragma config IOL1WAY = OFF     // allow multiple writes to the pin remapping
// Oscillator settings: Should produce Fosc = 32 MHz
#pragma config POSCMOD = XT      // Primary oscillator in XT mode (8MHz)
#pragma config FNOSC = PRIPLL    // Primary oscillator using 24x PLL
#pragma config PLL96MHZ = ON     // 96MHz PLL enabled
#pragma config PLLDIV = DIV2     // Divide primary oscillator input to PLL by 2

#endif	/* XC_HEADER_TEMPLATE_H */