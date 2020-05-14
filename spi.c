/*
 * File:       spi.c
 * Author:     Peter Thornton
 * Purpose:    Functions to initialize and use the SPI peripherals
 * Created on: 13 May 2020
 */

// Initialization, pin mapping, and low-level I/O for SPI1, SPI2, and SPI3.
// SPI1: Interface for SD card on MBM
// SPI2: Interface for Arducam #1 and Arducam #2, on camera board
// SPI3: Interface for Serial Flash Memory on PPM

#include "xc.h"
#include "spi.h"

// SPI1 is used for SD card interface, hardwired on MBM
// This routine includes setting the ON_SD signal, to enable power to the SD 
// interface and power level-shifting chips.
long init_spi1(long fosc, long fsck)
{
    // given an input system clock frequency (fosc), and 
    // a desired SPI clock frequency (fsck), calculate the required set of
    // primary and/or secondary SPI clock prescalars
    long fcy = fosc/2;
    long total_prescalar = fcy / fsck;
    int pre1, pre2;
    switch (total_prescalar)
    {
        case 2:
            pre1 = 0b11;   // 1:1
            pre2 = 0b110;  // 2:1
            break;
        case 4:
            pre1 = 0b10;   // 4:1
            pre2 = 0b111;  // 1:1
            break;
        case 8:
            pre1 = 0b10;   // 4:1
            pre2 = 0b110;  // 2:1
            break;
        case 64:
            pre1 = 0b00;   // 64:1
            pre2 = 0b111;  // 1:1
            break;
        case 128:
            pre1 = 0b00;   // 64:1
            pre2 = 0b110;  // 2:1
            break;
        case 256:
            pre1 = 0b00;   // 64:1
            pre2 = 0b100;  // 4:1
            break;
        default: // set a low SPI clock speed
            pre1 = 0b01;   // 16:1
            pre2 = 0b110;  // 2:1
    }
    
    // Pin info:
    // function   = RP#   (Port) CSKB#  
    // SPI1-out   = RP15   (F8)  H1.23
    // SPI1-in    = RP24   (D1)  H1.22
    // SPI1-clock = RP23   (D2)  H1.21
    // CS-SD      =  NA    (E5)  H1.24 (see define statement in spi.h)
    // -ON_SD     =  NA    (E4)  NA    (active low, enables power to interface)
    
    // ensure that the parallel master port functionality is disabled
    _PMPEN = 0;

    // configure the Port D, E, and F pins as digital 
    // D port pins 1, 2, Port E pin 4, 5, and port F pin 8 are digital-only, so no ANS setting

    // Set the digital pins as input or output
    TRISDbits.TRISD2 = 0;  // SPI1 clock as output
    TRISDbits.TRISD1 = 1;  // SPI1 MISO as input
    TRISFbits.TRISF8 = 0;  // SPI1 MOSI as output
    TRISEbits.TRISE5 = 0;  // CS for SD card as output
    TRISEbits.TRISE4 = 0;  // -ON_SD as output
    
    // SD slot power off 
    _RE4 = 1;
    
    // use the peripheral pin selection function to assign SPI1 I/O
    // Note that SPI1 clock pin (RP23) needs to be mapped as both input and output
    __builtin_write_OSCCONL( OSCCON & 0xbf); // clear bit 6 to unlock pin remap
    RPINR20bits.SDI1R = 24;  // set the SPI1 data input function (sdi) on pin RP24
    RPINR20bits.SCK1R = 23;  // set the SPI1 clock input function (sck) on pin RP23
    RPOR7bits.RP15R   =  7;  // set pin RP15 as the SPI1 data output function (sdo)
    RPOR11bits.RP23R  =  8;  // set pin RP23 as the SPI1 clock output function (sck)
    __builtin_write_OSCCONL( OSCCON | 0x40); // set bit 6 to lock pin remap
    
    // Configure the special function registers for SPI1
    SPI1CON1bits.DISSCK = 0;     // using internal serial clock
    SPI1CON1bits.DISSDO = 0;     // using output pin
    SPI1CON1bits.MODE16 = 0;     // 8-bit data
    SPI1CON1bits.MSTEN  = 1;     // Master mode. This must be set before SMP.
    SPI1CON1bits.SMP    = 0;     // input data sampled at middle of data output time
    SPI1CON1bits.CKE    = 1;     // output changes on clock active to idle (CPHA=0)
    SPI1CON1bits.SSEN   = 0;     // Chip Select pin not controlled by peripheral
    SPI1CON1bits.CKP    = 0;     // Clock idles low, active high (CPOL=0)
    SPI1CON1bits.SPRE   = pre2;  // secondary prescalar for serial clock, Set above.
    SPI1CON1bits.PPRE   = pre1;  // primary prescalar for serial clock, set above.
    
    // set the chip select line high (deselected) before enabling device
    CS_SD = 1;
    
    // clear the receive overflow status and enable the SPI1 peripheral
    SPI1STATbits.SPIROV = 0;
    SPI1STATbits.SPIEN  = 1;    
    
    // return a metric of the actual SPI1 clock speed
    return total_prescalar;
}

// SPI2 is used for data interface to the ArduCams (camera #1 and camera #2))
long init_spi2(long fosc, long fsck)
{
    // given an input system clock frequency (fosc), and 
    // a desired SPI clock frequency (fsck), calculate the required set of
    // primary and/or secondary SPI clock prescalars
    long fcy = fosc/2;
    long total_prescalar = fcy / fsck;
    int pre1, pre2;
    switch (total_prescalar)
    {
        case 2:
            pre1 = 0b11;   // 1:1
            pre2 = 0b110;  // 2:1
            break;
        case 4:
            pre1 = 0b10;   // 4:1
            pre2 = 0b111;  // 1:1
            break;
        case 8:
            pre1 = 0b10;   // 4:1
            pre2 = 0b110;  // 2:1
            break;
        default: // set a low SPI clock speed
            pre1 = 0b01;   // 16:1
            pre2 = 0b111;  // 1:1
    }
    
    // Pin info:
    // function   = RP#   (Port) CSKB#  
    // SPI2-out   = RP22   (D3)  H1.7
    // SPI2-in    = RP2    (D8)  H1.8
    // SPI2-clock = RP25   (D4)  H1.5
    // CS-camera1 =  NA   (D12)  H1.6  (see define statement in spi.h)
    // CS-camera2 =  NA    (E7)  H1.1  (see define statement in spi.h)
    
    // ensure that the parallel master port functionality is disabled
    _PMPEN = 0;

    // configure the Port D pins as digital 
    // D port pins 3, 4, 8, 12 and port E pin 7 are digital-only, so no ANS setting

    // Set the digital pins as input or output
    TRISDbits.TRISD4 = 0;  // SPI2 clock as output
    TRISDbits.TRISD8 = 1;  // SPI2 MISO as input
    TRISDbits.TRISD3 = 0;  // SPI2 MOSI as output
    TRISDbits.TRISD12= 0;  // SS for camera #1 as output
    TRISEbits.TRISE7 = 0;  // SS for camera #2 as output

    // use the peripheral pin selection function to assign SPI2 I/O
    // Note that SPI2 clock pin (RP21) needs to be mapped as both input and output
    __builtin_write_OSCCONL( OSCCON & 0xbf); // clear bit 6 to unlock pin remap
    RPINR22bits.SDI2R = 2;   // set the SPI2 data input function (sdi) on pin RP2
    RPINR22bits.SCK2R = 25;  // set the SPI2 clock input function (sck) on pin RP25
    RPOR11bits.RP22R = 10;   // set pin RP22 as the SPI2 data output function (sdo)
    RPOR12bits.RP25R = 11;   // set pin RP25 as the SPI2 clock output function (sck)
    __builtin_write_OSCCONL( OSCCON | 0x40); // set bit 6 to lock pin remap
    
    // Configure the special function registers for SPI2
    SPI2CON1bits.DISSCK = 0;     // using internal serial clock
    SPI2CON1bits.DISSDO = 0;     // using output pin
    SPI2CON1bits.MODE16 = 0;     // 8-bit data
    SPI2CON1bits.MSTEN  = 1;     // Master mode. This must be set before SMP.
    SPI2CON1bits.SMP    = 0;     // input data sampled at middle of data output time
    SPI2CON1bits.CKE    = 1;     // output changes on clock active to idle (CPHA=0)
    SPI2CON1bits.SSEN   = 0;     // Chip select pin not controlled by peripheral
    SPI2CON1bits.CKP    = 0;     // Clock idles low, active high (CPOL=0)
    SPI2CON1bits.SPRE   = pre2;  // secondary prescalar for serial clock, Set above.
    SPI2CON1bits.PPRE   = pre1;  // primary prescalar for serial clock, set above.
    
    // set the chip select lines high (deselected) before enabling device
    CS_CAM1 = 1;
    CS_CAM2 = 1;
    
    // clear the receive overflow status and enable the SPI2 peripheral
    SPI2STATbits.SPIROV = 0;
    SPI2STATbits.SPIEN  = 1;    
    
    // return a metric of the actual SPI2 clock speed
    return total_prescalar;
}

// SPI3 is used for data interface to the Serial Flash Memory on PPM
long init_spi3(long fosc, long fsck)
{
    // given an input system clock frequency (fosc), and 
    // a desired SPI clock frequency (fsck), calculate the required set of
    // primary and/or secondary SPI clock prescalars
    long fcy = fosc/2;
    long total_prescalar = fcy / fsck;
    int pre1, pre2;
    switch (total_prescalar)
    {
        case 2:
            pre1 = 0b11;   // 1:1
            pre2 = 0b110;  // 2:1
            break;
        case 4:
            pre1 = 0b10;   // 4:1
            pre2 = 0b111;  // 1:1
            break;
        case 8:
            pre1 = 0b10;   // 4:1
            pre2 = 0b110;  // 2:1
            break;
        default: // set a low SPI clock speed
            pre1 = 0b01;   // 16:1
            pre2 = 0b111;  // 1:1
    }
    
    // Pin info:
    // function   = RP#   (Port) CSKB#  
    // SPI3-out   = RP11   (D0)  --
    // SPI3-in    = RP3   (D10)  --
    // SPI3-clock = RP12  (D11)  --
    // CS-SFM     = NA    (D13)  --   (see define statement in spi.h)
    // WP-SFM     = NA     (D6)  --   (see define statement in spi.h)
    
    // ensure that the parallel master port functionality is disabled
    _PMPEN = 0;

    // configure the Port pins as digital 
    // Only D6 has an ANS setting, the others are digital-only
    ANSDbits.ANSD6 = 0;    // D6 as digital

    // Set the digital pins as input or output
    TRISDbits.TRISD11 = 0;  // SPI3 clock as output
    TRISDbits.TRISD10 = 1;  // SPI3 MISO as input
    TRISDbits.TRISD0  = 0;  // SPI3 MOSI as output
    TRISDbits.TRISD13 = 0;  // CS for Serial Flash Memory as output
    TRISDbits.TRISD6  = 0;  // WP for Serial Flash Memory as output

    // use the peripheral pin selection function to assign SPI3 I/O
    // Note that SPI3 clock pin (RP12) needs to be mapped as both input and output
    __builtin_write_OSCCONL( OSCCON & 0xbf); // clear bit 6 to unlock pin remap
    RPINR28bits.SDI3R = 3;   // set the SPI3 data input function (sdi) on pin RP3
    RPINR28bits.SCK3R = 12;  // set the SPI3 clock input function (sck) on pin RP12
    RPOR5bits.RP11R   = 32;  // set pin RP11 as the SPI3 data output function (sdo)
    RPOR6bits.RP12R   = 33;  // set pin RP12 as the SPI3 clock output function (sck)
    __builtin_write_OSCCONL( OSCCON | 0x40); // set bit 6 to lock pin remap
    
    // Configure the special function registers for SPI3
    SPI3CON1bits.DISSCK = 0;     // using internal serial clock
    SPI3CON1bits.DISSDO = 0;     // using output pin
    SPI3CON1bits.MODE16 = 0;     // 8-bit data
    SPI3CON1bits.MSTEN  = 1;     // Master mode. This must be set before SMP.
    SPI3CON1bits.SMP    = 0;     // input data sampled at middle of data output time
    SPI3CON1bits.CKE    = 1;     // output changes on clock active to idle (CPHA=0)
    SPI3CON1bits.SSEN   = 0;     // Chip select pin not controlled by peripheral
    SPI3CON1bits.CKP    = 0;     // Clock idles low, active high (CPOL=0)
    SPI3CON1bits.SPRE   = pre2;  // secondary prescalar for serial clock, Set above.
    SPI3CON1bits.PPRE   = pre1;  // primary prescalar for serial clock, set above.
    
    // set the slave select line(s) high (deselected) before enabling device
    CS_SFM = 1;   // deselect the SFM
    WP_SFM = 1;   // deassert the write protect line 
    
    // clear the receive overflow status and enable the SPI2 peripheral
    SPI3STATbits.SPIROV = 0;
    SPI3STATbits.SPIEN  = 1;    
    
    // return a metric of the actual SPI clock speed
    return total_prescalar;
}

int write_spi1(int data)
{
    SPI1BUF = data;     // write to buffer for TX
    while (!SPI1STATbits.SPIRBF);
    return SPI1BUF;
}

int write_spi2(int data)
{
    SPI2BUF = data;     // write to buffer for TX
    while (!SPI2STATbits.SPIRBF);
    return SPI2BUF;
}

int write_spi3(int data)
{
    SPI3BUF = data;     // write to buffer for TX
    while (!SPI3STATbits.SPIRBF);
    return SPI3BUF;
}