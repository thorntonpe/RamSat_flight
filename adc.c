/*
 * File:       adc.c
 * Author:     Peter Thornton
 * Purpose:    Functionality for 10-bit ADC peripheral on PIC24F
 * Created on: 24 May 2020
 *  
*/

#include "xc.h"
#include "adc.h"

int init_adc(void)
{
    int err = 0;
    
    // initialize the AD1CON1 register settings
    // AD1CON1
    _ADON = 0;     // disable module while configuring
    _ADSIDL = 1;   // stop the module while device is in idle mode
    _FORM = 0b00;  // output format as unsigned int (0000 00dd dddd dddd)
    _SSRC = 0b111; // auto conversion triggered after time set by SAMC
    _ASAM = 0;     // auto-sampling turned off
    _SAMP = 0;     // after initialization, setting SAMP to 1 starts sampling
    // AD1CON2
    _VCFG = 0;     // use internal V_DD and V_SS as reference voltages (3.3V, GND)
    _CSCNA = 1;    // use single channel (CH0SA) for MUX A input if clear, scan if set
    _SMPI = 7;     // interrupt after 7th sample/convert
    _BUFM = 0;     // 0 = buffer configured as one 16-word buffer (0 to FF)
    _ALTS = 0;     // always use MUX A input multiplexer settings
    // AD1CON3
    _ADRC = 0;     // Clock derived from system clock
    _SAMC = 6;     // Auto-sample time = 6 * T_AD
    _ADCS = 1;     // A/D conversion clock (2*T_CY) gives T_AD = 125 ns.
    // AD1CHS
    _CH0NB = 0;    // Negative input is VR-, as given by _VCFG (MUX B not used)
    _CH0SB = 15;   // Sample AN15 for MUX B (not used)
    _CH0NA = 0;    // Negative input is VT-, as given by _VCFG
    _CH0SA = 15;   // Sample AN15 for MUX A (if single sample)
    // AD1CSSL
    AD1CSSL = 0xff00;// set AN15-8 for sequential scan on MUX A
    
    // set Port B analog input pins with ANS and TRIS
    _ANSB15 = 1;
    _ANSB14 = 1;
    _ANSB13 = 1;
    _ANSB12 = 1;
    _ANSB11 = 1;
    _ANSB10 = 1;
    _ANSB9  = 1;
    _ANSB8  = 1;
    _TRISB15 = 1;
    _TRISB14 = 1;
    _TRISB13 = 1;
    _TRISB12 = 1;
    _TRISB11 = 1;
    _TRISB10 = 1;
    _TRISB9  = 1;
    _TRISB8  = 1;
    
    // turn the module on
    _ADON = 1;
    
    return err;
}

// a simple 1-channel test for the ADC: manual sample, auto conversion
int adc_test_msac(void)
{
    int result;
    _SAMP = 1;  //start single sample
    while (!_DONE);
    result = ADC1BUF0;
    return result;
}

// a multi-channel test for the ADC: scan sample, auto conversion
int adc_test_ssac(void)
{
    int result;
    _ASAM = 1;  // start scan sampling
    while (!_DONE);
    _ASAM = 0;  // stop the scan sampling
    result = ADC1BUF0;
    return result;
}
