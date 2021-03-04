/* 
 * File:     init.h
 * Author:   Peter Thornton
 * Purpose:  data type and prototypes for initialization routines
 * Created:  13 May 2020
 */

//#include "clock.h"

// data types
typedef struct
{
    long u1br_request;      // UART1 requested baud rate
    long u1br_actual;       // UART1 actual baud rate
    long u2br_request;      // UART2 requested baud rate
    long u2br_actual;       // UART2 actual baud rate
    long spi1_prescalar;    // SPI1 clock prescalar
    long spi1_fsck;         // SPI1 clock speed
    long spi2_prescalar;    // SPI2 clock prescalar
    long spi2_fsck;         // SPI2 clock speed
    long spi3_prescalar;    // SPI3 clock prescalar
    long spi3_fsck;         // SPI3 clock speed    
    long i2c1br;            // desired I2C_1 baud rate = 100 kHz
    long brg1_reload;       // reload value for I2C_1 BRG
    long i2c2br;            // desired I2C_2 baud rate = 100 kHz
    long brg2_reload;       // reload value for I2C_2 BRG
    int  adc_iserror;       // error condition for initialization of ADC
    int  sfm_iserror;       // error condition for test of serial flash memory
    int  sd_iserror;        // error condition for test of SD card write/read/delete
    int  rtc_flags_iserror; // error condition for first test of real time clock flags
    int  rtc_flags2_iserror;// error condition for second test of real time clock flags
    int  rtc_clear_iserror; // error condition for attempt to clear RTC flags
    int  rtc_flags;         // initial HALT (bit 2), STOP (bit 1), and OF (bit 0) flags
    int  rtc_flags2;        // RTC flags after attempted clear
    char rtc_halt_time[26]; // ISO 8601 datetime at previous RTC halt
    char rtc_init_time[26]; // ISO 8601 datetime at startup
    int  pdt_status;        // 0 = didn't wait, 1 = waited, 2 = unrecognized flag
    int  pdt_flag;          // the flag value read from serial flash memory on initialization
    unsigned char eps_antenna_on_iserror;  // error flag for EPS power on command to antenna (switch #8)
    unsigned char antenna_on_status;       // status value from command to turn on antenna power
    unsigned char antenna_on_status2;      // status value from command to turn on backup antenna power
    unsigned char eps_antenna_off_iserror; // error flag for EPS power off command to antenna (switch #8)
    unsigned char antenna_off_status;      // status value from command to turn off antenna power
    unsigned char antenna_off_status2;     // status value from command to turn off backup antenna power
    unsigned char ants0_deploy_status_msb; // MSB of antenna deployment status response (before arm)
    unsigned char ants0_deploy_status_lsb; // LSB of antenna deployment status response (before arm)
    unsigned char ants1_deploy_status_msb; // MSB of antenna deployment status response (after arm)
    unsigned char ants1_deploy_status_lsb; // LSB of antenna deployment status response (after arm)
    unsigned char ants2_deploy_status_msb; // MSB of antenna deployment status response (after deploy)
    unsigned char ants2_deploy_status_lsb; // LSB of antenna deployment status response (after deploy)
    unsigned char ants3_deploy_status_msb; // MSB of antenna deployment status response (after disarm)
    unsigned char ants3_deploy_status_lsb; // LSB of antenna deployment status response (after disarm)
    unsigned char ants_deploy_time1_msb;   // MSB of antenna 1 deployment time (x 50ms)
    unsigned char ants_deploy_time1_lsb;   // LSB of antenna 1 deployment time (x 50ms)
    unsigned char ants_deploy_time2_msb;   // MSB of antenna 2 deployment time (x 50ms)
    unsigned char ants_deploy_time2_lsb;   // LSB of antenna 2 deployment time (x 50ms)
    unsigned char ants_deploy_time3_msb;   // MSB of antenna 3 deployment time (x 50ms)
    unsigned char ants_deploy_time3_lsb;   // LSB of antenna 3 deployment time (x 50ms)
    unsigned char ants_deploy_time4_msb;   // MSB of antenna 4 deployment time (x 50ms)
    unsigned char ants_deploy_time4_lsb;   // LSB of antenna 4 deployment time (x 50ms)
    float batv;             // battery voltage at startup
} init_data_type;

// function prototypes
void init_peripherals(init_data_type *init_data_p);
void init_motherboard_components(init_data_type *init_data_p);
void init_wait(init_data_type *init_data_p);




