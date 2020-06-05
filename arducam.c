/*
 * File:       arducam.c
 * Author:     Peter Thornton
 * Purpose:    Functions to interface with Arducam 2MP Plus camera
 *             Includes both I2C interface for OV2640 registers, and
 *             SPI interface for image capture and retrieval
 * Created on: 17 November 2019
 */

#include "xc.h"
#include "clock.h"
#include "i2c.h"
#include "spi.h"
#include "arducam.h"

#define ARDU_ADDR 0x30               // address for Arducam image chip OV2640 on I2C bus
#define ARDU_DELAY 1                 // 1 msec delay after a stop to allow
                                     // device state to update

// common I2C address values used by all commands
int ardu_add_w = ARDU_ADDR << 1;          // Arducam address with write flag set (0)
int ardu_add_r = (ARDU_ADDR << 1) | 0b1;  // Arducam address with read flag set (1)

// timer variables
long wait;

// use the I2C interface to write register values to the OV2640 chip on camera #1
void write_ov2640_reg_cam1(int reg, int val)
{
    // ensure i2c bus is in idle state
    idle_i2c1();
    // initiate an i2c bus start, and wait for it to complete
    start_i2c1();
    // transmit device address byte with write flag
    transmit_i2c1(ardu_add_w);
    // transmit the register ID
    transmit_i2c1(reg);
    // transmit the register value
    transmit_i2c1(val);
    // initiate a stop and wait for it to complete
    stop_i2c1(ARDU_DELAY);
}

// use the I2C interface to write register values to the OV2640 chip on camera #2
void write_ov2640_reg_cam2(int reg, int val)
{
    // ensure i2c bus is in idle state
    idle_i2c2();
    // initiate an i2c bus start, and wait for it to complete
    start_i2c2();
    // transmit device address byte with write flag
    transmit_i2c2(ardu_add_w);
    // transmit the register ID
    transmit_i2c2(reg);
    // transmit the register value
    transmit_i2c2(val);
    // initiate a stop and wait for it to complete
    stop_i2c2(ARDU_DELAY);
}

// use the I2C interface to read register values from the OV2640 chip on camera #1
void read_ov2640_reg_cam1(int reg, int *val)
{
    // ensure i2c bus is in idle state
    idle_i2c1();
    // initiate an i2c bus start, and wait for it to complete
    start_i2c1();
    // transmit device address byte with write flag
    transmit_i2c1(ardu_add_w);
    // transmit the register ID
    transmit_i2c1(reg);
    // initiate a stop and wait for it to complete
    stop_i2c1(ARDU_DELAY);
     // ensure i2c bus is in idle state
    idle_i2c1();
    // initiate an i2c bus start, and wait for it to complete
    start_i2c1();
    // transmit device address byte with read flag
    transmit_i2c1(ardu_add_r);
    // receive the register value and acknowledge with NACK
    receive_i2c1_nack(val);
    // initiate a stop and wait for it to complete
    stop_i2c1(ARDU_DELAY);
}

// use the I2C interface to read register values from the OV2640 chip on camera #2
void read_ov2640_reg_cam2(int reg, int *val)
{
    // ensure i2c bus is in idle state
    idle_i2c2();
    // initiate an i2c bus start, and wait for it to complete
    start_i2c2();
    // transmit device address byte with write flag
    transmit_i2c2(ardu_add_w);
    // transmit the register ID
    transmit_i2c2(reg);
    // initiate a stop and wait for it to complete
    stop_i2c2(ARDU_DELAY);
     // ensure i2c bus is in idle state
    idle_i2c2();
    // initiate an i2c bus start, and wait for it to complete
    start_i2c2();
    // transmit device address byte with read flag
    transmit_i2c2(ardu_add_r);
    // receive the register value and acknowledge with NACK
    receive_i2c2_nack(val);
    // initiate a stop and wait for it to complete
    stop_i2c2(ARDU_DELAY);
}

// reset the OV2640 registers to factory defaults on camera #1
void reset_ov2640_regs_cam1()
{
    // switch to second register bank, set bit 7 of register 0x12 (COM7)
    // to perform a device reset of the OV2640 chip
    write_ov2640_reg_cam1(0xff, 0x01);
    write_ov2640_reg_cam1(0x12, 0x80);
    wait = 100 * DELAYMSEC;
    TMR1=0;
    while (TMR1 < wait);
}

// reset the OV2640 registers to factory defaults on camera #2
void reset_ov2640_regs_cam2()
{
    // switch to second register bank, set bit 7 of register 0x12 (COM7)
    // to perform a device reset of the OV2640 chip
    write_ov2640_reg_cam2(0xff, 0x01);
    write_ov2640_reg_cam2(0x12, 0x80);
    wait = 100 * DELAYMSEC;
    TMR1=0;
    while (TMR1 < wait);
}

// Write an array of values to 8 bit register address on camera #1
void init_ov2640_regs_cam1(const struct sensor_reg reglist[])
{
    int i=0;
    int reg_addr = 0;
    int reg_val = 0;
    while ((reglist[i].reg != 0xff) | (reglist[i].val != 0xff))
    {
        reg_addr = reglist[i].reg;
        reg_val = reglist[i].val;
        write_ov2640_reg_cam1(reg_addr, reg_val);
        i++;
    }
}

// Write an array of values to 8 bit register address on camera #2
void init_ov2640_regs_cam2(const struct sensor_reg reglist[])
{
    int i=0;
    int reg_addr = 0;
    int reg_val = 0;
    while ((reglist[i].reg != 0xff) | (reglist[i].val != 0xff))
    {
        reg_addr = reglist[i].reg;
        reg_val = reglist[i].val;
        write_ov2640_reg_cam2(reg_addr, reg_val);
        i++;
    }
}

void arduchip_write_reg(int reg, int val, int cs)
{
    // cs is a chip select index to camera #1 or camera #2
    // select the device
    switch (cs)
    {
        case 1:
            CS_CAM1 = 0;
            break;
        case 2:
            CS_CAM2 = 0;
            break;
        default:
            CS_CAM1 = 0;
    }

    // write the value to the register
    // setting bit 7 of the address indicates write operation
    write_spi2(reg | 0x80);
    write_spi2(val);

    // deselect the device
    switch (cs)
    {
        case 1:
            CS_CAM1 = 1;
            break;
        case 2:
            CS_CAM2 = 1;
            break;
        default:
            CS_CAM1 = 1;
    }
}

int arduchip_read_reg(int reg, int cs)
{
    int val;
    // cs is a chip select index to camera #1 or camera #2
    // select the device
    switch (cs)
    {
        case 1:
            CS_CAM1 = 0;
            break;
        case 2:
            CS_CAM2 = 0;
            break;
        default:
            CS_CAM1 = 0;
    }

    // write address, then read value
    write_spi2(reg);
    val = write_spi2(0);
    // deselect the device
    switch (cs)
    {
        case 1:
            CS_CAM1 = 1;
            break;
        case 2:
            CS_CAM2 = 1;
            break;
        default:
            CS_CAM1 = 1;
    }
    
    return val;
}

// test write and read to ArduChip register
int arduchip_testreg(int testin, int cs)
{
    // perform a test write and read to the arduchip TEST register
    // to test operation of the SPI interface
    // Returns testout, and the user should compare to testval.
    // cs is the chip select for arducam #1 (1) or arducam #2 (2)
    
    int testreg = 0x00;   // address of the arduchip TEST register
    // make sure that testout is initialized different than testin
    int testout = ~(testin);
    
    // write the testval to test register
    arduchip_write_reg(testreg, testin, cs);
    
    // read the value in test register
    testout = arduchip_read_reg(testreg, cs);
    
    return testout;
}

// reset the Arducam CPLD (arduchip)
void arduchip_reset(int cs)
{
    // cs is the chip select for arducam #1 (1) or arducam #2 (2)
    arduchip_write_reg(0x07, 0x80, cs);
    wait = 100 * DELAYMSEC;
    TMR1 = 0;
    while (TMR1 < wait);
    arduchip_write_reg(0x07, 0x00, cs);
    TMR1 = 0;
    while (TMR1 < wait);    
}

// clear the arduchip FIFO "write done" flag
void arduchip_clear_fifo(int cs)
{
    // cs is the chip select for arducam #1 (1) or arducam #2 (2)
    arduchip_write_reg(0x04, 0x01, cs);
}

// set the number of frames to capture
void arduchip_set_nframes(int n, int cs)
{
    // cs is the chip select for arducam #1 (1) or arducam #2 (2)
    int nframes = n-1;                 // register value + 1 = # of frames to write
    arduchip_write_reg(0x01, nframes, cs);
}

// start an image capture
long arduchip_start_capture(int cs)
{
    // cs is the chip select for arducam #1 (1) or arducam #2 (2)
    long msec;
    arduchip_write_reg(0x04, 0x02, cs);
    TMR1 = 0;
    while (!(arduchip_capture_done(cs)));
    msec = TMR1 / DELAYMSEC;
    return msec;    
}

// check if the image capture is done (0=no, 1=yes)
int arduchip_capture_done(int cs)
{
    // cs is the chip select for arducam #1 (1) or arducam #2 (2)
    int done_bit; // bit 3 of register 0x41 indicates write to FIFO is done
    int regval;
    regval = arduchip_read_reg(0x41, cs);
    done_bit = regval & 0x08;
    return (done_bit != 0);
}

long arduchip_fifo_length(int *outlen1, int *outlen2, int *outlen3, int cs)
{
    // cs is the chip select for arducam #1 (1) or arducam #2 (2)
    int len1,len2,len3;
    long llen1, llen2, llen3;
    long length=0;
    
    wait = 50 * DELAYMSEC;
    TMR1=0;
    while (TMR1 < wait);
    
    len1 = arduchip_read_reg(0x42, cs);
    len2 = arduchip_read_reg(0x43, cs);
    len3 = arduchip_read_reg(0x44, cs);
    llen1 = (long)len1;
    llen2 = (long)len2;
    llen3 = (long)len3;
    length = ((llen3 << 16) | (llen2 << 8) | llen1);
    *outlen1 = len1;
    *outlen2 = len2;
    *outlen3 = len3;
    return length;	
}
