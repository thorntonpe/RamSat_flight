/*
 * File:       eps_bat.c
 * Author:     Peter Thornton
 * Purpose:    Functions to interface with the Clyde Space 3rd Generation EPS
 *             and 10 Wh battery
 * Created on: 30 May 2020
*/

#include "xc.h"
#include "clock.h"
#include "i2c.h"
#include "eps_bat.h"

#define EPS_ADDR 0x2B               // 7-bit address for EPS on I2C bus
#define EPS_READ_DELAY 0            // ms delay after EPS read command 

#define BAT_ADDR 0x2A               // 7-bit address for Batery on I2C bus
#define BAT_READ_DELAY 0            // ms delay after Battery read command 


// common address values used by all commands
static int eps_add_w = EPS_ADDR << 1;          // EPS address with write flag set (0)
static int eps_add_r = (EPS_ADDR << 1) | 0b1;  // EPS address with read flag set (1)
static int bat_add_w = BAT_ADDR << 1;          // Bat address with write flag set (0)
static int bat_add_r = (BAT_ADDR << 1) | 0b1;  // Bat address with read flag set (1)

// common memory space used by all raw (byte-level) commands and responses
static unsigned char command[3];           // maximum size for any command
static unsigned char response[2];          // maximum size for any response

// low-level routine to write a command to EPS device
void eps_write_command(int nbytes, int delay)
{
    int i;
    
    // ensure i2c bus is in idle state
    idle_i2c1();
    // initiate an i2c bus start, and wait for it to complete
    start_i2c1();
    // transmit device address byte with write flag
    transmit_i2c1(eps_add_w);
    // transmit the requested number of command bytes
    for (i=0 ; i<nbytes ; i++)
    {
        transmit_i2c1(command[i]);
    }
    // Initiate a stop and wait the specified time (ms)
    // the wait gives device time to prepare response
    // NB: the EPS User Manual does not show the stop condition on 
    // write command, but it doesn't work without it. 
    stop_i2c1(delay);
}

// low-level routine to read a response from EPS device
void eps_read_response(int nbytes)
{
    int i;
    int rcv;
    
    // ensure i2c bus is in idle state
    idle_i2c1();
    // initiate an i2c bus start, and wait for it to complete
    start_i2c1();
    // transmit address byte with read flag
    transmit_i2c1(eps_add_r);
    // receive the requested number of response bytes
    // receive with ack for first nbytes-1 bytes
    for (i=0 ; i<nbytes-1 ; i++)
    {
        receive_i2c1(&rcv);
        response[i] = rcv;
    }
    // receive with nack on last received byte
    receive_i2c1_nack(&rcv);
    response[i] = rcv;
    
    // initiate a stop and wait for it to complete
    stop_i2c1(EPS_READ_DELAY);
}

// low-level routine to write a command to Battery device
void bat_write_command(int nbytes, int delay)
{
    int i;
    
    // ensure i2c bus is in idle state
    idle_i2c1();
    // initiate an i2c bus start, and wait for it to complete
    start_i2c1();
    // transmit device address byte with write flag
    transmit_i2c1(bat_add_w);
    // transmit the requested number of command bytes
    for (i=0 ; i<nbytes ; i++)
    {
        transmit_i2c1(command[i]);
    }
    // Initiate a stop and wait the specified time (ms)
    // the wait gives device time to prepare response
    // NB: the EPS User Manual does not show the stop condition on 
    // write command, but it doesn't work without it. 
    stop_i2c1(delay);
}

// low-level routine to read a response from Battery device
void bat_read_response(int nbytes)
{
    int i;
    int rcv;
    
    // ensure i2c bus is in idle state
    idle_i2c1();
    // initiate an i2c bus start, and wait for it to complete
    start_i2c1();
    // transmit address byte with read flag
    transmit_i2c1(bat_add_r);
    // receive the requested number of response bytes
    // receive with ack for first nbytes-1 bytes
    for (i=0 ; i<nbytes-1 ; i++)
    {
        receive_i2c1(&rcv);
        response[i] = rcv;
    }
    // receive with nack on last received byte
    receive_i2c1_nack(&rcv);
    response[i] = rcv;
    
    // initiate a stop and wait for it to complete
    stop_i2c1(BAT_READ_DELAY);
}

unsigned char eps_get_status()
{
    // load command and parameters
    int nbytes = 2;
    command[0] = 0x01;
    command[1] = 0x00;
    int delay = 1;
    // send command
    eps_write_command(nbytes, delay);
    // read response
    eps_read_response(2);
    // the useful status byte is the second byte returned
    return response[1];
}

float eps_get_bcr1v()
{
    // load command and parameters
    int nbytes = 3;
    command[0] = 0x10;
    command[1] = 0xE1;
    command[2] = 0x10;
    int delay = 5;
    // send command
    eps_write_command(nbytes, delay);
    // read response
    eps_read_response(2);
    // return -999.99 if response is 0xFFFF
    if (response[0]==0xFF && response[1]==0xFF)
        return -999.99;
    // convert 2-byte (10-bit) ADC output to float
    unsigned int adc = (response[0] << 8) | response[1];
    return (float)adc * 0.032600898 - 0.163924275;
}

float eps_get_bcr2v()
{
    // load command and parameters
    int nbytes = 3;
    command[0] = 0x10;
    command[1] = 0xE1;
    command[2] = 0x20;
    int delay = 5;
    // send command
    eps_write_command(nbytes, delay);
    // read response
    eps_read_response(2);
    // convert 2-byte (10-bit) ADC output to float
    int adc = (response[0] << 8) | response[1];
    return (float)adc * 0.032589813 - 0.149882281;
}

float eps_get_bcr3v()
{
    // load command and parameters
    int nbytes = 3;
    command[0] = 0x10;
    command[1] = 0xE1;
    command[2] = 0x30;
    int delay = 5;
    // send command
    eps_write_command(nbytes, delay);
    // read response
    eps_read_response(2);
    // convert 2-byte (10-bit) ADC output to float (V)
    int adc = (response[0] << 8) | response[1];
    return (float)adc * 0.009996558 + 0.001752591;
}

float eps_get_bcroutv()
{
    // load command and parameters
    int nbytes = 3;
    command[0] = 0x10;
    command[1] = 0xE2;
    command[2] = 0x80;
    int delay = 5;
    // send command
    eps_write_command(nbytes, delay);
    // read response
    eps_read_response(2);
    // convert 2-byte (10-bit) ADC output to float (V)
    int adc = (response[0] << 8) | response[1];
    return (float)adc * 0.009009009 - 0.005405405;
}

float eps_get_batv()
{
    // load command and parameters
    int nbytes = 3;
    command[0] = 0x10;
    command[1] = 0xE2;
    command[2] = 0x20;
    int delay = 5;
    // send command
    eps_write_command(nbytes, delay);
    // read response
    eps_read_response(2);
    // convert 2-byte (10-bit) ADC output to float (V)
    int adc = (response[0] << 8) | response[1];
    return (float)adc * 0.009237022 - 0.225405738;
}

float eps_get_bus12v()
{
    // load command and parameters
    int nbytes = 3;
    command[0] = 0x10;
    command[1] = 0xE2;
    command[2] = 0x30;
    int delay = 5;
    // send command
    eps_write_command(nbytes, delay);
    // read response
    eps_read_response(2);
    // convert 2-byte (10-bit) ADC output to float (V)
    int adc = (response[0] << 8) | response[1];
    return (float)adc * 0.013665944 - 0.22075922;
}

float eps_get_bus5v()
{
    // load command and parameters
    int nbytes = 3;
    command[0] = 0x10;
    command[1] = 0xE2;
    command[2] = 0x10;
    int delay = 5;
    // send command
    eps_write_command(nbytes, delay);
    // read response
    eps_read_response(2);
    // convert 2-byte (10-bit) ADC output to float (V)
    int adc = (response[0] << 8) | response[1];
    return (float)adc * 0.007616438 - 1.522821918;
}

float eps_get_bus33v()
{
    // load command and parameters
    int nbytes = 3;
    command[0] = 0x10;
    command[1] = 0xE2;
    command[2] = 0x00;
    int delay = 5;
    // send command
    eps_write_command(nbytes, delay);
    // read response
    eps_read_response(2);
    // convert 2-byte (10-bit) ADC output to float (V)
    int adc = (response[0] << 8) | response[1];
    return (float)adc * 0.005818306 - 1.179393443;
}

float eps_get_bcr1ia()
{
    // load command and parameters
    int nbytes = 3;
    command[0] = 0x10;
    command[1] = 0xE1;
    command[2] = 0x14;
    int delay = 5;
    // send command
    eps_write_command(nbytes, delay);
    // read response
    eps_read_response(2);
    // convert 2-byte (10-bit) ADC output to float (mA)
    int adc = (response[0] << 8) | response[1];
    return (float)adc * 0.965003633 - 0.970816178;
}

float eps_get_bcr1ib()
{
    // load command and parameters
    int nbytes = 3;
    command[0] = 0x10;
    command[1] = 0xE1;
    command[2] = 0x15;
    int delay = 5;
    // send command
    eps_write_command(nbytes, delay);
    // read response
    eps_read_response(2);
    // convert 2-byte (10-bit) ADC output to float (mA)
    int adc = (response[0] << 8) | response[1];
    return (float)adc * 0.97409839 + 7.889228648;
}

float eps_get_bcr2ia()
{
    // load command and parameters
    int nbytes = 3;
    command[0] = 0x10;
    command[1] = 0xE1;
    command[2] = 0x24;
    int delay = 5;
    // send command
    eps_write_command(nbytes, delay);
    // read response
    eps_read_response(2);
    // convert 2-byte (10-bit) ADC output to float (mA)
    int adc = (response[0] << 8) | response[1];
    return (float)adc * 0.991807411 + 10.87345601;
}

float eps_get_bcr2ib()
{
    // load command and parameters
    int nbytes = 3;
    command[0] = 0x10;
    command[1] = 0xE1;
    command[2] = 0x25;
    int delay = 5;
    // send command
    eps_write_command(nbytes, delay);
    // read response
    eps_read_response(2);
    // convert 2-byte (10-bit) ADC output to float (mA)
    int adc = (response[0] << 8) | response[1];
    return (float)adc * 0.98125368 + 9.791931336;
}

float eps_get_bcr3ia()
{
    // load command and parameters
    int nbytes = 3;
    command[0] = 0x10;
    command[1] = 0xE1;
    command[2] = 0x34;
    int delay = 5;
    // send command
    eps_write_command(nbytes, delay);
    // read response
    eps_read_response(2);
    // convert 2-byte (10-bit) ADC output to float (mA)
    int adc = (response[0] << 8) | response[1];
    return (float)adc * 1.000522183 - 0.81609817;
}

float eps_get_bcr3ib()
{
    // load command and parameters
    int nbytes = 3;
    command[0] = 0x10;
    command[1] = 0xE1;
    command[2] = 0x35;
    int delay = 5;
    // send command
    eps_write_command(nbytes, delay);
    // read response
    eps_read_response(2);
    // convert 2-byte (10-bit) ADC output to float (mA)
    int adc = (response[0] << 8) | response[1];
    return (float)adc * 0.971043376 + 5.230754201;
}

float eps_get_bati()
{
    // load command and parameters
    int nbytes = 3;
    command[0] = 0x10;
    command[1] = 0xE2;
    command[2] = 0x24;
    int delay = 5;
    // send command
    eps_write_command(nbytes, delay);
    // read response
    eps_read_response(2);
    // convert 2-byte (10-bit) ADC output to float (mA)
    int adc = (response[0] << 8) | response[1];
    return (float)adc * 6.843346065 - 10.61176728;
}

float eps_get_bus12i()
{
    // load command and parameters
    int nbytes = 3;
    command[0] = 0x10;
    command[1] = 0xE2;
    command[2] = 0x34;
    int delay = 5;
    // send command
    eps_write_command(nbytes, delay);
    // read response
    eps_read_response(2);
    // convert 2-byte (10-bit) ADC output to float (mA)
    int adc = (response[0] << 8) | response[1];
    return (float)adc * 2.056275285 + 0.641165577;
}

float eps_get_bus5i()
{
    // load command and parameters
    int nbytes = 3;
    command[0] = 0x10;
    command[1] = 0xE2;
    command[2] = 0x14;
    int delay = 5;
    // send command
    eps_write_command(nbytes, delay);
    // read response
    eps_read_response(2);
    // convert 2-byte (10-bit) ADC output to float (mA)
    int adc = (response[0] << 8) | response[1];
    return (float)adc * 6.848193861 - 11.51178137;
}

float eps_get_bus33i()
{
    // load command and parameters
    int nbytes = 3;
    command[0] = 0x10;
    command[1] = 0xE2;
    command[2] = 0x04;
    int delay = 5;
    // send command
    eps_write_command(nbytes, delay);
    // read response
    eps_read_response(2);
    // convert 2-byte (10-bit) ADC output to float (mA)
    int adc = (response[0] << 8) | response[1];
    return (float)adc * 6.834722211 - 12.64684042;
}

float eps_get_eps5i()
{
    // load command and parameters
    int nbytes = 3;
    command[0] = 0x10;
    command[1] = 0xE2;
    command[2] = 0x15;
    int delay = 5;
    // send command
    eps_write_command(nbytes, delay);
    // read response
    eps_read_response(2);
    // convert 2-byte (10-bit) ADC output to float (mA)
    int adc = (response[0] << 8) | response[1];
    return (float)adc * 1.327547;
}

float eps_get_eps33i()
{
    // load command and parameters
    int nbytes = 3;
    command[0] = 0x10;
    command[1] = 0xE2;
    command[2] = 0x05;
    int delay = 5;
    // send command
    eps_write_command(nbytes, delay);
    // read response
    eps_read_response(2);
    // convert 2-byte (10-bit) ADC output to float (mA)
    int adc = (response[0] << 8) | response[1];
    return (float)adc * 1.327547;
}

float eps_get_mbt()
{
    // load command and parameters
    int nbytes = 3;
    command[0] = 0x10;
    command[1] = 0xE3;
    command[2] = 0x08;
    int delay = 5;
    // send command
    eps_write_command(nbytes, delay);
    // read response
    eps_read_response(2);
    // convert 2-byte (10-bit) ADC output to float (mA)
    int adc = (response[0] << 8) | response[1];
    return ((float)adc * 0.372434) - 273.15;
}

// Turn on PDM Switch #7, 5V power to Arducams
unsigned char eps_cameras_on()
{
    // load command and parameters
    int nbytes = 2;
    command[0] = 0x50;
    command[1] = 0x07;
    int delay = 1;
    // send command
    eps_write_command(nbytes, delay);
    // read response
    eps_read_response(2);
    // the useful status byte is the second byte returned
    return response[1];    
}

// Turn off PDM Switch #7, 5V power to Arducams
unsigned char eps_cameras_off()
{
    // load command and parameters
    int nbytes = 2;
    command[0] = 0x51;
    command[1] = 0x07;
    int delay = 1;
    // send command
    eps_write_command(nbytes, delay);
    // read response
    eps_read_response(2);
    // the useful status byte is the second byte returned
    return response[1];    
}

// battery telemetry: status byte
unsigned char bat_get_status()
{
    // load command and parameters
    int nbytes = 2;
    command[0] = 0x01;
    command[1] = 0x00;
    int delay = 1;
    // send command
    bat_write_command(nbytes, delay);
    // read response
    bat_read_response(2);
    // the useful status byte is the second byte returned
    return response[1];
}

// battery telemetry: voltage
float bat_get_batv()
{
    // load command and parameters
    int nbytes = 3;
    command[0] = 0x10;
    command[1] = 0xE2;
    command[2] = 0x80;
    int delay = 5;
    // send command
    bat_write_command(nbytes, delay);
    // read response
    bat_read_response(2);
    // convert 2-byte (10-bit) ADC output to float (V)
    int adc = (response[0] << 8) | response[1];
    return (float)adc * 0.008967413;
}

// battery telemetry: current
float bat_get_bati()
{
    // load command and parameters
    int nbytes = 3;
    command[0] = 0x10;
    command[1] = 0xE2;
    command[2] = 0x84;
    int delay = 5;
    // send command
    bat_write_command(nbytes, delay);
    // read response
    bat_read_response(2);
    // convert 2-byte (10-bit) ADC output to float (mA)
    int adc = (response[0] << 8) | response[1];
    return (float)adc * 14.49751076 - 3.353549658;
}

// battery telemetry: charge/discharge status
int bat_get_batischarging()
{
    // load command and parameters
    int nbytes = 3;
    command[0] = 0x10;
    command[1] = 0xE2;
    command[2] = 0x8E;
    int delay = 5;
    // send command
    bat_write_command(nbytes, delay);
    // read response
    bat_read_response(2);
    // convert 2-byte (10-bit) ADC output to float (mA)
    int adc = (response[0] << 8) | response[1];
    // if adc less than 512 battery is charging, otherwise battery is discharging
    return (adc < 512);
}

// battery telemetry: motherboard temperature
float bat_get_mbt()
{
    // load command and parameters
    int nbytes = 3;
    command[0] = 0x10;
    command[1] = 0xE3;
    command[2] = 0x08;
    int delay = 5;
    // send command
    bat_write_command(nbytes, delay);
    // read response
    bat_read_response(2);
    // convert 2-byte (10-bit) ADC output to float (mA)
    int adc = (response[0] << 8) | response[1];
    return ((float)adc * 0.394871139) - 293.068931;
}

// battery telemetryL daughterboard temperature
float bat_get_dbt()
{
    // load command and parameters
    int nbytes = 3;
    command[0] = 0x10;
    command[1] = 0xE3;
    command[2] = 0x98;
    int delay = 5;
    // send command
    bat_write_command(nbytes, delay);
    // read response
    bat_read_response(2);
    // convert 2-byte (10-bit) ADC output to float (mA)
    int adc = (response[0] << 8) | response[1];
    return ((float)adc * 0.428580355) - 262.0490741;
}


