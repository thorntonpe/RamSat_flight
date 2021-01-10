/*
 * File:       ants.c
 * Author:     Peter Thornton
 * Purpose:    Functions to interface with ISIS ANTs antenna
 * Created on: 6 June 2020
 */

#include "xc.h"
#include "clock.h"
#include "i2c.h"
#include "ants.h"

#define ANTS_ADDR 0x31               // address for ANTS (A) on I2C bus
#define ANTS_DELAY 1                 // 1 msec delay after a stop to allow
                                     // device state to update
// common address values used by all commands
static int add_w = ANTS_ADDR << 1;          // ANTS address with write flag set (0)
static int add_r = (ANTS_ADDR << 1) | 0b1;  // ANTS address with read flag set (1)

// common memory space used by all raw (byte-level) commands and responses
static unsigned char command[2];          // maximum size for any command
static unsigned char response[2];         // maximum size for any response,

// low-level routine to write a command to ANTs device
void write_ants_command(int nbytes)
{
    int i=0;
    
    // ensure i2c bus is in idle state
    idle_i2c1();
    // initiate an i2c bus start, and wait for it to complete
    start_i2c1();
    // transmit device address byte with write flag
    transmit_i2c1(add_w);
    // transmit the requested number of command bytes
    for (i=0 ; i<nbytes ; i++)
    {
        transmit_i2c1(command[i]);
    }
    // initiate a stop and wait for it to complete
    stop_i2c1(ANTS_DELAY);
}

// low-level routine to write a command to ANTs device, with no ACK expected
void write_ants_command_noack(int nbytes)
{
    int i=0;
    
    // ensure i2c bus is in idle state
    idle_i2c1();
    // initiate an i2c bus start, and wait for it to complete
    start_i2c1();
    // transmit device address byte with write flag
    transmit_i2c1(add_w);
    // transmit the requested number of command bytes
    for (i=0 ; i<nbytes ; i++)
    {
        transmit_i2c1_noack(command[i]);
    }
}

// low-level routine to read a response from ANTs device
void read_ants_response(int nbytes)
{
    int i=0;
    int rcv;
    
    // ensure i2c bus is in idle state
    idle_i2c1();
    // initiate an i2c bus start, and wait for it to complete
    start_i2c1();
    // transmit address byte with read flag
    transmit_i2c1(add_r);
    // receive the requested number of response bytes
    for (i=0 ; i<nbytes ; i++)
    {
        receive_i2c1(&rcv);
        response[i] = rcv;
    }
    // initiate a stop and wait for it to complete
    stop_i2c1(ANTS_DELAY);
}

// low-level routine to read a response from ANTs device
void read_ants_response_nack(int nbytes)
{
    int i=0;
    int rcv;
    
    // ensure i2c bus is in idle state
    idle_i2c1();
    // initiate an i2c bus start, and wait for it to complete
    start_i2c1();
    // transmit address byte with read flag
    transmit_i2c1(add_r);
    // receive the requested number of response bytes
    for (i=0 ; i<nbytes-1 ; i++)
    {
        receive_i2c1(&rcv);
        response[i] = rcv;
    }
    receive_i2c1_nack(&rcv);
    response[i] = rcv;
    // initiate a stop and wait for it to complete
    stop_i2c1(ANTS_DELAY);
}

// perform a software reset of the ANTs device
// (equivalent to power cycling the device)
void ants_reset()
{
    int cmd_id1 = 0xAA;               // command id byte1
    int cmd_nbytes = 1;               // number of bytes in command
    long wait;                        // timer delay
    
    // fill the command bytes
    command[0] = cmd_id1;
    // send the command
    write_ants_command_noack(cmd_nbytes);
    
    // hardwired wait for device reset, 2 seconds
    wait = 1000L * DELAYMSEC;
    TMR1=0;
    while(TMR1 < wait);
    TMR1=0;
    while(TMR1 < wait);
}

// report deployment status (also a test of the i2c interface))
void ants_deploy_status(unsigned char* ants_resp)
{
    int cmd_id = 0xC3;                // command id
    int cmd_nbytes = 1;               // number of bytes in command
    int rsp_nbytes = 2;               // number of bytes in response
    
    // fill the command bytes
    command[0] = cmd_id;
    // send the command
    write_ants_command(cmd_nbytes);
    // read the response
    read_ants_response_nack(rsp_nbytes);
    // transfer response bytes into output array
    // MSB first, then LSB
    *ants_resp++ = response[1];
    *ants_resp   = response[0];
}

// arm the antenna system to enable deployment
void ants_arm()
{
    int cmd_id1 = 0xAD;               // command id byte1
    int cmd_nbytes = 1;               // number of bytes in command
    
    // fill the command bytes
    command[0] = cmd_id1;
    // send the command
    write_ants_command(cmd_nbytes);
}

// disarm the antenna system to disable deployment
void ants_disarm()
{
    int cmd_id1 = 0xAC;               // command id byte1
    int cmd_nbytes = 1;               // number of bytes in command
    
    // fill the command bytes
    command[0] = cmd_id1;
    // send the command
    write_ants_command(cmd_nbytes);
}

// initiate the deployment of all antennas, one at a time, from 1 to 4
void ants_deploy_all()
{
    int cmd_id = 0xA5;                // command id
    int cmd_nbytes = 2;               // number of bytes in command
    
    // fill the command bytes
    command[0] = cmd_id;
    command[1] = 10;     // 10 seconds max to deploy per antenna
    // send the command
    write_ants_command(cmd_nbytes);    
}

// time taken to deploy antenna 1
void ants_time_1(unsigned char* ants_resp)
{
    int cmd_id = 0xB4;                // command id
    int cmd_nbytes = 1;               // number of bytes in command
    int rsp_nbytes = 2;               // number of bytes in response
    
    // fill the command bytes
    command[0] = cmd_id;
    // send the command
    write_ants_command(cmd_nbytes);
    // read the response
    read_ants_response_nack(rsp_nbytes);
    // transfer response bytes into output array
    *ants_resp++ = response[1];
    *ants_resp   = response[0];
}

// time taken to deploy antenna 2
void ants_time_2(unsigned char* ants_resp)
{
    int cmd_id = 0xB5;                // command id
    int cmd_nbytes = 1;               // number of bytes in command
    int rsp_nbytes = 2;               // number of bytes in response
    
    // fill the command bytes
    command[0] = cmd_id;
    // send the command
    write_ants_command(cmd_nbytes);
    // read the response
    read_ants_response_nack(rsp_nbytes);
    // transfer response bytes into output array
    *ants_resp++ = response[1];
    *ants_resp   = response[0];
}

// time taken to deploy antenna 3
void ants_time_3(unsigned char* ants_resp)
{
    int cmd_id = 0xB6;                // command id
    int cmd_nbytes = 1;               // number of bytes in command
    int rsp_nbytes = 2;               // number of bytes in response
    
    // fill the command bytes
    command[0] = cmd_id;
    // send the command
    write_ants_command(cmd_nbytes);
    // read the response
    read_ants_response_nack(rsp_nbytes);
    // transfer response bytes into output array
    *ants_resp++ = response[1];
    *ants_resp   = response[0];
}

// time taken to deploy antenna 4
void ants_time_4(unsigned char* ants_resp)
{
    int cmd_id = 0xB7;                // command id
    int cmd_nbytes = 1;               // number of bytes in command
    int rsp_nbytes = 2;               // number of bytes in response
    
    // fill the command bytes
    command[0] = cmd_id;
    // send the command
    write_ants_command(cmd_nbytes);
    // read the response
    read_ants_response_nack(rsp_nbytes);
    // transfer response bytes into output array
    *ants_resp++ = response[1];
    *ants_resp   = response[0];
}
