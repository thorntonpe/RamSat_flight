/*
 * File:       imtq.c
 * Author:     Peter Thornton
 * Purpose:    Functions to interface with ISIS iMTQ Magnetorquer
 * Created on: 30 August 2019
 */

#include "xc.h"
#include "clock.h"
#include "i2c.h"
#include "imtq.h"

#define IMTQ_ADDR 0x10               // address for iMTQ on I2C bus
#define IMTQ_DELAY 1                 // 1 msec delay after a stop to allow
                                     // device state to update
// common address values used by all commands
static int add_w = IMTQ_ADDR << 1;          // iMTQ address with write flag set (0)
static int add_r = (IMTQ_ADDR << 1) | 0b1;  // iMTQ address with read flag set (1)

// common memory space used by all raw (byte-level) commands and responses
static unsigned char command[12];           // maximum size for any command
static unsigned char response[120];         // maximum size for any response,
                                     // assumes single-axis self-test

// used to extract signed long values from response byte stream
static union slong
{
    signed long val;
    unsigned char bytes[4];
} slong1;

// used to extract unsigned long values from response byte stream
static union ulong
{
    unsigned long val;
    unsigned char bytes[4];
} ulong1;

// used to extract signed short int values from response byte stream
static union sshort
{
    signed short val;
    unsigned char bytes[2];
} sshort1;

// used to extract unsigned short int values from response byte stream
static union ushort
{
    unsigned short val;
    unsigned char bytes[2];
} ushort1;

// low-level routine to write a command to iMTQ device
void write_imtq_command(int nbytes)
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
    stop_i2c1(IMTQ_DELAY);
}

// low-level routine to write a command to iMTQ device, with no ACK expected
void write_imtq_command_noack(int nbytes)
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
// low-level routine to read a response from iMTQ device
void read_imtq_response(int nbytes)
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
    stop_i2c1(IMTQ_DELAY);
}

// low-level routine to extract one signed long value from response byte stream
void extract_slong(int start, signed long *result)
{
    slong1.bytes[0]=response[start];
    slong1.bytes[1]=response[start+1];
    slong1.bytes[2]=response[start+2];
    slong1.bytes[3]=response[start+3];
    *result = slong1.val;
}

// low-level routine to extract one unsigned long value from response byte stream
void extract_ulong(int start, unsigned long *result)
{
    ulong1.bytes[0]=response[start];
    ulong1.bytes[1]=response[start+1];
    ulong1.bytes[2]=response[start+2];
    ulong1.bytes[3]=response[start+3];
    *result = ulong1.val;
}

// low-level routine to extract one signed short int value from response byte stream
void extract_sshort(int start, signed short *result)
{
    sshort1.bytes[0]=response[start];
    sshort1.bytes[1]=response[start+1];
    *result = sshort1.val;
}

// low-level routine to extract one unsigned short int value from response byte stream
void extract_ushort(int start, unsigned short *result)
{
    ushort1.bytes[0]=response[start];
    ushort1.bytes[1]=response[start+1];
    *result = ushort1.val;
}

// perform a software reset of the iMTQ device
// (equivalent to power cycling the device)
void imtq_reset()
{
    int cmd_id1 = 0xAA;               // command id byte1
    int cmd_id2 = 0xA5;               // command id byte2
    int cmd_nbytes = 2;               // number of bytes in command
    long wait;                        // timer delay
    
    // fill the command bytes
    command[0] = cmd_id1;
    command[1] = cmd_id2;
    // send the command
    write_imtq_command_noack(cmd_nbytes);
    
    // hardwired wait for device reset
    wait = 2000L * DELAYMSEC;
    TMR1=0;
    while(TMR1 < wait)
    {
        
    }
}

// no-operation command to test i2c interface
void imtq_no_op(imtq_resp_common* imtq_common)
{
    int cmd_id = 0x02;                // command id
    int cmd_nbytes = 1;               // number of bytes in command
    int rsp_nbytes = 2;               // number of bytes in response
    
    // fill the command bytes
    command[0] = cmd_id;
    // send the command
    write_imtq_command(cmd_nbytes);
    // read the response
    read_imtq_response(rsp_nbytes);
    // translate response bytes into response struct
    imtq_common->cc = response[0];
    imtq_common->stat = response[1];
}

// get the current state of iMTQ
void imtq_get_state(imtq_resp_common* imtq_common, imtq_resp_state* imtq_state)
{
    int cmd_id = 0x41;                // command id
    int cmd_nbytes = 1;               // number of bytes in command
    int rsp_nbytes = 9;               // number of bytes in response
    
    // fill the command bytes
    command[0] = cmd_id;
    // send the command
    write_imtq_command(cmd_nbytes);
    // read the response
    read_imtq_response(rsp_nbytes);
    // translate response bytes into struct values
    imtq_common->cc = response[0];
    imtq_common->stat = response[1];
    imtq_state->mode = response[2];
    imtq_state->err  = response[3];
    imtq_state->conf = response[4];
    extract_ulong(5, &imtq_state->uptime);
}

// start making a magnetometer reading
void imtq_start_mtm(imtq_resp_common* imtq_common)
{
    int cmd_id = 0x04;                // command id
    int cmd_nbytes = 1;               // number of bytes in command
    int rsp_nbytes = 2;               // number of bytes in response
    
    // fill the command bytes
    command[0] = cmd_id;
    // send the command
    write_imtq_command(cmd_nbytes);
    // read the response
    read_imtq_response(rsp_nbytes);
    // translate response bytes into response struct
    imtq_common->cc = response[0];
    imtq_common->stat = response[1];
}

// get calibrated magnetometer data
void imtq_get_calib_mtm(imtq_resp_common* imtq_common, imtq_resp_mtm* imtq_mtm)
{
    int cmd_id = 0x43;                // command id
    int cmd_nbytes = 1;               // number of bytes in command
    int rsp_nbytes = 15;              // number of bytes in response
    
    // fill the command bytes
    command[0] = cmd_id;
    // send the command
    write_imtq_command(cmd_nbytes);
    // read the response
    read_imtq_response(rsp_nbytes);
    // translate response bytes into response struct
    imtq_common->cc = response[0];
    imtq_common->stat = response[1];
    extract_slong(2,  &imtq_mtm->x);
    extract_slong(6,  &imtq_mtm->y);
    extract_slong(10, &imtq_mtm->z);
    imtq_mtm->coilact = response[14];
}

// get raw (uncalibrated) magnetometer data
void imtq_get_raw_mtm(imtq_resp_common* imtq_common, imtq_resp_mtm* imtq_mtm)
{
    int cmd_id = 0x42;                // command id
    int cmd_nbytes = 1;               // number of bytes in command
    int rsp_nbytes = 15;              // number of bytes in response
    
    // fill the command bytes
    command[0] = cmd_id;
    // send the command
    write_imtq_command(cmd_nbytes);
    // read the response
    read_imtq_response(rsp_nbytes);
    // translate response bytes into response struct
    imtq_common->cc = response[0];
    imtq_common->stat = response[1];
    extract_slong(2,  &imtq_mtm->x);
    extract_slong(6,  &imtq_mtm->y);
    extract_slong(10, &imtq_mtm->z);
    imtq_mtm->coilact = response[14];
}

// get coil current data
void imtq_get_coil_current(imtq_resp_common* imtq_common, imtq_resp_coilcur* imtq_coilcur)
{
    int cmd_id = 0x44;                // command id
    int cmd_nbytes = 1;               // number of bytes in command
    int rsp_nbytes = 8;               // number of bytes in response
    
    // fill the command bytes
    command[0] = cmd_id;
    // send the command
    write_imtq_command(cmd_nbytes);
    // read the response
    read_imtq_response(rsp_nbytes);
    // translate response bytes into response struct
    imtq_common->cc = response[0];
    imtq_common->stat = response[1];
    extract_sshort(2,  &imtq_coilcur->x);
    extract_sshort(4,  &imtq_coilcur->y);
    extract_sshort(6,  &imtq_coilcur->z);
}

// get MTM integration time parameter
void imtq_get_mtm_integ(imtq_resp_common* imtq_common, imtq_resp_integ* imtq_integ)
{
    int cmd_id = 0x81;
    int cmd_nbytes = 3;
    int rsp_nbytes = 5;
    // parameter ID for the internal MTM integration time
    int param_id = 0x2003;
    
    // fill the command bytes
    command[0] = cmd_id;
    // little-endian copy
    command[1] = param_id & 0x00FF;
    command[2] = (param_id & 0xFF00) >> 8;
    
    // send the command
    write_imtq_command(cmd_nbytes);
    // read the response
    read_imtq_response(rsp_nbytes);
    // translate response bytes into response struct
    imtq_common->cc = response[0];
    imtq_common->stat = response[1];
    extract_ushort(2,  &imtq_integ->id);
    imtq_integ->val = response[4];    
}

// set MTM integration time parameter
void imtq_set_mtm_integ(imtq_resp_common* imtq_common, imtq_resp_integ* imtq_integ, int val)
{
    int cmd_id = 0x82;
    int cmd_nbytes = 4;
    int rsp_nbytes = 5;
    // parameter ID for the internal MTM integration time
    int param_id = 0x2003;
    
    // fill the command bytes
    command[0] = cmd_id;
    // little-endian copy of parameter ID
    command[1] = param_id & 0x00FF;
    command[2] = (param_id & 0xFF00) >> 8;
    // set parameter value
    command[3] = val;
    
    // send the command
    write_imtq_command(cmd_nbytes);
    // read the response
    read_imtq_response(rsp_nbytes);
    // translate response bytes into response struct
    imtq_common->cc = response[0];
    imtq_common->stat = response[1];
    extract_ushort(2,  &imtq_integ->id);
    imtq_integ->val = response[4];    
}

// start a self-test, with a specified axis parameter
void imtq_start_selftest(int axis_id, imtq_resp_common* imtq_common)
{
    int cmd_id = 0x08;                // command id
    int cmd_nbytes = 2;               // number of bytes in command
    int rsp_nbytes = 2;               // number of bytes in response
    
    // fill the command bytes
    command[0] = cmd_id;
    command[1] = axis_id;
    // send the command
    write_imtq_command(cmd_nbytes);
    // read the response
    read_imtq_response(rsp_nbytes);
    // translate response bytes into response struct
    imtq_common->cc = response[0];
    imtq_common->stat = response[1];    
}

// read the result of a single-axis self-test
void imtq_get_selftest(imtq_resp_common* imtq_common_init,
        imtq_resp_common* imtq_common_test, imtq_resp_common* imtq_common_fina,
        imtq_resp_selftest* imtq_selftest_init, imtq_resp_selftest* imtq_selftest_test,
        imtq_resp_selftest* imtq_selftest_fina)
{
    int cmd_id = 0x47;                // command id
    int cmd_nbytes = 1;               // number of bytes in command
    int rsp_nbytes = 120;             // number of bytes in response
    
    int off;                          // offset for the response bytes
    
    // fill the command bytes
    command[0] = cmd_id;
    // send the command
    write_imtq_command(cmd_nbytes);
    // read the response
    read_imtq_response(rsp_nbytes);
    // translate response bytes into response struct
    // get the first 40 bytes of response as init step
    off = 0;
    imtq_common_init->cc = response[off+0];
    imtq_common_init->stat = response[off+1];
    imtq_selftest_init->err = response[off+2];
    imtq_selftest_init->step = response[off+3];
    extract_slong(off+4,  &imtq_selftest_init->raw_x);
    extract_slong(off+8,  &imtq_selftest_init->raw_y);
    extract_slong(off+12, &imtq_selftest_init->raw_z);
    extract_slong(off+16, &imtq_selftest_init->cal_x);
    extract_slong(off+20, &imtq_selftest_init->cal_y);
    extract_slong(off+24, &imtq_selftest_init->cal_z);
    extract_sshort(off+28, &imtq_selftest_init->cur_x);
    extract_sshort(off+30, &imtq_selftest_init->cur_y);
    extract_sshort(off+32, &imtq_selftest_init->cur_z);
    extract_sshort(off+34, &imtq_selftest_init->temp_x);
    extract_sshort(off+36, &imtq_selftest_init->temp_y);
    extract_sshort(off+38, &imtq_selftest_init->temp_z);
    
    // get the second 40 bytes as the test step
    off = 40;
    imtq_common_test->cc = response[off+0];
    imtq_common_test->stat = response[off+1];
    imtq_selftest_test->err = response[off+2];
    imtq_selftest_test->step = response[off+3];
    extract_slong(off+4,  &imtq_selftest_test->raw_x);
    extract_slong(off+8,  &imtq_selftest_test->raw_y);
    extract_slong(off+12, &imtq_selftest_test->raw_z);
    extract_slong(off+16, &imtq_selftest_test->cal_x);
    extract_slong(off+20, &imtq_selftest_test->cal_y);
    extract_slong(off+24, &imtq_selftest_test->cal_z);
    extract_sshort(off+28, &imtq_selftest_test->cur_x);
    extract_sshort(off+30, &imtq_selftest_test->cur_y);
    extract_sshort(off+32, &imtq_selftest_test->cur_z);
    extract_sshort(off+34, &imtq_selftest_test->temp_x);
    extract_sshort(off+36, &imtq_selftest_test->temp_y);
    extract_sshort(off+38, &imtq_selftest_test->temp_z);
    
    // get the third 40 bytes as the final step
    off = 80;
    imtq_common_fina->cc = response[off+0];
    imtq_common_fina->stat = response[off+1];
    imtq_selftest_fina->err = response[off+2];
    imtq_selftest_fina->step = response[off+3];
    extract_slong(off+4,  &imtq_selftest_fina->raw_x);
    extract_slong(off+8,  &imtq_selftest_fina->raw_y);
    extract_slong(off+12, &imtq_selftest_fina->raw_z);
    extract_slong(off+16, &imtq_selftest_fina->cal_x);
    extract_slong(off+20, &imtq_selftest_fina->cal_y);
    extract_slong(off+24, &imtq_selftest_fina->cal_z);
    extract_sshort(off+28, &imtq_selftest_fina->cur_x);
    extract_sshort(off+30, &imtq_selftest_fina->cur_y);
    extract_sshort(off+32, &imtq_selftest_fina->cur_z);
    extract_sshort(off+34, &imtq_selftest_fina->temp_x);
    extract_sshort(off+36, &imtq_selftest_fina->temp_y);
    extract_sshort(off+38, &imtq_selftest_fina->temp_z);
}

// start PWM actuation
void imtq_start_actpwm(imtq_resp_common* imtq_common, signed short pwm_x, signed short pwm_y,
        signed short pwm_z, unsigned short dur)
{
    int cmd_id = 0x07;
    int cmd_nbytes = 9;
    int rsp_nbytes = 2;
    
    // fill the command bytes
    command[0] = cmd_id;
    // little-endian copy of pwm_x
    command[1] = pwm_x & 0x00FF;
    command[2] = (pwm_x & 0xFF00) >> 8;
    // little-endian copy of pwm_y
    command[3] = pwm_y & 0x00FF;
    command[4] = (pwm_y & 0xFF00) >> 8;
    // little-endian copy of pwm_z
    command[5] = pwm_z & 0x00FF;
    command[6] = (pwm_z & 0xFF00) >> 8;
    // little-endian copy of duration
    command[7] = dur & 0x00FF;
    command[8] = (dur & 0xFF00) >> 8;
    
    // send the command
    write_imtq_command(cmd_nbytes);
    // read the response
    read_imtq_response(rsp_nbytes);
    // translate response bytes into response struct
    imtq_common->cc = response[0];
    imtq_common->stat = response[1];
}

// start B-Dot detumble operation
void imtq_start_detumble(unsigned short nseconds, imtq_resp_common* imtq_common)
{
    int cmd_id = 0x09;                // command id
    int cmd_nbytes = 3;               // number of bytes in command
    int rsp_nbytes = 2;               // number of bytes in response
    
    // fill the command bytes
    command[0] = cmd_id;
    command[1] = nseconds & 0x00ff;
    command[2] = (nseconds & 0xff00) >> 8;
    
    // send the command
    write_imtq_command(cmd_nbytes);
    // read the response
    read_imtq_response(rsp_nbytes);
    // translate response bytes into response struct
    imtq_common->cc = response[0];
    imtq_common->stat = response[1];
}

// get current detumble data
void imtq_get_detumble_data(imtq_resp_common* imtq_common, imtq_resp_detumble* imtq_detumble)
{
    int cmd_id = 0x48;                // command id
    int cmd_nbytes = 1;               // number of bytes in command
    int rsp_nbytes = 56;             // number of bytes in response

    // fill the command bytes
    command[0] = cmd_id;
    // send the command
    write_imtq_command(cmd_nbytes);
    // read the response
    read_imtq_response(rsp_nbytes);
    
    // translate response bytes into response structs
    imtq_common->cc = response[0];
    imtq_common->stat = response[1];
    extract_slong(2,  &imtq_detumble->cal_x);
    extract_slong(6,  &imtq_detumble->cal_y);
    extract_slong(10, &imtq_detumble->cal_z);
    extract_slong(14, &imtq_detumble->filt_x);
    extract_slong(18, &imtq_detumble->filt_y);
    extract_slong(22, &imtq_detumble->filt_z);
    extract_slong(26, &imtq_detumble->bdot_x);
    extract_slong(30, &imtq_detumble->bdot_y);
    extract_slong(34, &imtq_detumble->bdot_z);
    extract_sshort(38, &imtq_detumble->dip_x);
    extract_sshort(40, &imtq_detumble->dip_y);
    extract_sshort(42, &imtq_detumble->dip_z);
    extract_sshort(44, &imtq_detumble->ccur_x);
    extract_sshort(46, &imtq_detumble->ccur_y);
    extract_sshort(48, &imtq_detumble->ccur_z);
    extract_sshort(50, &imtq_detumble->cur_x);
    extract_sshort(52, &imtq_detumble->cur_y);
    extract_sshort(54, &imtq_detumble->cur_z);
}

