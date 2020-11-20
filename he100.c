/* 
 * File:             he100.c
 * Author:           Peter Thornton
 * Comments:         Interface to the He-100 VHF/UHF transceiver, via UART2
 * Revision history: 2 March 2020
 */

#include "xc.h"
#include "he100.h"
#include "uart.h"
#include <string.h>
#include <stdio.h>

// CRC calculation. Stores the two checksum bytes in the two buffer locations
// after pointer+nbytes
void he100_checksum(unsigned char *buf, int nbytes)
{
    int i;
    unsigned char ck_a, ck_b;
    ck_a = 0;
    ck_b = 0;
    for (i=0 ; i<nbytes ; i++)
    {
        ck_a += *buf++;
        ck_b += ck_a;
    }
    *buf++ = ck_a;
    *buf   = ck_b;
}

int he100_noop(unsigned char* response)
{
    int i;
    unsigned char buf[8];
    unsigned char ck_a, ck_b;
    
    // fill the header
    buf[0]=0x48; // 'H'
    buf[1]=0x65; // 'e'
    buf[2]=0x10; // command prefix (10 = send)
    buf[3]=0x01; // command code (01 = no-op)
    buf[4]=0x00; // no payload
    buf[5]=0x00; // no payload
    // set the CRC bytes for header
    he100_checksum(&buf[2],4);
    
    // write 8 bytes to UART2
    for (i=0 ; i<8 ; i++)
    {
        write_char2(buf[i]);
    }
    
    // read 8-byte response from radio board on UART2
    for (i=0 ; i<8 ; i++)
    {
        buf[i] = read_char2();
    }
    
    // copy from buf to response output array
    for (i=0 ; i<8 ; i++)
    {
        *response++ = buf[i];
    }
    
    // test the response checksum
    ck_a = buf[6];
    ck_b = buf[7];
    
    // calculate the response checksum
    he100_checksum(&buf[2],4);
    
    // return 1 if the checksum test fails, 0 otherwise
    return ((ck_a != buf[6]) || (ck_b != buf[7]));
}

int he100_telemetry(unsigned char* telem_raw)
{
    int i;
    unsigned char buf[26]; // buffer big enough for header and telemetry
    unsigned char ck_a, ck_b; // checksum bytes
    int is_err = 0;

    // fill the header for command
    buf[0]=0x48; // 'H'
    buf[1]=0x65; // 'e'
    buf[2]=0x10; // command prefix (10 = send)
    buf[3]=0x07; // command code (07 = retrieve telemetry)
    buf[4]=0x00; // no payload
    buf[5]=0x00; // no payload
    // set the CRC bytes for header
    he100_checksum(&buf[2],4);
    
    // write 8 bytes to UART2
    for (i=0 ; i<8 ; i++)
    {
        write_char2(buf[i]);
    }
    
    // read 26-byte response from radio board on UART2
    // 8 bytes header, 16 bytes telemetry data, 2 bytes checksum
    for (i=0 ; i<26 ; i++)
    {
        buf[i] = read_char2();
    }
    
    // save the checksum, then recalculate
    ck_a = buf[24];
    ck_b = buf[25];
    he100_checksum(&buf[2],22);
    if (ck_a == buf[24] && ck_b == buf[25])
    {
        // good checksum
        // copy the 16-byte telemetry data to response, skip header
        for (i=0 ; i<16 ; i++)
        {
            *telem_raw++ = buf[i+8];
        }
        is_err = 0;
    }
    else
    {
        is_err = 1;
    }
    return is_err;
}

void he100_transmit_packet(unsigned char* response, char* data)
{
    int i;
    unsigned char buf[260];
    unsigned char data_len;
    int packet_len;
    
    // Note that the downlink handling assumes the data array is a null-terminated
    // string.
    // length of data string
    data_len = (unsigned char)strlen(data);
    // If the string is longer than the max allowable payload length (255 bytes)
    // then truncate the end of the string.
    if (data_len > 255) data_len = 255;
    
    // packet length = header(8) + data_len + checksum(2)
    packet_len = 8 + data_len + 2;
    
    // fill the header
    buf[0]=0x48;     // 'H'
    buf[1]=0x65;     // 'e'
    buf[2]=0x10;     // command prefix (10 = send)
    buf[3]=0x03;     // command code (03 = transmit)
    buf[4]=0x00;     // payload size, MSB
    buf[5]=data_len; // payload size, LSB
    // set the CRC bytes for header (skip first two bytes))
    he100_checksum(&buf[2],4);
    
    // fill the payload
    for (i=0 ; i<data_len ; i++)
    {
        buf[8+i]=data[i];
    }
    // checksum for entire packet (skip first two bytes of header)
    he100_checksum(&buf[2],6+data_len);
    
    // write complete packet to UART2 for transmission
    for (i=0 ; i<packet_len ; i++)
    {
        write_char2(buf[i]);
    }
    
    // read 8-byte response from radio board on UART2
    for (i=0 ; i<7 ; i++)
    {
        *response++ = read_char2();
    }
    *response = read_char2();
}

