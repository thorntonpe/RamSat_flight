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

void he100_noop(unsigned char* response)
{
    int i;
    unsigned char buf[8];
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
        *response++ = read_char2();
    }
    *response = read_char2();
}

void he100_transmit_test_msg1(unsigned char* response)
{
    int i;
    unsigned char buf[250];
    unsigned char msg_len;
    int packet_len;
    const char test_msg[] = "All work and no play makes Peter a dull boy.";
    
    // length of test message
    msg_len = (unsigned char)strlen(test_msg);
    // packet length = header(8) + msg_len + checksum(2)
    packet_len = 8 + msg_len + 2;
    
    // fill the header
    buf[0]=0x48;    // 'H'
    buf[1]=0x65;    // 'e'
    buf[2]=0x10;    // command prefix (10 = send)
    buf[3]=0x03;    // command code (03 = transmit)
    buf[4]=0x00;    // payload size, MSB
    buf[5]=msg_len; // payload size, LSB
    // set the CRC bytes for header (skip first two bytes))
    he100_checksum(&buf[2],4);
    
    // fill the payload
    for (i=0 ; i<msg_len ; i++)
    {
        buf[8+i]=test_msg[i];
    }
    // checksum for entire packet (skip first two bytes of header)
    he100_checksum(&buf[2],6+msg_len);
    
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

void he100_transmit_test_msg2(unsigned char* response)
{
    int i;
    unsigned char buf[250];
    unsigned char msg_len;
    int packet_len;
    const char test_msg[] = "Got an ack/nack!";
    
    // length of test message
    msg_len = (unsigned char)strlen(test_msg);
    // packet length = header(8) + msg_len + checksum(2)
    packet_len = 8 + msg_len + 2;
    
    // fill the header
    buf[0]=0x48;    // 'H'
    buf[1]=0x65;    // 'e'
    buf[2]=0x10;    // command prefix (10 = send)
    buf[3]=0x03;    // command code (03 = transmit)
    buf[4]=0x00;    // payload size, MSB
    buf[5]=msg_len; // payload size, LSB
    // set the CRC bytes for header (skip first two bytes))
    he100_checksum(&buf[2],4);
    
    // fill the payload
    for (i=0 ; i<msg_len ; i++)
    {
        buf[8+i]=test_msg[i];
    }
    // checksum for entire packet (skip first two bytes of header)
    he100_checksum(&buf[2],6+msg_len);
    
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

void he100_transmit_test_msg3(unsigned char* response)
{
    int i;
    unsigned char buf[250];
    unsigned char msg_len;
    int packet_len;
    const char test_msg[] = "Got a data packet!";
    
    // length of test message
    msg_len = (unsigned char)strlen(test_msg);
    // packet length = header(8) + msg_len + checksum(2)
    packet_len = 8 + msg_len + 2;
    
    // fill the header
    buf[0]=0x48;    // 'H'
    buf[1]=0x65;    // 'e'
    buf[2]=0x10;    // command prefix (10 = send)
    buf[3]=0x03;    // command code (03 = transmit)
    buf[4]=0x00;    // payload size, MSB
    buf[5]=msg_len; // payload size, LSB
    // set the CRC bytes for header (skip first two bytes))
    he100_checksum(&buf[2],4);
    
    // fill the payload
    for (i=0 ; i<msg_len ; i++)
    {
        buf[8+i]=test_msg[i];
    }
    // checksum for entire packet (skip first two bytes of header)
    he100_checksum(&buf[2],6+msg_len);
    
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
