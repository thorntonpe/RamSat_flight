/* 
 * File:             he100.h
 * Author:           Peter Thornton
 * Comments:         Interface to He-100 VHF/UHF transceiver
 * Revision history: 3/2/2020
 */

#define HEAD_NBYTES 16   // Number of extra bytes at head of received packet
#define TAIL_NBYTES  5   // Number of extra bytes at tail of received packet

void he100_checksum(unsigned char *buf, int nbytes);
int he100_noop(unsigned char* response);
int he100_telemetry(unsigned char* telem_raw);
void he100_transmit_packet(unsigned char* response, char* data);

// telemetry data structures
struct he100_telem_struct {
    unsigned int op_counter;
    signed int msp430_temp;
    unsigned char time_count[3];
    unsigned char rssi;
    unsigned long int bytes_received;
    unsigned long int bytes_transmitted;
};

union he100_telem_union {
    unsigned char raw[16];
    struct he100_telem_struct telem;
};
