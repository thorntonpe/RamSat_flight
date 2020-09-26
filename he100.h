/* 
 * File:             he100.h
 * Author:           Peter Thornton
 * Comments:         Interface to He-100 VHF/UHF transceiver
 * Revision history: 3/2/2020
 */

void he100_checksum(unsigned char *buf, int nbytes);
int he100_noop(unsigned char* response);
int he100_telemetry(unsigned char* telem_raw);
void he100_transmit_test_msg1(unsigned char* response, float batv);
void he100_transmit_test_msg2(unsigned char* response, char* msg);
void he100_transmit_test_msg3(unsigned char* response);

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
