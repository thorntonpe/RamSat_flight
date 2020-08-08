/* 
 * File:             he100.h
 * Author:           Peter Thornton
 * Comments:         Interface to He-100 VHF/UHF transceiver
 * Revision history: 3/2/2020
 */

void he100_checksum(unsigned char *buf, int nbytes);
void he100_noop(unsigned char* response);
void he100_telemetry(unsigned char* response);
void he100_transmit_test_msg1(unsigned char* response, float batv);
void he100_transmit_test_msg2(unsigned char* response, char* msg);
void he100_transmit_test_msg3(unsigned char* response);
