/* 
 * File:        ants.h
 * Author:      Peter Thornton
 * Purpose:     Data structures and function prototypes for ANTs interface
 * Created on:  6 June 2020
 */

// Interface to the ISIS ANTs commands, via I2C
// See the ANTs Operation Manual for details on commands and return values


void write_ants_command(int nbytes);
void write_ants_command_noack(int nbytes);
void read_ants_response(int nbytes);
void ants_reset();
void ants_deploy_status(unsigned char* ants_resp);
void ants_arm();
void ants_disarm();
void ants_deploy_all();
void ants_time_1(unsigned char* ants_resp);
void ants_time_2(unsigned char* ants_resp);
void ants_time_3(unsigned char* ants_resp);
void ants_time_4(unsigned char* ants_resp);


