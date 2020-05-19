/* 
 * File:       i2c.h
 * Author:     Peter Thornton
 * Purpose:    Function prototypes for the PIC24F i2c peripheral
 * Created on: 18 May 2020
 */

void idle_i2c1(void);
void idle_i2c2(void);
void start_i2c1(void);
void start_i2c2(void);
void restart_i2c1(void);
void restart_i2c2(void);
void stop_i2c1(int delay_msec);
void stop_i2c2(int delay_msec);
void transmit_i2c1(int data);
void transmit_i2c2(int data);
void transmit_i2c1_noack(int data);
void transmit_i2c2_noack(int data);
void receive_i2c1(int *data);
void receive_i2c2(int *data);
void receive_i2c1_nack(int *data);
void receive_i2c2_nack(int *data);
void ack_i2c1(void);
void ack_i2c2(void);
void nack_i2c1(void);
void nack_i2c2(void);
long init_i2c1(long fosc, long i2c1br);
long init_i2c2(long fosc, long i2c2br);


