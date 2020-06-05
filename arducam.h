/*
 * File:        arducam.h
 * Author:      Peter Thornton
 * Purpose:     Data structures and function prototypes for Arducam interface
 * Created on:  17 November 2019
 */

// chip select indices for camera #1 and camera #2
#define CS_CAM1 _RD12  // chip select for ArduCam #1, CSKB H1.6, IO.18, Port D.12
#define CS_CAM2 _RE7   // chip select for Arducam #2, CSKB H1.1, IO.23, Port E.7

// define a structure for sensor register values
struct sensor_reg {
    int reg;
    int val;
};

// function prototypes
void write_ov2640_reg_cam1(int reg, int val);
void write_ov2640_reg_cam2(int reg, int val);
void read_ov2640_reg_cam1(int reg, int *val);
void read_ov2640_reg_cam2(int reg, int *val);
void reset_ov2640_regs_cam1();
void reset_ov2640_regs_cam2();
void init_ov2640_regs_cam1(const struct sensor_reg reglist[]);
void init_ov2640_regs_cam2(const struct sensor_reg reglist[]);
void arduchip_write_reg(int reg, int val, int ss);
int arduchip_read_reg(int reg, int ss);
int arduchip_testreg(int testin, int ss);
void arduchip_reset(int ss);
void arduchip_clear_fifo(int ss);
void arduchip_set_nframes(int n, int ss);
long arduchip_start_capture(int ss);
int arduchip_capture_done(int ss);
long arduchip_fifo_length(int *outlen1, int *outlen2, int *outlen3, int ss);


