/* 
 * File:        imtq.h
 * Author:      Peter Thornton
 * Purpose:     Data structures and function prototypes for iMTQ interface
 * Created on:  30 August 2019
 */

// Interface to the ISIS iMTQ Magnetorquer commands, via I2C
// See the iMTQ Operation Manual for details on commands and return values

// these two values are returned at the beginning of every command response
typedef struct
{
    unsigned char cc;    // command code      
    unsigned char stat;  // status byte
} imtq_resp_common;

// device state
typedef struct
{
    unsigned char mode;  // iMTQ state (0=idle, 1=selftest, 2=detumble)
    unsigned char err;   // error during previous iteration?
    unsigned char conf;  // any parameter updated since power-on?
    unsigned long int uptime;  // uptime since power-on (s)
} imtq_resp_state;

// magnetometer readings
typedef struct
{
    // calibrated MTM values are in units 1e-9 T
    // raw MTM values are in units 7.5e-9 T
    signed long x;         // X magnetic field
    signed long y;         // Y magnetic field
    signed long z;         // Z magnetic field
    unsigned char coilact; // 0=MTQ not actuating, 1=MTQ actuating 
} imtq_resp_mtm;

// coil current readings
typedef struct
{
    // coil current in units 1e-4 A
    signed short x;         // X coil current
    signed short y;         // Y coil current
    signed short z;         // Z coil current
} imtq_resp_coilcur;

// mtm time integration parameter
typedef struct
{
    unsigned short id;      // returned parameter ID 
    unsigned char val;      // Time integration parameter value
} imtq_resp_integ;

// self-test result, for a single step of the self-test
typedef struct
{
    unsigned char err;      // bit-field with multiple error flags
    unsigned char step;     // result for initial, test, or final step
    signed long raw_x;      // raw X magnetic field
    signed long raw_y;      // raw Y magnetic field
    signed long raw_z;      // raw Z magnetic field
    signed long cal_x;      // calibrated X magnetic field
    signed long cal_y;      // calibrated Y magnetic field
    signed long cal_z;      // calibrated Z magnetic field
    signed short cur_x;     // X coil current
    signed short cur_y;     // Y coil current
    signed short cur_z;     // Z coil current
    signed short temp_x;    // X coil temperature
    signed short temp_y;    // Y coil temperature
    signed short temp_z;    // Z coil temperature
} imtq_resp_selftest;

// Detumble data 
typedef struct
{
    signed long cal_x;      // calibrated X magnetic field (1e-9 T)
    signed long cal_y;      // calibrated Y magnetic field (1e-9 T)
    signed long cal_z;      // calibrated Z magnetic field (1e-9 T)
    signed long filt_x;     // filtered X magnetic field (1e-9 T)
    signed long filt_y;     // filtered Y magnetic field (1e-9 T)
    signed long filt_z;     // filtered Z magnetic field (1e-9 T)
    signed long bdot_x;     // B-dot X (1e-9 T s-1)
    signed long bdot_y;     // B-dot Y (1e-9 T s-1)
    signed long bdot_z;     // B-dot Z (1e-9 T s-1)
    signed short dip_x;     // commanded dipole X (1e-4 Am-2)
    signed short dip_y;     // commanded dipole Y (1e-4 Am-2)
    signed short dip_z;     // commanded dipole Z (1e-4 Am-2)
    signed short ccur_x;    // commanded current X (1e-4 A)
    signed short ccur_y;    // commanded current Y (1e-4 A)
    signed short ccur_z;    // commanded current Z (1e-4 A)
    signed short cur_x;     // measured current X (1e-4 A)
    signed short cur_y;     // measured current Y (1e-4 A)
    signed short cur_z;     // measured current Z (1e-4 A)
} imtq_resp_detumble;

void write_imtq_command(int nbytes);
void write_imtq_command_noack(int nbytes);
void read_imtq_response(int nbytes);
void extract_slong(int start, signed long *result);
void extract_ulong(int start, unsigned long *result);
void extract_sshort(int start, signed short *result);
void imtq_reset();
void imtq_no_op(imtq_resp_common* imtq_common);
void imtq_get_state(imtq_resp_common* imtq_common, imtq_resp_state* imtq_state);
void imtq_start_mtm(imtq_resp_common* imtq_common);
void imtq_get_calib_mtm(imtq_resp_common* imtq_common, imtq_resp_mtm* imtq_mtm);
void imtq_get_raw_mtm(imtq_resp_common* imtq_common, imtq_resp_mtm* imtq_mtm);
void imtq_get_coil_current(imtq_resp_common* imtq_common, imtq_resp_coilcur* imtq_coilcur);
void imtq_get_mtm_integ(imtq_resp_common* imtq_common, imtq_resp_integ* imtq_integ);
void imtq_set_mtm_integ(imtq_resp_common* imtq_common, imtq_resp_integ* imtq_integ, int val);
void imtq_start_selftest(int axis_id, imtq_resp_common* imtq_common);
void imtq_get_selftest(imtq_resp_common* imtq_common_init,
        imtq_resp_common* imtq_common_test, imtq_resp_common* imtq_common_fina,
        imtq_resp_selftest* imtq_selftest_init, imtq_resp_selftest* imtq_selftest_test,
        imtq_resp_selftest* imtq_selftest_fina);
void imtq_start_actpwm(imtq_resp_common* imtq_common, signed short pwm_x, signed short pwm_y,
        signed short pwm_z, unsigned short dur);
void imtq_start_detumble(unsigned short nseconds, imtq_resp_common* imtq_common);
void imtq_get_detumble_data(imtq_resp_common* imtq_common, imtq_resp_detumble* imtq_detumble);


