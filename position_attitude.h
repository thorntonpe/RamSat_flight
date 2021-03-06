/* 
 * File:       position_attitude.h
 * Author:     Peter Thornton
 * Purpose:    data structures for RamSat position and attitude
 * Created:    2 January 2021
 */

// constants
#define PI 3.141592653589793238462643383279502884197

// RamSat position and attitude information at specified JD
typedef struct
{
    double jd;          // Julian date
    double t_since;     // time since epoch of current TLE (minutes)
    double px_eci;      // RamSat position X coordinate (km, ECI)
    double py_eci;      // RamSat position Y coordinate (km, ECI)
    double pz_eci;      // RamSat position Z coordinate (km, ECI)
    double upx_eci;     // RamSat position X (unit vector, ECI)
    double upy_eci;     // RamSat position Y (unit vector, ECI)
    double upz_eci;     // RamSat position Z (unit vector, ECI)
    double lon;         // RamSat longitude (degrees, geocentric, +/- 180, 0=Greenwich, -=west, +=east)
    double lat;         // RamSat latitude (degrees, geocentric, +/- 90, 0=equator, -=south, +=north)
    double cor_lat;     // RamSat latitude corrected for ellipsoid (degrees, geodetic, +/- 90)
    double elev;        // RamSat elevation (meters above mean sea level)
    double lst;         // RamSat local sidereal time (degrees, 0 to 360)
    double decimal_year;// decimal year, used for WMM alculation
    double B_locx;      // WMM magnetic field X in local tangential coordinates (nT)
    double B_locy;      // WMM magnetic field Y in local tangential coordinates (nT)
    double B_locz;      // WMM magnetic field Z in local tangential coordinates (nT)
    double bx_eci;      // WMM magnetic field X in ECI coordinates (nT)
    double by_eci;      // WMM magnetic field Y in ECI coordinates (nT)
    double bz_eci;      // WMM magnetic field Z in ECI coordinates (nT)
    double ubx_eci;     // WMM magnetic field X (unit vector, ECI)
    double uby_eci;     // WMM magnetic field Y (unit vector, ECI)
    double ubz_eci;     // WMM magnetic field Z (unit vector, ECI)
    double bx_body;     // iMTQ magnetic field X in Frame coordinates (nT)
    double by_body;     // iMTQ magnetic field Y in Frame coordinates (nT)
    double bz_body;     // iMTQ magnetic field Z in Frame coordinates (nT)
    double ubx_body;    // iMTQ unit mag vector X in frame coordinates
    double uby_body;    // iMTQ unit mag vector Y in frame coordinates
    double ubz_body;    // iMTQ unit mag vector Z in frame coordinates
    double sx_eci;      // sun position X (km, ECI coordinates)
    double sy_eci;      // sun position Y (km, ECI coordinates)
    double sz_eci;      // sun position Z (km, ECI coordinates)
    double usx_eci;     // sun position X (unit vector, ECI)
    double usy_eci;     // sun position Y (unit vector, ECI)
    double usz_eci;     // sun position Z (unit vector, ECI)
    double sxybodymag_max; // maximum magnitude for sun sensor readings in body X-Y plane
    double sx_body;     // unit sun vector X in frame coordinates
    double sy_body;     // unit sun vector Y in frame coordinates
    double sz_body;     // unit sun vector Z in frame coordinates
    double usx_body;    // unit sun vector X in frame coordinates
    double usy_body;    // unit sun vector Y in frame coordinates
    double usz_body;    // unit sun vector Z in frame coordinates
    double cos_res;     // cosine of RamSat-Earth-Sun angle (+=sunlit)
    double res;         // RamSat-Earth-Sun angle (degrees)
    double dtime;       // time increment between pq1 and pq2
    double offnadir_angle; // angle (degrees) between +Z axis and nadir-pointing vector (body)
    double pq1[4];      // position quaternion #1
    double pq2[4];      // position quaternion #2
    double dq[4];       // desired quaternion
    double omega[3];    // angular velocity (rad/s)
    double dipole[3];   // desired dipole (A m^2)
    double torque[3];   // torque (N m)
    double nadir_body[3]; // nadir-pointing unit vector in body coordinate
    signed short pwm_x;       // PWM X actuation level (-1000 to 1000)
    signed short pwm_y;       // PWM Y actuation level (-1000 to 1000)
    signed short pwm_z;       // PWM Z actuation level (-1000 to 1000)
    unsigned char imtq_cc;  // return from PWM actuation
    unsigned char imtq_stat;// return from PWM actuation
    int isAutoPWM;      // flag for PWM actuation

} position_attitude_type;




