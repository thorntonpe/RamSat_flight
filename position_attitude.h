/* 
 * File:       position_attitude.h
 * Author:     Peter Thornton
 * Purpose:    data structures for RamSat position and attitude
 * Created:    2 January 2021
 */

// RamSat position and attitude information at specified JD
typedef struct
{
    double jd;          // Julian date
    double t_since;     // time since epoch of current TLE (minutes)
    double x_eci;       // X coordinate (km, ECI)
    double y_eci;       // Y coordinate (km, ECI)
    double z_eci;       // Z coordinate (km, ECI)
    double lon;         // longitude (degrees, geocentric, +/- 180, 0=Greenwich, -=west, +=east)
    double lat;         // latitude (degrees, geocentric, +/- 90, 0=equator, -=south, +=north)
    double cor_lat;     // latitude corrected for ellipsoid (degrees, geodetic, +/- 90)
    double elev;        // elevation (meters above mean sea level)
    double lst;         // local sidereal time (degrees, 0 to 360)
    double B_locx;      // magnetic field X in local tangential coordinates (nT)
    double B_locy;      // magnetic field Y in local tangential coordinates (nT)
    double B_locz;      // magnetic field Z in local tangential coordinates (nT)
    double B_x;         // magnetic field X in ECI coordinates (nT)
    double B_y;         // magnetic field Y in ECI coordinates (nT)
    double B_z;         // magnetic field Z in ECI coordinates (nT)
    double B_fx;        // magnetic field X in Frame coordinates (nT)
    double B_fy;        // magnetic field Y in Frame coordinates (nT)
    double B_fz;        // magnetic field Z in Frame coordinates (nT)
    double b_x;         // unit mag vector X in ECI coordinates
    double b_y;         // unit mag vector Y in ECI coordinates
    double b_z;         // unit mag vector Z in ECI coordinates
    double bf_x;        // unit mag vector X in frame coordinates
    double bf_y;        // unit mag vector Y in frame coordinates
    double bf_z;        // unit mag vector Z in frame coordinates
    double s_x;         // unit sun vector X in ECI coordinates
    double s_y;         // unit sun vector Y in ECI coordinates
    double s_z;         // unit sun vector Z in ECI coordinates
    double sf_x;        // unit sun vector X in frame coordinates
    double sf_y;        // unit sun vector Y in frame coordinates
    double sf_z;        // unit sun vector Z in frame coordinates
    double q0;
    double q1;
    double q2;
    double q3;
} position_attitude_type;




