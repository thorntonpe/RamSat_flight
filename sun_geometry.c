/*
 * File:       sun_geometry.c
 * Author:     Peter Thornton
 * Purpose:    Predict sun position in ECI and sat-earth-sun angle
 * Created on: 7 February 2021
 *  
*/


#include "xc.h"
#include "sun_geometry.h"
#include <math.h>

void sun_ECI(double jd, double *sunx_eci, double *suny_eci, double *sunz_eci)
{
    // calculate the sun position in ECI coordinates
    double day_n;
    day_n = jd - 2451545.0;
    double deg_to_rad = 0.01745329;
    // calculate the ecliptic longitude of sun position
    double ecl_g, ecl_l, ecl_lon;
    ecl_g = 357.528 + (0.9856003 * day_n);
    ecl_l = 280.460 + (0.9856474 * day_n);
    ecl_g *= deg_to_rad;
    ecl_lon = ecl_l + (1.915 * sin(ecl_g)) + (0.02 * sin(2.0*ecl_g));
    ecl_lon *= deg_to_rad;
    // sun-earth distance, in AU
    double sun_earth_dist = 1.00014 - (0.01671 * cos(ecl_g)) - (0.00014 * cos(2.0*ecl_g));
    // convert from AU to km
    sun_earth_dist *= 149597870.7; // definition of 1 AU in km
    double obliq = 23.43664; // Earth's rotational axis obliquity, in degrees, as of 26 April 2020
    obliq *= deg_to_rad;
    *sunx_eci = sun_earth_dist * cos(ecl_lon);
    *suny_eci = sun_earth_dist * cos(obliq) * sin(ecl_lon);
    *sunz_eci = sun_earth_dist * sin(obliq) * sin(ecl_lon);
}

