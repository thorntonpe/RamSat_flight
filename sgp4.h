/* 
 * File: sgp4.h   
 * Author: Peter Thornton
 * Comments: copied relevant parts from Bill Gray's sat_code/norad.h
 * Revision history: Created 27 May 2019
 */

#define N_SGP4_PARAMS         30
#define N_SAT_PARAMS          N_SGP4_PARAMS

/* Two-line-element satellite orbital data */
typedef struct
{
  double epoch, xndt2o, xndd6o, bstar;
  double xincl, xnodeo, eo, omegao, xmo, xno;
  int norad_number, bulletin_number, revolution_number;
  char classification;    /* "U" = unclassified;  only type I've seen */
  char ephemeris_type;
  char intl_desig[9];
} tle_t;

   /* NOTE: xndt2o and xndt6o are used only in the "classic" SGP, */
   /* not in SxP4 or SxP8. */
   /* epoch is a Julian Day,  UTC */
   /* xmo = mean anomaly at epoch,  radians */
   /* xno = mean motion at epoch,  radians/minute*/

// deep_arg_t data structure, internal to SGP4
typedef struct
{
  double
  /* Common between SGP4 and SDP4: */
  aodp, cosio, sinio, omgdot, xmdot, xnodot, xnodp,
  /* Used by dpinit part of Deep() */
  eosq, betao, cosio2, sing, cosg, betao2,

  /* Used by dpsec and dpper parts of Deep() */
  xll, omgadf, xnode, em, xinc, xn, t,

       /* 'd####' secular coeffs for 12-hour, e>.5 orbits: */
   d2201, d2211, d3210, d3222, d4410, d4422, d5220, d5232, d5421, d5433,
      /* formerly static to Deep( ),   but more logically part of this struct: */
   atime, del1, del2, del3, e3, ee2, omegaq, pe, pgh, ph, pinc, pl, preep,
   savtsn, se2, se3, sgh2, sgh3, sgh4, sh2, sh3, si2, si3, sl2, sl3,
   sl4, sse, ssg, ssh, ssi, ssl, thgr, xfact, xgh2, xgh3, xgh4, xh2,
   xh3, xi2, xi3, xl2, xl3, xl4, xlamo, xli, xni, xnq, xqncl, zcosgl,
   zcoshl, zcosil, zmol, zmos, zsingl, zsinhl, zsinil;

   int resonance_flag, synchronous_flag;
} deep_arg_t;

// another data structure internal to SGP4
typedef struct
{
   double coef, coef1, tsi, s4, unused_a3ovk2, eta;
} init_t;

/* Table of constant values */
#define pi               3.141592653589793238462643383279502884197
#define twopi            (pi*2.)
#define e6a              1.0E-6
#define two_thirds       (2. / 3.)
#define xj3             -2.53881E-6
#define minus_xj3        2.53881E-6
#define earth_radius_in_km           6378.135
#ifndef minutes_per_day
   #define minutes_per_day  1440.
#endif
#define ae                  1.0
#define xj2                 1.082616e-3
#define ck2                 (.5 * xj2 * ae * ae)
#define xj4      (-1.65597e-6)
#define ck4      (-.375 * xj4 * ae * ae * ae * ae)
#define s_const  (ae * (1. + 78. / earth_radius_in_km))
#define qoms2t   1.880279159015270643865e-9
#define xke      0.0743669161331734132
#define a3ovk2   (minus_xj3/ck2*ae*ae*ae)


// function prototypes
void SGP4_init( double *params, const tle_t *tle);
void sxpx_common_init( double *params, const tle_t *tle,
                                  init_t *init, deep_arg_t *deep_arg);
void sxpall_common_init( const tle_t *tle, deep_arg_t *deep_arg);
int SGP4( const double tsince, const tle_t *tle, const double *params,
                                                    double *pos, double *vel);
int sxpx_posn_vel( const double xnode, const double a, const double ecc,
      const double cosio, const double sinio,
      const double xincl, const double omega,
      const double xl, double *pos, double *vel);
double centralize_angle( const double ival);
double ThetaG( double jd);
void sat_lon_lat_elev( double jd, double *pos, double *lon, double *lat, double *elev, double *lst);
