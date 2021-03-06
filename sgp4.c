/*
 * File:       sgp4.c
 * Author:     Peter Thornton (based on ...)
 * Purpose:    Orbital prediction algorithms
 * Created on: 27 October 2020
 *  
*/

#include "xc.h"
#include "sgp4.h"
#include <math.h>
#include <stdio.h>

#define c2             params[0]
#define c1             params[2]
#define c4             params[3]
#define xnodcf         params[4]
#define t2cof          params[5]
#define p_aodp         params[10]
#define p_cosio        params[11]
#define p_sinio        params[12]
#define p_omgdot       params[13]
#define p_xmdot        params[14]
#define p_xnodot       params[15]
#define p_xnodp        params[16]
#define c5             params[17]
#define d2             params[18]
#define d3             params[19]
#define d4             params[20]
#define delmo          params[21]
#define p_eta          params[22]
#define omgcof         params[23]
#define sinmo          params[24]
#define t3cof          params[25]
#define t4cof          params[26]
#define t5cof          params[27]
#define xmcof          params[28]
#define simple_flag *((int *)( params + 29))
#define MINIMAL_E    1.e-4
#define ECC_EPS      1.e-6     /* Too low for computing further drops. */

void SGP4_init( double *params, const tle_t *tle)
{
   deep_arg_t deep_arg;
   init_t init;
   double eeta, etasq;

   sxpx_common_init( params, tle, &init, &deep_arg);
   p_aodp =   deep_arg.aodp;
   p_cosio =  deep_arg.cosio;
   p_sinio =  deep_arg.sinio;
   p_omgdot = deep_arg.omgdot;
   p_xmdot =  deep_arg.xmdot;
   p_xnodot = deep_arg.xnodot;
   p_xnodp =  deep_arg.xnodp;
   p_eta = deep_arg.aodp*tle->eo*init.tsi;
// p_eta = init.eta;

   eeta = tle->eo*p_eta;
   /* For perigee less than 220 kilometers, the "simple" flag is set */
   /* and the equations are truncated to linear variation in sqrt a  */
   /* and quadratic variation in mean anomaly.  Also, the c3 term,   */
   /* the delta omega term, and the delta m term are dropped.        */
   simple_flag = ((p_aodp*(1-tle->eo)/ae) < (220./earth_radius_in_km+ae));
   if( !simple_flag)
      {
      const double c1sq = c1*c1;
      double temp;

      simple_flag = 0;
      delmo = 1. + p_eta * cos(tle->xmo);
      delmo *= delmo * delmo;
      d2 = 4*p_aodp*init.tsi*c1sq;
      temp = d2*init.tsi*c1/3;
      d3 = (17*p_aodp+init.s4)*temp;
      d4 = 0.5*temp*p_aodp*init.tsi*(221*p_aodp+31*init.s4)*c1;
      t3cof = d2+2*c1sq;
      t4cof = 0.25*(3*d3+c1*(12*d2+10*c1sq));
      t5cof = 0.2*(3*d4+12*c1*d3+6*d2*d2+15*c1sq*(2*d2+c1sq));
      sinmo = sin(tle->xmo);
      if( tle->eo < MINIMAL_E)
         omgcof = xmcof = 0.;
      else
         {
         const double c3 =
              init.coef * init.tsi * a3ovk2 * p_xnodp * ae * p_sinio / tle->eo;

         xmcof = -two_thirds * init.coef * tle->bstar * ae / eeta;
         omgcof = tle->bstar*c3*cos(tle->omegao);
         }
      } /* End of if (isFlagClear(SIMPLE_FLAG)) */
   etasq = p_eta * p_eta;
   c5 = 2*init.coef1*p_aodp * deep_arg.betao2*(1+2.75*(etasq+eeta)+eeta*etasq);
} /* End of SGP4() initialization */

void sxpx_common_init( double *params, const tle_t *tle,
                                  init_t *init, deep_arg_t *deep_arg)
{
   double
         eeta, etasq, perige, pinv, pinvsq,
         psisq, qoms24, temp1, temp2, temp3,
         cosio4, tsi_squared, x3thm1, xhdot1;

   sxpall_common_init( tle, deep_arg);
   x3thm1 = 3. * deep_arg->cosio2 - 1.;
   /* For perigee below 156 km, the values */
   /* of s and qoms2t are altered.         */
   init->s4 = s_const;
   qoms24 = qoms2t;
   perige = (deep_arg->aodp * (1-tle->eo) - ae) * earth_radius_in_km;
   if( perige < 156.)
      {
      double temp_val, temp_val_squared;

      if(perige <= 98.)
         init->s4 = 20;
      else
         init->s4 = perige-78.;
      temp_val = (120. - init->s4) * ae / earth_radius_in_km;
      temp_val_squared = temp_val * temp_val;
      qoms24 = temp_val_squared * temp_val_squared;
      init->s4 = init->s4 / earth_radius_in_km + ae;
      }  /* End of if(perige <= 156) */

   pinv = 1. / (deep_arg->aodp * deep_arg->betao2);
   pinvsq = pinv * pinv;
   init->tsi = 1. / (deep_arg->aodp - init->s4);
   init->eta = deep_arg->aodp*tle->eo*init->tsi;
   etasq = init->eta*init->eta;
   eeta = tle->eo*init->eta;
   psisq = fabs(1-etasq);
   tsi_squared = init->tsi * init->tsi;
   init->coef = qoms24 * tsi_squared * tsi_squared;
   init->coef1 = init->coef / pow(psisq,3.5);
   c2 = init->coef1 * deep_arg->xnodp * (deep_arg->aodp*(1+1.5*etasq+eeta*
   (4+etasq))+0.75*ck2*init->tsi/psisq*x3thm1*(8+3*etasq*(8+etasq)));
   c1 = tle->bstar*c2;
   deep_arg->sinio = sin(tle->xincl);
   c4 = 2*deep_arg->xnodp*init->coef1*deep_arg->aodp*deep_arg->betao2*
        (init->eta*(2+0.5*etasq)+tle->eo*(0.5+2*etasq)-2*ck2*init->tsi/
        (deep_arg->aodp*psisq)*(-3*x3thm1*(1-2*eeta+etasq*
        (1.5-0.5*eeta))+0.75*(1. - deep_arg->cosio2) *(2*etasq-eeta*(1+etasq))*
        cos(2*tle->omegao)));
   cosio4 = deep_arg->cosio2 * deep_arg->cosio2;
   temp1 = 3*ck2*pinvsq*deep_arg->xnodp;
   temp2 = temp1 * ck2 * pinvsq;
   temp3 = 1.25 * ck4 * pinvsq * pinvsq * deep_arg->xnodp;
   deep_arg->xmdot = deep_arg->xnodp
            + temp1 * deep_arg->betao* x3thm1 / 2.
            + temp2 * deep_arg->betao*
                    (13-78*deep_arg->cosio2+137*cosio4) / 16.;
   deep_arg->omgdot = -temp1 * (1. - 5 * deep_arg->cosio2) / 2.
              + temp2 * (7-114*deep_arg->cosio2+395*cosio4) / 16.
              + temp3 * (3-36*deep_arg->cosio2+49*cosio4);
   xhdot1 = -temp1*deep_arg->cosio;
   deep_arg->xnodot = xhdot1+(temp2*(4-19*deep_arg->cosio2) / 2.
           + 2*temp3*(3-7*deep_arg->cosio2))*deep_arg->cosio;
   xnodcf = 3.5*deep_arg->betao2*xhdot1*c1;
   t2cof = 1.5*c1;
}

void sxpall_common_init( const tle_t *tle, deep_arg_t *deep_arg)
{
   const double a1 = pow(xke / tle->xno, two_thirds);  /* in Earth radii */
   double del1, ao, delo, tval;

   /* Recover original mean motion (xnodp) and   */
   /* semimajor axis (aodp) from input elements. */
   deep_arg->cosio = cos( tle->xincl);
   deep_arg->cosio2 = deep_arg->cosio * deep_arg->cosio;
   deep_arg->eosq = tle->eo*tle->eo;
   deep_arg->betao2 = 1-deep_arg->eosq;
   deep_arg->betao = sqrt(deep_arg->betao2);
   tval = 1.5 * ck2 * (3. * deep_arg->cosio2 - 1.) / (deep_arg->betao * deep_arg->betao2);
   del1 = tval / (a1 * a1);
   ao = a1 * (1. - del1 * (1. / 3. + del1 * ( 1. + 134./81. * del1)));
   delo = tval / (ao * ao);
   deep_arg->xnodp = tle->xno / (1+delo);   /* in radians/minute */
   deep_arg->aodp = ao / (1-delo);
}

int SGP4( const double tsince, const tle_t *tle, const double *params,
                                                    double *pos, double *vel)
{
  double
        a, e, omega, omgadf,
        temp, tempa, tempe, templ, tsq,
        xl, xmdf, xmp, xnoddf, xnode;

  /* Update for secular gravity and atmospheric drag. */
  xmdf = tle->xmo+p_xmdot*tsince;
  omgadf = tle->omegao+p_omgdot*tsince;
  xnoddf = tle->xnodeo+p_xnodot*tsince;
  omega = omgadf;
  xmp = xmdf;
  tsq = tsince*tsince;
  xnode = xnoddf+xnodcf*tsq;
  tempa = 1-c1*tsince;
  tempe = tle->bstar*c4*tsince;
  templ = t2cof*tsq;
  if( !simple_flag)
    {
      const double delomg = omgcof*tsince;
      double delm = 1. + p_eta * cos(xmdf);
      double tcube, tfour;

      delm = xmcof * (delm * delm * delm - delmo);
      temp = delomg+delm;
      xmp = xmdf+temp;
      omega = omgadf-temp;
      tcube = tsq*tsince;
      tfour = tsince*tcube;
      tempa = tempa-d2*tsq-d3*tcube-d4*tfour;
      tempe = tempe+tle->bstar*c5*(sin(xmp)-sinmo);
      templ = templ+t3cof*tcube+tfour*(t4cof+tsince*t5cof);
    }; /* End of if (isFlagClear(SIMPLE_FLAG)) */

  a = p_aodp*tempa*tempa;
  e = tle->eo-tempe;
         /* A highly arbitrary lower limit on e,  of 1e-6: */
  if( e < ECC_EPS)
     e = ECC_EPS;
  xl = xmp+omega+xnode+p_xnodp*templ;
  if( tempa < 0.)       /* force negative a,  to indicate error condition */
     a = -a;
  return( sxpx_posn_vel( xnode, a, e, p_cosio, p_sinio, tle->xincl,
                                          omega, xl, pos, vel));
} /*SGP4*/

#define MAX_KEPLER_ITER 10
#define SXPX_ERR_NEARLY_PARABOLIC         -1
#define SXPX_ERR_NEGATIVE_MAJOR_AXIS      -2
#define SXPX_WARN_ORBIT_WITHIN_EARTH      -3
#define SXPX_WARN_PERIGEE_WITHIN_EARTH    -4
#define SXPX_ERR_NEGATIVE_XN              -5
#define SXPX_ERR_CONVERGENCE_FAIL         -6

int sxpx_posn_vel( const double xnode, const double a, const double ecc,
      const double cosio, const double sinio,
      const double xincl, const double omega,
      const double xl, double *pos, double *vel)
{
  /* Long period periodics */
   const double axn = ecc*cos(omega);
   double temp = 1/(a*(1.-ecc*ecc));
   const double xlcof = .125 * a3ovk2 * sinio * (3+5*cosio)/ (1. + cosio);
   const double aycof = 0.25 * a3ovk2 * sinio;
   const double xll = temp*xlcof*axn;
   const double aynl = temp*aycof;
   const double xlt = xl+xll;
   const double ayn = ecc*sin(omega)+aynl;
   const double elsq = axn*axn+ayn*ayn;
   const double capu = centralize_angle( xlt - xnode);
   const double chicken_factor_on_eccentricity = 1.e-6;
   double epw = capu;
   double temp1, temp2;
   double ecosE, esinE, pl, r;
   double betal;
   double u, sinu, cosu, sin2u, cos2u;
   double rk, uk, xnodek, xinck;
   double sinuk, cosuk, sinik, cosik, sinnok, cosnok, xmx, xmy;
   double sinEPW, cosEPW;
   double ux, uy, uz;
   int i, rval = 0;

/* Dundee changes:  items dependent on cosio get recomputed: */
   const double cosio_squared = cosio * cosio;
   const double x3thm1 = 3.0 * cosio_squared - 1.0;
   const double sinio2 = 1.0 - cosio_squared;
   const double x7thm1 = 7.0 * cosio_squared - 1.0;

                /* Added 29 Mar 2003,  modified 26 Sep 2006:  extremely    */
                /* decayed satellites can end up "orbiting" within the     */
                /* earth.  Eventually,  the semimajor axis becomes zero,   */
                /* then negative.  In that case,  or if the orbit is near  */
                /* to parabolic,  we zero the posn/vel and quit.  If the   */
                /* object has a perigee or apogee indicating a crash,  we  */
                /* just flag it.  Revised 28 Oct 2006.                     */

   if( a < 0.)
      rval = SXPX_ERR_NEGATIVE_MAJOR_AXIS;
   if( elsq > 1. - chicken_factor_on_eccentricity)
      rval = SXPX_ERR_NEARLY_PARABOLIC;
   for( i = 0; i < 3; i++)
      {
      pos[i] = 0.;
      if( vel)
         vel[i] = 0.;
      }
   if( rval)
      return( rval);
   if( a * (1. - ecc) < 1. && a * (1. + ecc) < 1.)   /* entirely within earth */
      rval = SXPX_WARN_ORBIT_WITHIN_EARTH;     /* remember, e can be negative */
   if( a * (1. - ecc) < 1. || a * (1. + ecc) < 1.)   /* perigee within earth */
      rval = SXPX_WARN_PERIGEE_WITHIN_EARTH;
  /* Solve Kepler's' Equation */
   for( i = 0; i < MAX_KEPLER_ITER; i++)
      {
      const double newton_raphson_epsilon = 1e-12;
      double f, fdot, delta_epw;
      int do_second_order_newton_raphson = 1;

      sinEPW = sin( epw);
      cosEPW = cos( epw);
      ecosE = axn * cosEPW + ayn * sinEPW;
      esinE = axn * sinEPW - ayn * cosEPW;
      f = capu - epw + esinE;
      if (fabs(f) < newton_raphson_epsilon) break;
      fdot = 1. - ecosE;
      delta_epw = f / fdot;
      if( !i)
         {
         const double max_newton_raphson = 1.25 * fabs( ecc);

         do_second_order_newton_raphson = 0;
         if( delta_epw > max_newton_raphson)
            delta_epw = max_newton_raphson;
         else if( delta_epw < -max_newton_raphson)
            delta_epw = -max_newton_raphson;
         else
            do_second_order_newton_raphson = 1;
         }
      if( do_second_order_newton_raphson)
         delta_epw = f / (fdot + 0.5*esinE*delta_epw);
                             /* f/(fdot - 0.5*fdotdot * f / fdot) */
      epw += delta_epw;
      }

   if( i == MAX_KEPLER_ITER)
      return( SXPX_ERR_CONVERGENCE_FAIL);

  /* Short period preliminary quantities */
   temp = 1-elsq;
   pl = a*temp;
   r = a*(1-ecosE);
   temp2 = a / r;
   betal = sqrt(temp);
   temp = esinE/(1+betal);
   cosu = temp2 * (cosEPW - axn + ayn * temp);
   sinu = temp2 * (sinEPW - ayn - axn * temp);
   u = atan2( sinu, cosu);
   sin2u = 2*sinu*cosu;
   cos2u = 2*cosu*cosu-1;
   temp1 = ck2 / pl;
   temp2 = temp1 / pl;

  /* Update for short periodics */
   rk = r*(1-1.5*temp2*betal*x3thm1)+0.5*temp1*sinio2*cos2u;
   uk = u-0.25*temp2*x7thm1*sin2u;
   xnodek = xnode+1.5*temp2*cosio*sin2u;
   xinck = xincl+1.5*temp2*cosio*sinio*cos2u;

  /* Orientation vectors */
   sinuk = sin(uk);
   cosuk = cos(uk);
   sinik = sin(xinck);
   cosik = cos(xinck);
   sinnok = sin(xnodek);
   cosnok = cos(xnodek);
   xmx = -sinnok*cosik;
   xmy = cosnok*cosik;
   ux = xmx*sinuk+cosnok*cosuk;
   uy = xmy*sinuk+sinnok*cosuk;
   uz = sinik*sinuk;

  /* Position and velocity */
   pos[0] = rk * ux * earth_radius_in_km;
   pos[1] = rk * uy * earth_radius_in_km;
   pos[2] = rk * uz * earth_radius_in_km;
   if( vel)
      {
      const double rdot = xke * sqrt(a) * esinE / r;
      const double rfdot = xke * sqrt(pl) / r;
      const double xn = xke / (a * sqrt(a));
      const double rdotk = rdot - xn * temp1 * sinio2 * sin2u;
      const double rfdotk = rfdot + xn * temp1 * (sinio2 * cos2u + 1.5 * x3thm1);
      const double vx = xmx * cosuk - cosnok * sinuk;
      const double vy = xmy * cosuk - sinnok * sinuk;
      const double vz = sinik*cosuk;

      vel[0] = (rdotk * ux + rfdotk * vx) * earth_radius_in_km;
      vel[1] = (rdotk * uy + rfdotk * vy) * earth_radius_in_km;
      vel[2] = (rdotk * uz + rfdotk * vz) * earth_radius_in_km;
      }
   return( rval);
} /* sxpx_posn_vel */

double centralize_angle( const double ival)
{
   double rval = fmod( ival, twopi);

   if( rval > pi)
      rval -= twopi;
   else if( rval < - pi)
      rval += twopi;
   return( rval);
}

double ThetaG( double jd)
{
  /* Reference:  The 1992 Astronomical Almanac, page B6. */
  const double omega_E = 1.00273790934;
                   /* Earth rotations per sidereal day (non-constant) */
  const double UT = fmod( jd + .5, 1.);
  const double seconds_per_day = 86400.;
  const double jd_2000 = 2451545.0;   /* 1.5 Jan 2000 = JD 2451545. */
  double t_cen, GMST, rval;

  t_cen = (jd - UT - jd_2000) / 36525.;
  GMST = 24110.54841 + t_cen * (8640184.812866 + t_cen *
                           (0.093104 - t_cen * 6.2E-6));
  GMST = fmod( GMST + seconds_per_day * omega_E * UT, seconds_per_day);
  if( GMST < 0.)
     GMST += seconds_per_day;
  rval = 2. * pi * GMST / seconds_per_day;

  return( rval);
} /*Function thetag*/

void sat_lon_lat_elev( double jd, double *pos, double *lon, double *lat, double *elev, double *lst)
{
    // returns longitude and latitude in radians for the ground track 
    // of a satellite with specified position pos[0]=x, pos[1]=y, pos[2]=z (km)
    // lat = +/- pi/2 radians
    // lon = +/- pi radians ( 0 = Greenwich, - = west, + = east)
    // elev (km) is approximate, assuming spherical Earth
    // lst = local sidereal time (0 to 2*pi radians)
    
    // calculate the offset angle to Greenwich at this jd
    double thetag = ThetaG(jd);
    // dce = distance from center of Earth to satellite
    // rxy = length of position vector projected to x-y plane
    // thetao = angle of orbit in x-y plane (radians))
    double t1 = pos[0]*pos[0] + pos[1]*pos[1];
    double dce = sqrt(t1 + pos[2]*pos[2]);
    double rxy = sqrt(t1);
    double thetao = acos(pos[0]/rxy);
    // get values from pi to 2*pi for negative y in position vector
    if (pos[1] < 0.0)
    {
        thetao = (2.0 * pi) - thetao;
    }
    double tlon = thetao - thetag;
    // if lon less than -180 degrees, wrap around to positive longitude
    if (tlon < -pi)
    {
        tlon = tlon + (2.0*pi);
    }
    if (tlon > pi)
    {
        tlon = tlon - (2.0*pi);
    }
    *lat = asin(pos[2]/dce);
    *lon = tlon;
    *elev = dce - earth_radius_in_km;
    *lst = thetao;
}

