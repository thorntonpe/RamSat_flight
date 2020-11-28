/*
 * File:       wmm.c
 * Author:     Peter Thornton
 * Created on: 22 June 2019
 * Purpose:    Functions to return X, Y, and Z components of geomagnetic field
 */

#include "xc.h"
#include "wmm.h"
#include <math.h>
#include <stdio.h>


// G, H, dG, dH coefficients for a 12-harmonic model, 
// from WMM-2020 epoch file WMM.COF (12/10/2019)
// the first entry of each array is 0.0, followed by 90 terms
const double coeff_G[91]={0.0,
    -29404.5, -1450.7, -2500.0,  2982.0,  1676.8,
      1363.9, -2381.0,  1236.2,   525.7,   903.1,
       809.4,    86.2,  -309.4,    47.9,  -234.4,
       363.1,   187.8,  -140.7,  -151.2,    13.7,
        65.9,    65.6,    73.0,  -121.5,   -36.2,
        13.5,   -64.7,    80.6,   -76.8,    -8.3,
        56.5,    15.8,     6.4,    -7.2,     9.8,
        23.6,     9.8,   -17.5,    -0.4,   -21.1,
        15.3,    13.7,   -16.5,    -0.3,     5.0,
         8.2,     2.9,    -1.4,    -1.1,   -13.3,
         1.1,     8.9,    -9.3,   -11.9,    -1.9,
        -6.2,    -0.1,     1.7,    -0.9,     0.6,
        -0.9,     1.9,     1.4,    -2.4,    -3.9,
         3.0,    -1.4,    -2.5,     2.4,    -0.9,
         0.3,    -0.7,    -0.1,     1.4,    -0.6,
         0.2,     3.1,    -2.0,    -0.1,     0.5,
         1.3,    -1.2,     0.7,     0.3,     0.5,
        -0.2,    -0.5,     0.1,    -1.1,    -0.3
};
const double coeff_H[91]={0.0,
         0.0,  4652.9,     0.0, -2991.6,  -734.8,
         0.0,   -82.2,   241.8,  -542.9,     0.0,
       282.0,  -158.4,   199.8,  -350.1,     0.0,
        47.7,   208.4,  -121.3,    32.2,    99.1,
         0.0,   -19.1,    25.0,    52.7,   -64.4,
         9.0,    68.1,     0.0,   -51.4,   -16.8,
         2.3,    23.5,    -2.2,   -27.2,    -1.9,
         0.0,     8.4,   -15.3,    12.8,   -11.8,
        14.9,     3.6,    -6.9,     2.8,     0.0,
       -23.3,    11.1,     9.8,    -5.1,    -6.2,
         7.8,     0.4,    -1.5,     9.7,     0.0,
         3.4,    -0.2,     3.5,     4.8,    -8.6,
        -0.1,    -4.2,    -3.4,    -0.1,    -8.8,
         0.0,    -0.0,     2.6,    -0.5,    -0.4,
         0.6,    -0.2,    -1.7,    -1.6,    -3.0,
        -2.0,    -2.6,     0.0,    -1.2,     0.5,
         1.3,    -1.8,     0.1,     0.7,    -0.1,
         0.6,     0.2,    -0.9,    -0.0,     0.5
};
const double coeff_dG[91]={0.0,
         6.7,     7.7,   -11.5,    -7.1,    -2.2,
         2.8,    -6.2,     3.4,   -12.2,    -1.1,
        -1.6,    -6.0,     5.4,    -5.5,    -0.3,
         0.6,    -0.7,     0.1,     1.2,     1.0,
        -0.6,    -0.4,     0.5,     1.4,    -1.4,
        -0.0,     0.8,    -0.1,    -0.3,    -0.1,
         0.7,     0.2,    -0.5,    -0.8,     1.0,
        -0.1,     0.1,    -0.1,     0.5,    -0.1,
         0.4,     0.5,     0.0,     0.4,    -0.1,
        -0.2,    -0.0,     0.4,    -0.3,    -0.0,
         0.3,    -0.0,    -0.0,    -0.4,     0.0,
        -0.0,    -0.0,     0.2,    -0.1,    -0.2,
        -0.0,    -0.1,    -0.2,    -0.1,    -0.0,
        -0.0,    -0.1,    -0.0,     0.0,    -0.0,
        -0.1,     0.0,    -0.0,    -0.1,    -0.1,
        -0.1,    -0.1,     0.0,    -0.0,    -0.0,
         0.0,    -0.0,    -0.0,     0.0,    -0.0,
         0.0,    -0.0,    -0.0,    -0.0,    -0.1
};
const double coeff_dH[91]={0.0,
         0.0,   -25.1,     0.0,   -30.2,   -23.9,
         0.0,     5.7,    -1.0,     1.1,     0.0,
         0.2,     6.9,     3.7,    -5.6,     0.0,
         0.1,     2.5,    -0.9,     3.0,     0.5,
         0.0,     0.1,    -1.8,    -1.4,     0.9,
         0.1,     1.0,     0.0,     0.5,     0.6,
        -0.7,    -0.2,    -1.2,     0.2,     0.3,
         0.0,    -0.3,     0.7,    -0.2,     0.5,
        -0.3,    -0.5,     0.4,     0.1,     0.0,
        -0.3,     0.2,    -0.4,     0.4,     0.1,
        -0.0,    -0.2,     0.5,     0.2,     0.0,
        -0.0,     0.1,    -0.3,     0.1,    -0.2,
         0.1,    -0.0,    -0.1,     0.2,    -0.0,
         0.0,    -0.0,     0.1,     0.0,     0.2,
        -0.0,     0.0,     0.1,    -0.0,    -0.1,
         0.0,    -0.0,     0.0,    -0.0,     0.0,
        -0.1,     0.1,    -0.0,     0.0,    -0.0,
         0.1,    -0.0,    -0.0,     0.0,    -0.1
};
double epoch = 2020.0;  // base year for the coefficients
int nMax = 12;          // number of spherical harmonics

// constants 
double re = 6371.2;  // Earth's radius (km)

// arrays holding the time-corrected coefficients
double timed_G[92];
double timed_H[92];

// array for Legendre function
double Pcup[92];
double dPcup[92];

// arrays for spherical harmonic variables
double RelativeRadiusPower[13];
double cos_mlambda[13];
double sin_mlambda[13];

// local function prototypes
int TimedCoeff(double DecimalYear);
int ComputeSphericalHarmonicVariables(double lambda, double phig, double r);
int AssociatedLegendreFunction(double phig);
int Summation(double phig, double *X, double *Y, double *Z);

// wrapper function called from main program
// DecimalYear = decimal year (valid up to 2020)
// lambda = longitude (radians, W negative)
// phig = geocentric latitude (radians, N positive)
// r = distance from center of Earth (km)
int calc_WMM(double DecimalYear, double lambda, double phig, double r, double *X, double *Y, double *Z)
{
    int ans;
    // update the time varying coefficients
    ans = TimedCoeff(DecimalYear);
    
    // main calculation steps
    ans = ComputeSphericalHarmonicVariables(lambda, phig, r);
    ans = AssociatedLegendreFunction(phig);
    ans = Summation(phig, X, Y, Z);
    
    return 1;
}

// input the current time as decimal year, for coefficient corrections from epoch
int TimedCoeff(double DecimalYear)
{
    int n, m, index; 
    for(n = 1; n <= nMax; n++)
    {
        for(m = 0; m <= n; m++)
        {
            index = (n * (n + 1) / 2 + m);
            timed_H[index] = coeff_H[index] + (DecimalYear - epoch) * coeff_dH[index];
            timed_G[index] = coeff_G[index] + (DecimalYear - epoch) * coeff_dG[index];
        }
    }
    return 1;
}

// Compute the spherical harmonic variables
// lambda = longitude (radians)
// phig = geocentric latitude (radians)
// r = distance from center of Earth (km)
int ComputeSphericalHarmonicVariables(double lambda, double phig, double r)
{
    double cos_lambda, sin_lambda;
    int m, n;
    cos_lambda = cos(lambda);
    sin_lambda = sin(lambda);
    /* for n = 0 ... model_order, compute (Radius of Earth / Spherical radius r)^(n+2)
    for n  1..nMax-1 (this is much faster than calling pow MAX_N+1 times).      */
    RelativeRadiusPower[0] = (re / r) * (re / r);
    for(n = 1; n <= nMax; n++)
    {
        RelativeRadiusPower[n] = RelativeRadiusPower[n - 1] * (re / r);
    }

    /*
     Compute cos(m*lambda), sin(m*lambda) for m = 0 ... nMax
           cos(a + b) = cos(a)*cos(b) - sin(a)*sin(b)
           sin(a + b) = cos(a)*sin(b) + sin(a)*cos(b)
     */
    cos_mlambda[0] = 1.0;
    sin_mlambda[0] = 0.0;

    cos_mlambda[1] = cos_lambda;
    sin_mlambda[1] = sin_lambda;
    for(m = 2; m <= nMax; m++)
    {
        cos_mlambda[m] = cos_mlambda[m - 1] * cos_lambda - sin_mlambda[m - 1] * sin_lambda;
        sin_mlambda[m] = cos_mlambda[m - 1] * sin_lambda + sin_mlambda[m - 1] * cos_lambda;
    }
    return 1;
}

// compute the associated Legendre functions (Pcup)
// phig = geocentric latitude (radians)
int AssociatedLegendreFunction(double phig)
{
    int n, m, index, index1, index2;
    double k, x, z, schmidtQuasiNorm[92];
    
    Pcup[0] = 1.0;
    dPcup[0] = 0.0;
    /*sin (geocentric latitude) - sin_phi */
    x = sin(phig);
    z = sqrt((1.0 - x) * (1.0 + x));

    /*	 First,	Compute the Gauss-normalized associated Legendre  functions*/
    for(n = 1; n <= nMax; n++)
    {
        for(m = 0; m <= n; m++)
        {
            index = (n * (n + 1) / 2 + m);
            if(n == m)
            {
                index1 = (n - 1) * n / 2 + m - 1;
                Pcup [index] = z * Pcup[index1];
                dPcup[index] = z * dPcup[index1] + x * Pcup[index1];
            } else if(n == 1 && m == 0)
            {
                index1 = (n - 1) * n / 2 + m;
                Pcup[index] = x * Pcup[index1];
                dPcup[index] = x * dPcup[index1] - z * Pcup[index1];
            } else if(n > 1 && n != m)
            {
                index1 = (n - 2) * (n - 1) / 2 + m;
                index2 = (n - 1) * n / 2 + m;
                if(m > n - 2)
                {
                    Pcup[index] = x * Pcup[index2];
                    dPcup[index] = x * dPcup[index2] - z * Pcup[index2];
                } else
                {
                    k = (double) (((n - 1) * (n - 1)) - (m * m)) / (double) ((2 * n - 1) * (2 * n - 3));
                    Pcup[index] = x * Pcup[index2] - k * Pcup[index1];
                    dPcup[index] = x * dPcup[index2] - z * Pcup[index2] - k * dPcup[index1];
                }
            }
        }
    }
    /* Compute the ratio between the the Schmidt quasi-normalized associated Legendre
     * functions and the Gauss-normalized version. */

    schmidtQuasiNorm[0] = 1.0;
    for(n = 1; n <= nMax; n++)
    {
        index = (n * (n + 1) / 2);
        index1 = (n - 1) * n / 2;
        /* for m = 0 */
        schmidtQuasiNorm[index] = schmidtQuasiNorm[index1] * (double) (2 * n - 1) / (double) n;

        for(m = 1; m <= n; m++)
        {
            index = (n * (n + 1) / 2 + m);
            index1 = (n * (n + 1) / 2 + m - 1);
            schmidtQuasiNorm[index] = schmidtQuasiNorm[index1] * sqrt((double) ((n - m + 1) * (m == 1 ? 2 : 1)) / (double) (n + m));
        }
    }

    /* Converts the  Gauss-normalized associated Legendre
              functions to the Schmidt quasi-normalized version using pre-computed
              relation stored in the variable schmidtQuasiNorm */

    for(n = 1; n <= nMax; n++)
    {
        for(m = 0; m <= n; m++)
        {
            index = (n * (n + 1) / 2 + m);
            Pcup[index] = Pcup[index] * schmidtQuasiNorm[index];
            dPcup[index] = -dPcup[index] * schmidtQuasiNorm[index];
            /* The sign is changed since the new WMM routines use derivative with respect to latitude
            insted of co-latitude */
        }
    }
    
    return 1;
}

// Computes Geomagnetic Field Elements X, Y and Z 
// in Spherical coordinate system using spherical harmonic summation.
// phig = geocentric latitude (radians)
int Summation(double phig, double *X, double *Y, double *Z)
{
    int m, n, index;
    double Bz = 0.0;
    double By = 0.0;
    double Bx = 0.0;
    
    for(n = 1; n <= nMax; n++)
    {
        for(m = 0; m <= n; m++)
        {
            index = (n * (n + 1) / 2 + m);

            /*		    nMax  	(n+2) 	  n     m            m           m
                    Bz =   -SUM (a/r)   (n+1) SUM  [g cos(m p) + h sin(m p)] P (sin(phi))
                                    n=1      	      m=0   n            n           n  */
            /* Equation 12 in the WMM Technical report.  Derivative with respect to radius.*/
            Bz -= RelativeRadiusPower[n] *
                (timed_G[index] * cos_mlambda[m] + timed_H[index] * sin_mlambda[m])
                * (double) (n + 1) * Pcup[index];

            /*		  1 nMax  (n+2)    n     m            m           m
                    By =    SUM (a/r) (m)  SUM  [g cos(m p) + h sin(m p)] dP (sin(phi))
                               n=1             m=0   n            n           n  */
            /* Equation 11 in the WMM Technical report. Derivative with respect to longitude, divided by radius. */
            By += RelativeRadiusPower[n] *
                (timed_G[index] * sin_mlambda[m] - timed_H[index] * cos_mlambda[m])
                * (double) (m) * Pcup[index];
            /*		   nMax  (n+2) n     m            m           m
                    Bx = - SUM (a/r)   SUM  [g cos(m p) + h sin(m p)] dP (sin(phi))
                               n=1         m=0   n            n           n  */
            /* Equation 10  in the WMM Technical report. Derivative with respect to latitude, divided by radius. */

            Bx -= RelativeRadiusPower[n] *
                (timed_G[index] * cos_mlambda[m] + timed_H[index] * sin_mlambda[m])
                * dPcup[index];
        }
    }
    By = By / cos(phig);

    *X = Bx;
    *Y = By;
    *Z = Bz;

    return 1;
}