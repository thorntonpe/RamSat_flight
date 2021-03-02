/* 
 * File:       triad.h
 * Author:     Melissa Dumas
 * Purpose:    Function prototypes for triad algorithm 
 * Created on: 12 February 2021 
 */

// function prototypes 
void triad(double *marr, double *sarr, double *iarr, double *jarr, double *pq, double *att);
void rbody(double *att, double *in_vector, double *out_vector);
void desired_q(double *att, double *nadir_eci, double *q);
void rotate(double *dtime, double *pq1, double *pq2, double *dq, double *ub_body, double *omega, double *dipole);





