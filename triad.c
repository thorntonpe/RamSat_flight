/********************************************************************************* 
 * This function reads unit vectors from the magnetometer and the sun sensors    *
 * and generates a matrix of basis vectors that define the coordinate system     *
 * of the satellite body.  It also reads the vectors from the IGRF magnetic      *
 * field lookup tables and the Joint Space Operations Center to establish        *
 * satellite position relative to the earth (test files included are magout.txt, *
 * sunout.txt, igrfout.txt and jspocout.txt)..  Using the TRIAD algorithm and    *
 * these vectors, an attitude matrix is generated relating the satellite         *
 * body and orbital frames.  The formulation for the TRIAD comes from            *
 * www.dept.aoe.vt.edu/~cdhall/courses/aoe4140/attde.pdf .                       *
 * The name of this program is triad.c . Program author is Melissa Dumas.        *
 * Need to include the -lm switch to the gcc compile line to link to the         * 
 * math.h library so that the executable finds the sqrt function.                *
 *********************************************************************************/

#include "xc.h"
#include "triad.h"
#include <stdlib.h>
#include <math.h>

// Declare body frame arrays magarray and sunarray
// and Earth frame arrays are igrfarray and jspocarray
//double magarray[MAXPOINTS];
//double sunarray[MAXPOINTS];
//double igrfarray[MAXPOINTS];
//double jspocarray[MAXPOINTS];

// call
// err = triad(magarray, sunarray, igrfarray, jspocarray, &q0, &q1, &q2, &q3);


// Define triad function
void triad(double *marr, double *sarr, double *iarr, double *jarr, double *pq, double *att)
{

/**********************************************************************************************
 * Use the TRIAD algorithm to create the rotation matrix that relates the two frames and      *
 * creates the measured quaternion.  Begin by creating the component vectors.  The first of   *
 * each frame's component vectors will match exactly the known most accurate input vector of  *
 * each pair of body and earth vectors.  The remaining two triad vectors for each frame are   *
 * cross products.  Triad 2 for each frame is the unit vector of the cross product of the two *
 * measured vectors for each frame. Triad 3 in each frame is the cross product of the triad 1 *
 * and triad 2 vectors in each frame.                                                         *
 **********************************************************************************************/
 
    double t1body[3]; 
    double t2body[3];  
    double t3body[3];  

    double t1earth[3]; 
    double t2earth[3];   
    double t3earth[3];
    
    //double qdbody[3];

    // Input the known most accurate body vector for body triad 1
    t1body[0] = marr[0];
    t1body[1] = marr[1];
    t1body[2] = marr[2];

    // Perform the cross product:  magarray x sunarray to get body triad 2
    // Then normalize
    t2body[0] = ((marr[1]*sarr[2]) - (marr[2]*sarr[1]));
    t2body[1] = ((marr[2]*sarr[0]) - (marr[0]*sarr[2]));
    t2body[2] = ((marr[0]*sarr[1]) - (marr[1]*sarr[0]));

    // Perform the cross product:  t1body x t2body to get body triad 3
    t3body[0] = (t1body[1]*t2body[2]) - (t1body[2]*t2body[1]);
    t3body[1] = (t1body[2]*t2body[0]) - (t1body[0]*t2body[2]);
    t3body[2] = (t1body[0]*t2body[1]) - (t1body[1]*t2body[0]);

    // Input the known most accurate earth vector for earth triad 1
    t1earth[0] = iarr[0];
    t1earth[1] = iarr[1];
    t1earth[2] = iarr[2];

    // Perform the cross product:  igrfarray x jspocarray to get earth triad 2
    
    t2earth[0] = ((iarr[1]*jarr[2]) - (iarr[2]*jarr[1]));
    t2earth[1] = ((iarr[2]*jarr[0]) - (iarr[0]*jarr[2]));
    t2earth[2] = ((iarr[0]*jarr[1]) - (iarr[1]*jarr[0]));

    // Perform the cross product:  t1earth x t2earth to get earth triad 3
    t3earth[0] = (t1earth[1]*t2earth[2]) - (t1earth[2]*t2earth[1]);
    t3earth[1] = (t1earth[2]*t2earth[0]) - (t1earth[0]*t2earth[2]);
    t3earth[2] = (t1earth[0]*t2earth[1]) - (t1earth[1]*t2earth[0]);

    /******************************************************************************* 
     * Next we form the attitude matrix.  This is done by multiplying the matrix   *
     * of body vectors by the matrix of earth vectors.  First we declare the       *
     * matrices.  Body and earth vectors are entered into these  matrices as       *
     * column vectors.  Then we multiply them in the order matbody*matearth.       *
     * *****************************************************************************/
    
    // Declare the attitude matrix and multiply the body and earth matrices to get
    // its elements.
    double attmat[3][3] = { {((t1body[0]*t1earth[0])+(t2body[0]*t2earth[0])+(t3body[0]*t3earth[0])), 
                             ((t1body[0]*t1earth[1])+(t2body[0]*t2earth[1])+(t3body[0]*t3earth[1])), 
                             ((t1body[0]*t1earth[2])+(t2body[0]*t2earth[2])+(t3body[0]*t3earth[2]))},
                            {((t1body[1]*t1earth[0])+(t2body[1]*t2earth[0])+(t3body[1]*t3earth[0])), 
                             ((t1body[1]*t1earth[1])+(t2body[1]*t2earth[1])+(t3body[1]*t3earth[1])), 
                             ((t1body[1]*t1earth[2])+(t2body[1]*t2earth[2])+(t3body[1]*t3earth[2]))},
                            {((t1body[2]*t1earth[0])+(t2body[2]*t2earth[0])+(t3body[2]*t3earth[0])), 
                             ((t1body[2]*t1earth[1])+(t2body[2]*t2earth[1])+(t3body[2]*t3earth[1])), 
                             ((t1body[2]*t1earth[2])+(t2body[2]*t2earth[2])+(t3body[2]*t3earth[2]))} };
    
    /*******************************************************************************
    * Extract the quaternion from the attmat and dattmat using formulation        *
    *from "An Inverse Dynamics Satellite Attitude Determination and Control       *
    *System with Autonomous Calibration" Omar et al. 2015. pdf available at       *
    *pdfs.semanticscholar.org/1072/6477c7d2a90b9797452a1a045f27a5406e6d.pdf       *
    *******************************************************************************/
    
    double lq0 = sqrt((attmat[0][0]+attmat[1][1]+attmat[2][2]+1)/4);
    pq[0] = lq0;
    pq[1] = (attmat[2][1]-attmat[1][2])/(4*lq0);
    pq[2] = (attmat[0][2]-attmat[2][0])/(4*lq0);
    pq[3] = (attmat[1][0]-attmat[0][1])/(4*lq0);
    
    // export the attitude matrix as the function return value in att[9] = 
    // [[row1],[row2],[row3]]
    int i,j;
    int k = 0;
    for (i=0 ; i<3 ; i++)
    {
        for (j=0 ; j<3 ; j++)
        {
            att[k]=attmat[i][j];
            k++;
        }
    }
    
}

void rbody(double *att, double *in_vector, double *out_vector)
{
    out_vector[0] = in_vector[0]*att[0] + in_vector[1]*att[1] + in_vector[2]*att[2];
    out_vector[1] = in_vector[0]*att[3] + in_vector[1]*att[4] + in_vector[2]*att[5];
    out_vector[2] = in_vector[0]*att[6] + in_vector[1]*att[7] + in_vector[2]*att[8];
}

void desired_q(double *att, double *nadir_eci, double *q)
{
    double nadir_body[3];
    rbody(att, nadir_eci, nadir_body);
    
    // define a vector in body coordinates that points to the +Z direction
    const double zvec[3] = {0.0,0.0,1.0};
    // calculate angle between zvec and nadir_body using dot product
    double zdotbody = zvec[0]*nadir_body[0] + zvec[1]*nadir_body[1] + zvec[2]*nadir_body[2];
    double zvec_angle = acos(zdotbody);
    // calculate scalar component of desired quaternion
    q[0] = cos(zvec_angle/2.0);
    
    // calculate the desired rotation axis as cross product with zvec and nadir_body
    double zvec_rotaxis[3];
    zvec_rotaxis[0] = (zvec[1]*nadir_body[2]-zvec[2]*nadir_body[1]);
    zvec_rotaxis[1] = (zvec[2]*nadir_body[0]-zvec[0]*nadir_body[2]);
    zvec_rotaxis[2] = (zvec[0]*nadir_body[1]-zvec[1]*nadir_body[0]);
    
    // calculate the vector part of quaternion
    q[1] = zvec_rotaxis[0]*sin(zvec_angle/2.0);
    q[2] = zvec_rotaxis[1]*sin(zvec_angle/2.0);
    q[3] = zvec_rotaxis[2]*sin(zvec_angle/2.0);
    
    //q[0] = zvec_angle;
    //q[1] = nadir_body[0];
    //q[2] = nadir_body[1];
    //q[3] = nadir_body[2];
    //q[1] = zvec_rotaxis[0];
    //q[2] = zvec_rotaxis[1];
    //q[3] = zvec_rotaxis[2];
}

void rotate(double *dtime, double *pq1, double *pq2, double *dq, double *b_body, double *omega, double *dipole)
{
    // b_body is in nT, converted here to T
    // omega is radians/sec double[3]
    // dipole return is in Am^2 double[3]
    
    double ts[3] = {180, 180, 180};                    // settling time in seconds
    double zeta[3] = {0.65, 0.65, 0.65};            // damping coefficient

    // normalize the two input position quaternions
    double pq1_mag = sqrt(pq1[0]*pq1[0] + pq1[1]*pq1[1] + pq1[2]*pq1[2] + pq1[3]*pq1[3]);
    if (pq1_mag)
    {
        pq1[0] /= pq1_mag;
        pq1[1] /= pq1_mag;
        pq1[2] /= pq1_mag;
        pq1[3] /= pq1_mag;
    }
    double pq2_mag = sqrt(pq2[0]*pq2[0] + pq2[1]*pq2[1] + pq2[2]*pq2[2] + pq2[3]*pq2[3]);
    if (pq2_mag)
    {
        pq2[0] /= pq2_mag;
        pq2[1] /= pq2_mag;
        pq2[2] /= pq2_mag;
        pq2[3] /= pq2_mag;
    }
    double dq_mag = sqrt(dq[0]*dq[0] + dq[1]*dq[1] + dq[2]*dq[2] + dq[3]*dq[3]);
    if (dq_mag)
    {
        dq[0] /= dq_mag;
        dq[1] /= dq_mag;
        dq[2] /= dq_mag;
        dq[3] /= dq_mag;
    }
    
    // Declare RamSat dimensions
    double satmass = 2.556;   // units: kg
    double satheight = 0.20;  // units: m
    double satwidth = 0.10;
    double satdepth = 0.10;

    // Calculate the inertia matrix for the satellite body (This is the J term in torque calc)
    double ibody[3][3] = { {(satmass*((satheight*satheight)+(satdepth*satdepth)))/12, 0, 0},
                           { 0, (satmass*((satwidth*satwidth)+(satheight*satheight)))/12, 0},
                           { 0, 0, (satmass*((satdepth*satdepth)+(satwidth*satwidth)))/12} };

    // natural frequency                                               
    double omegan[3] = {4.4/(ts[0]*zeta[0]), 4.4/(ts[1]*zeta[1]), 4.4/(ts[2]*zeta[2])};

    // proportional gains (2*J*zeta*omegan)   
    double kp[3] = {2*ibody[0][0]*zeta[0]*omegan[0], 
                 2*ibody[1][1]*zeta[1]*omegan[1], 
                 2*ibody[2][2]*zeta[2]*omegan[2]}; 

    // derivative gains (2*J*omegan^2)
    double kd[3] = {2*ibody[0][0]*omegan[0]*omegan[0], 
                 2*ibody[1][1]*omegan[1]*omegan[1], 
                 2*ibody[2][2]*omegan[2]*omegan[2]};

    // attitude error (position quaternion vs desired position quaternion)
    // quaternion error https://digitalcommons.usu.edu/cgi/viewcontent.cgi?article=3221&context=smallsat
    double ae[4] = { dq[0]*pq1[0]+dq[1]*pq1[1]+dq[2]*pq1[2]+dq[3]*pq1[3],
                  -dq[1]*pq1[0]+dq[0]*pq1[1]+dq[3]*pq1[2]-dq[2]*pq1[3],
                  -dq[2]*pq1[0]-dq[3]*pq1[1]+dq[0]*pq1[2]+dq[1]*pq1[3],
                  -dq[3]*pq1[0]+dq[2]*pq1[1]-dq[1]*pq1[2]+dq[0]*pq1[3] };
   
    // Define the 4D matrix from the second quaternion that is used to calculate angular velocity
    double q12[4] = { pq2[0]*pq1[0]+pq2[1]*pq1[1]+pq2[2]*pq1[2]+pq2[3]*pq1[3],
                  -pq2[1]*pq1[0]+pq2[0]*pq1[1]+pq2[3]*pq1[2]-pq2[2]*pq1[3],
                  -pq2[2]*pq1[0]-pq2[3]*pq1[1]+pq2[0]*pq1[2]+pq2[1]*pq1[3],
                  -pq2[3]*pq1[0]+pq2[2]*pq1[1]-pq2[1]*pq1[2]+pq2[0]*pq1[3] };
    // Calculate the angle from the first measured quaternion to the second measured quaternion
    double theta = 2*acos(q12[0]);
    double fomega = theta/(*dtime);
    // Calculate the rotation vector around which the rotation occurred
    double nhat[3] = {0,0,0};
    if (theta)
    {
        nhat[0] = q12[0]/sin(theta/2);
        nhat[1] = q12[1]/sin(theta/2);
        nhat[2] = q12[2]/sin(theta/2);
    }
    omega[0] = fomega*nhat[0];
    omega[1] = fomega*nhat[1];
    omega[2] = fomega*nhat[2];  
    
    // control torque (Nm) calculation 
    // Multiplied the vector part of the quaternion with kp as in:
    // https://digitalcommons.usu.edu/cgi/viewcontent.cgi?article=3221&context=smallsat
    double propterm[3] = {-(kp[0]*ae[1]), (-kp[1]*ae[2]), (-kp[2]*ae[3])}; 
    double derivterm[3] = {kd[0]*omega[0], kd[1]*omega[1], kd[2]*omega[2]};
    double torque[3] = {propterm[0]-derivterm[0], propterm[1]-derivterm[1], propterm[2]-derivterm[2]};
    // convert magnetic field in body frame from nT to T
    double bT[3];
    bT[0] = b_body[0] * 1e-9;
    bT[1] = b_body[1] * 1e-9;
    bT[2] = b_body[2] * 1e-9;
    double bT_mag = sqrt(bT[0]*bT[0] + bT[1]*bT[1] + bT[2]*bT[2]);
    double bT_mag2 = bT_mag * bT_mag;
    
    // Calculate the magnetic dipole (Am^2) required to rotate the RamSat body to the desired attitude
    // bT X torque 
    dipole[0] = (bT[1]*torque[2]-bT[2]*torque[1]) / bT_mag2;
    dipole[1] = (bT[2]*torque[0]-bT[0]*torque[2]) / bT_mag2;
    dipole[2] = (bT[0]*torque[1]-bT[1]*torque[0]) / bT_mag2;
}