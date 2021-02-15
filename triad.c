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
int triad(double *marr, double *sarr, double *iarr, double *jarr, double *q0, double *q1, double *q2, double *q3, double *qd0, double *qd1, double *qd2, double *qd3)
{
    double lq0;
    //double lqd0;
    
    // Calculate the magnitudes of each vector array
    double magnorm = sqrt(marr[0]*marr[0]+marr[1]*marr[1]+marr[2]*marr[2]);
    double sunnorm = sqrt(sarr[0]*sarr[0]+sarr[1]*sarr[1]+sarr[2]*sarr[2]);
    double igrfnorm = sqrt(iarr[0]*iarr[0]+iarr[1]*iarr[1]+iarr[2]*iarr[2]);
    double jspocnorm = sqrt(jarr[0]*jarr[0]+jarr[1]*jarr[1]+jarr[2]*jarr[2]);

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
    t2body[0] = ((marr[1]*sarr[2]) - (marr[2]*sarr[1]))/(magnorm*sunnorm);
    t2body[1] = ((marr[2]*sarr[0]) - (marr[0]*sarr[2]))/(magnorm*sunnorm);
    t2body[2] = ((marr[0]*sarr[1]) - (marr[1]*sarr[0]))/(magnorm*sunnorm);

    // Perform the cross product:  t1body x t2body to get body triad 3
    t3body[0] = (t1body[1]*t2body[2]) - (t1body[2]*t2body[1]);
    t3body[1] = (t1body[2]*t2body[0]) - (t1body[0]*t2body[2]);
    t3body[2] = (t1body[0]*t2body[1]) - (t1body[1]*t2body[0]);

    //qdbody[0] = 0;
    //qdbody[1] = 0;
    //qdbody[2] = 1;

    // Input the known most accurate earth vector for earth triad 1
    t1earth[0] = iarr[0];
    t1earth[1] = iarr[1];
    t1earth[2] = iarr[2];

    // Perform the cross product:  igrfarray x jspocarray to get earth triad 2
    // Then normalize
    
    t2earth[0] = ((iarr[1]*jarr[2]) - (iarr[2]*jarr[1]))/(igrfnorm*jspocnorm);
    t2earth[1] = ((iarr[2]*jarr[0]) - (iarr[0]*jarr[2]))/(igrfnorm*jspocnorm);
    t2earth[2] = ((iarr[0]*jarr[1]) - (iarr[1]*jarr[0]))/(igrfnorm*jspocnorm);

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
    
    //double qdmatbody[3][3] = { {qdbody[0], qdbody[0], qdbody[0]},
    //                           {qdbody[1], qdbody[1], qdbody[1]},
    //                           {qdbody[2], qdbody[2], qdbody[2]} };
    //double matbody[3][3] = { {t1body[0], t2body[0], t3body[0]},
    //                         {t1body[1], t2body[1], t3body[1]},
    //                         {t1body[2], t2body[2], t3body[2]} };
    //double matearth[3][3] = { {t1earth[0], t2earth[0], t3earth[0]},
    //                          {t1earth[1], t2earth[1], t3earth[1]},
    //                          {t1earth[2], t2earth[2], t3earth[2]} };

    // Declare the desired attitude matrix and multiply the qdbody and earth matrices to get
    // its elements.
    //double dattmat[3][3] = { {((qdbody[0]*t1earth[0])+(qdbody[0]*t1earth[1])+(qdbody[0]*t1earth[2])), 
    //                        ((qdbody[0]*t2earth[0])+(qdbody[0]*t2earth[1])+(qdbody[0]*t2earth[2])), 
    //                         ((qdbody[0]*t3earth[0])+(qdbody[0]*t3earth[1])+(qdbody[0]*t3earth[2]))},
    //                        {((qdbody[1]*t1earth[0])+(qdbody[1]*t1earth[1])+(qdbody[1]*t1earth[2])), 
    //                         ((qdbody[1]*t2earth[0])+(qdbody[1]*t2earth[1])+(qdbody[1]*t2earth[2])), 
    //                         ((qdbody[1]*t3earth[0])+(qdbody[1]*t3earth[1])+(qdbody[1]*t3earth[2]))},
    //                        {((qdbody[2]*t1earth[0])+(qdbody[2]*t1earth[1])+(qdbody[2]*t1earth[2])), 
    //                         ((qdbody[2]*t2earth[0])+(qdbody[2]*t2earth[1])+(qdbody[2]*t2earth[2])), 
    //                         ((qdbody[2]*t3earth[0])+(qdbody[2]*t3earth[1])+(qdbody[2]*t3earth[2]))} };
    // Declare the attitude matrix and multiply the body and earth matrices to get
    // its elements.
    double attmat[3][3] = { {((t1body[0]*t1earth[0])+(t2body[0]*t1earth[1])+(t3body[0]*t1earth[2])), 
                             ((t1body[0]*t2earth[0])+(t2body[0]*t2earth[1])+(t3body[0]*t2earth[2])), 
                             ((t1body[0]*t3earth[0])+(t2body[0]*t3earth[1])+(t3body[0]*t3earth[2]))},
                            {((t1body[1]*t1earth[0])+(t2body[1]*t1earth[1])+(t3body[1]*t1earth[2])), 
                             ((t1body[1]*t2earth[0])+(t2body[1]*t2earth[1])+(t3body[1]*t2earth[2])), 
                             ((t1body[1]*t3earth[0])+(t2body[1]*t3earth[1])+(t3body[1]*t3earth[2]))},
                            {((t1body[2]*t1earth[0])+(t2body[2]*t1earth[1])+(t3body[2]*t1earth[2])), 
                             ((t1body[2]*t2earth[0])+(t2body[2]*t2earth[1])+(t3body[2]*t2earth[2])), 
                             ((t1body[2]*t3earth[0])+(t2body[2]*t3earth[1])+(t3body[2]*t3earth[2]))} };
    
   /*******************************************************************************
    * Extract the quaternion from the attmat and dattmat using formulation        *
    *from "An Inverse Dynamics Satellite Attitude Determination and Control       *
    *System with Autonomous Calibration" Omar et al. 2015. pdf available at       *
    *pdfs.semanticscholar.org/1072/6477c7d2a90b9797452a1a045f27a5406e6d.pdf       *
    *******************************************************************************/
   
    lq0 = sqrt((attmat[0][0]+attmat[1][1]+attmat[2][2]+1)/4);
    *q1 = (attmat[2][1]-attmat[1][2])/(4*lq0);
    *q2 = (attmat[0][2]-attmat[2][0])/(4*lq0);
    *q3 = (attmat[1][0]-attmat[0][1])/(4*lq0);
    *q0 = lq0;

    
    //lqd0 = sqrt((dattmat[0][0]+dattmat[1][1]+dattmat[2][2]+1)/4);
    //*qd1 = (dattmat[2][1]-dattmat[1][2])/(4*lqd0);
    //*qd2 = (dattmat[0][2]-dattmat[2][0])/(4*lqd0);
    //*qd3 = (dattmat[1][0]-dattmat[0][1])/(4*lqd0);
    //*qd0 = lqd0;


    return 0;  
}