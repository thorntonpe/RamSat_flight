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
void triad(double *marr, double *sarr, double *iarr, double *jarr, double *q0, double *q1, double *q2, double *q3, double *att)
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
    *q1 = (attmat[2][1]-attmat[1][2])/(4*lq0);
    *q2 = (attmat[0][2]-attmat[2][0])/(4*lq0);
    *q3 = (attmat[1][0]-attmat[0][1])/(4*lq0);
    *q0 = lq0;
    
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
    
    q[0]=nadir_body[0];
    q[1]=nadir_body[1];
    q[2]=nadir_body[2];
}