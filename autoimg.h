/* 
 * File:       autoimg.h
 * Author:     Peter Thornton
 * Purpose:    holds data type for auto image collection
 * Created on: 4 March 2021 
 */

typedef struct
{
    int on;             // Flag: auto-imaging is on (1) or off (0)
    double min_lon;     // minimum longitude for imaging window (degrees)
    double max_lon;     // maximum longitude for imaging window (degrees)
    double min_lat;     // minimum latitude for imaging window (degrees)
    double max_lat;     // maximum latitude for imaging window (degrees)
    double max_res;     // maximum allowable ramsat-earth-sun angle (degrees)
    double max_offnadir;// maximum allowable off-nadir view angle (degrees)
    double max_images;  // maximum number of images to take with current settings
    int nextimg;        // (base-0) image number that will be taken next
    long int time_since_last; // number of minutes elapsed since the last autoimg
    int capture;        // set to 1 in an interval when images were captured
} auto_image_type;




