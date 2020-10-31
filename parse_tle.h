/* 
 * File:       parse_tle.h
 * Author:     Peter Thornton
 * Purpose:    Function prototypes for parsing TLE
 * Created on: 25 October 2020 
 */

// function prototypes 
int tle_checksum( const char *buff);
int parse_elements( const char *line1, const char *line2, tle_t *sat);







