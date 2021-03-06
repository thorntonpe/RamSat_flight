/* 
 * File:       arducam_user.h
 * Author:     Peter Thornton
 * Purpose:    High-level user interface for arducam functions
 * Created on: 4 June 2020 
 */

// camera identifiers (distinct from the chip select defines)
#define CAM1 1
#define CAM2 2

// function prototypes 
int test_arducam_spi(void);
int init_arducam(void);
int arducam_auto_image(int image_number);






