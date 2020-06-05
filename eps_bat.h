/* 
 * File:              eps.h
 * Author:            Peter Thornton 
 * Comments:          Public interface for eps routines
 * Revision history:  1 March 2020
 */

unsigned char eps_get_status();
float eps_get_bcr1v();
float eps_get_bcr2v();
float eps_get_bcr3v();
float eps_get_bcroutv();
float eps_get_batv();
float eps_get_bus12v();
float eps_get_bus5v();
float eps_get_bus33v();
float eps_get_bcr1ia();
float eps_get_bcr1ib();
float eps_get_bcr2ia();
float eps_get_bcr2ib();
float eps_get_bcr3ia();
float eps_get_bcr3ib();
float eps_get_bati();
float eps_get_bus12i();
float eps_get_bus5i();
float eps_get_bus33i();
float eps_get_eps5i();
float eps_get_eps33i();
float eps_get_mbt();
unsigned char eps_cameras_on();
unsigned char eps_cameras_off();

unsigned char bat_get_status();
float bat_get_mbt();
float bat_get_batv();
float bat_get_bati();
int   bat_get_batischarging();
float bat_get_dbt();

