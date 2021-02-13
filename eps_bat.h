/* 
 * File:              eps.h
 * Author:            Peter Thornton 
 * Comments:          Public interface for eps routines
 * Revision history:  1 March 2020
 */

unsigned char eps_get_status();
void eps_reset_watchdog();
void eps_set_watchdog(unsigned char minutes);
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
float eps_get_bcrouti();
float eps_get_bati();
float eps_get_bus12i();
float eps_get_bus5i();
float eps_get_bus33i();
float eps_get_eps5i();
float eps_get_eps33i();
float eps_get_mbt();
float eps_get_sa1at();
float eps_get_sa1bt();
float eps_get_sa2at();
float eps_get_sa2bt();
float eps_get_sa3bt();
void eps_set_pdm_initial_off(int pdm);
void eps_set_pdm_all_off();
int eps_get_pdm_initial();
int eps_get_pdm_expected();
int eps_get_pdm_actual();
void eps_cameras_on();
void eps_cameras_off();
void eps_antenna_on();
void eps_antenna_off();
unsigned char eps_antenna_status();
unsigned char eps_get_last_error();
void eps_batvbus_reset();

unsigned char bat_get_status();
float bat_get_mbt();
float bat_get_batv();
float bat_get_bati();
int   bat_get_batischarging();
float bat_get_dbt();
int   bat_get_nbr();
int   bat_get_nar();
int   bat_get_nmr();

