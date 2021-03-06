/* 
 * File:       command.h
 * Author:     Peter Thornton
 * Purpose:    function prototypes for command handlers
 * Created on: 14 Oct 2020
 */

// function prototypes 
void CmdNoOp(void);
int CmdFileCount(void);
int CmdFileList(void);
int CmdFileDump(char *paramstr);
int CmdFileDumpRange(char *paramstr);
int CmdFileDumpOnePacket(char *paramstr);
int CmdNewTLE(char *paramstr, int param_nbytes, int *good_tle);
int CmdGetDateTime(void);
int CmdSetDateTime(char* paramstr, int param_nbytes);
int CmdEraseSector(char* paramstr);
int CmdWritePage(char* paramstr);
int CmdDownlinkPage(char* paramstr);
int CmdGetTelemControl(char* paramstr);
int CmdGetTelemData(char* paramstr);
int CmdCurrentTelemetry(char* paramstr);
int CmdCaptureImage(char* paramstr);
int CmdCameraPower(char* paramstr);
int CmdFileDelete(char *paramstr);
int CmdStartDetumble(char *paramstr);
int CmdImtqPWM(char *paramstr);
int CmdConfigAutoImage(char *paramstr, auto_image_type *autoimgp);
int CmdAutoImageOn(char *paramstr, auto_image_type *autoimgp);
int CmdRotateParams(char *paramstr, double *ts, double *zeta);
int CmdSunSensorMask(char *paramstr, int *ss_m);
int CmdSunSensorScalar(char *paramstr, float *ss_s);
int CmdConfigTelem0(char *paramstr);
int CmdConfigTelem1(char *paramstr);
int CmdConfigTelem2(char *paramstr);
int CmdTelemIsActive(char *paramstr);
void CmdSetPDT(void);
void CmdResetHe100(void);
void CmdResetPIC(void);





