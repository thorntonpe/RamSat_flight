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
int CmdNewTLE(char *paramstr, int param_nbytes, int *good_tle);
int CmdGetDateTime(void);
int CmdSetDateTime(char* paramstr, int param_nbytes);
int CmdEraseSector(char* paramstr);
int CmdWritePage(char* paramstr);
int CmdDownlinkPage(char* paramstr);
int CmdGetTelemControl(char* paramstr);
int CmdGetTelemData(char* paramstr);
int CmdCaptureImage(char* paramstr);
void CmdSetPDT(void);
void CmdReset(void);





