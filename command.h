/* 
 * File:       command.h
 * Author:     Peter Thornton
 * Purpose:    function prototypes for command handlers
 * Created on: 14 Oct 2020
 */

// function prototypes 
int CmdFileCount(void);
int CmdFileList(void);
int CmdFileDump(char *paramstr);
int CmdNewTLE(char *paramstr, int param_nbytes, int *good_tle);





