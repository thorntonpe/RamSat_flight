/* 
 * File:       pdt.h
 * Author:     Peter Thornton
 * Purpose:    parameters for the post-deployment timer mechanism
 * Created on: 14 Nov 2020 
 */

// address and flag values for the post-deployment timer
#define PDT_NMIN 2        // Number of minutes to wait after deployment
#define PDT_ADR1 0x00      // Address on SFM for post-deploy timer flag:
#define PDT_ADR2 0x10      // (00, 10, 00) is at the start of the second
#define PDT_ADR3 0x00      // 4k block in sector 1.
#define MUST_WAIT 0x55     // Flag value: Post-deploy timer has not yet completed
#define DONT_WAIT 0xaa     // Flag value: Post-deploy timer has already completed






