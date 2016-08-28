/*********************************************************************
*                                                                    *
*                       Software License Agreement                   *
*                                                                    *
*   The software supplied herewith by Microchip Technology           *
*   Incorporated (the "Company") for its dsPIC controller            *
*   is intended and supplied to you, the Company's customer,         *
*   for use solely and exclusively on Microchip dsPIC                *
*   products. The software is owned by the Company and/or its        *
*   supplier, and is protected under applicable copyright laws. All  *
*   rights are reserved. Any use in violation of the foregoing       *
*   restrictions may subject the user to criminal sanctions under    *
*   applicable laws, as well as to civil liability for the breach of *
*   the terms and conditions of this license.                        *
*                                                                    *
*   THIS SOFTWARE IS PROVIDED IN AN "AS IS" CONDITION.  NO           *
*   WARRANTIES, WHETHER EXPRESS, IMPLIED OR STATUTORY, INCLUDING,    *
*   BUT NOT LIMITED TO, IMPLIED WARRANTIES OF MERCHANTABILITY AND    *
*   FITNESS FOR A PARTICULAR PURPOSE APPLY TO THIS SOFTWARE. THE     *
*   COMPANY SHALL NOT, IN ANY CIRCUMSTANCES, BE LIABLE FOR SPECIAL,  *
*   INCIDENTAL OR CONSEQUENTIAL DAMAGES, FOR ANY REASON WHATSOEVER.  *
*                                                                    *
*********************************************************************/

//*****************************************************************
// psv2010.c 
// This simple program shows how PSV is done on a dsPIC using a
// MCLV demo board.  A character string is stored in PM
// as a constant.  It is automatically retreived as data memory
// by the C30 compiler using the PSV feature of dsPIC.
// To view the whole data and program memory mapping, create a
// *.map file in MPLAB and use the editor to view it after a compile.
// The user must connect a Hyperterm program to the UART and hit SW1
// on the PICDEM MCLV, to transmit a character string stored in PM.  
// The string will then be displayed in Hyperterm set at 19200 baud.
//*****************************************************************
//*                                                               *
//* Written by:  Stan D'Souza                                 *                               *
//*              Microchip Technology Inc.                        *
//* Date:        2-26-05                                 *
//* Revision:	 1.00                                             *
//*****************************************************************
//                                                                *
//*****************************************************************

#include "c:\pic30_tools\support\h\p30F3010.h"
#include "C:\pic30_tools\include\stdio.h"
#include "C:\pic30_tools\include\stdlib.h"




//---------------------------------------------------------------------
void DelayNmSec(unsigned int N);









//---------------------------------------------------------------------
// This is a generic 1ms delay routine to give a 1mS to 65.5 Seconds delay
// For N = 1 the delay is 1 mS, for N = 65535 the delay is 65,535 mS. 
// Note that FCY is used in the computation.  Please make the necessary
// Changes(PLLx4 or PLLx8 etc) to compute the right FCY as in the define
// statement above.

void DelayNmSec(unsigned int N)
{
unsigned int j;
while(N--)
 	for(j=0;j < MILLISEC;j++);
}


