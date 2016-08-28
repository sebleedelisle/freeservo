/**********************************************************************
 *                                                                     *
 *                        Software License Agreement                   *
 *                                                                     *
 *    The software supplied herewith by Microchip Technology           *
 *    Incorporated (the "Company") for its dsPIC controller            *
 *    is intended and supplied to you, the Company's customer,         *
 *    for use solely and exclusively on Microchip dsPIC                *
 *    products. The software is owned by the Company and/or its        *
 *    supplier, and is protected under applicable copyright laws. All  *
 *    rights are reserved. Any use in violation of the foregoing       *
 *    restrictions may subject the user to criminal sanctions under    *
 *    applicable laws, as well as to civil liability for the breach of *
 *    the terms and conditions of this license.                        *
 *                                                                     *
 *    THIS SOFTWARE IS PROVIDED IN AN "AS IS" CONDITION.  NO           *
 *    WARRANTIES, WHETHER EXPRESS, IMPLIED OR STATUTORY, INCLUDING,    *
 *    BUT NOT LIMITED TO, IMPLIED WARRANTIES OF MERCHANTABILITY AND    *
 *    FITNESS FOR A PARTICULAR PURPOSE APPLY TO THIS SOFTWARE. THE     *
 *    COMPANY SHALL NOT, IN ANY CIRCUMSTANCES, BE LIABLE FOR SPECIAL,  *
 *    INCIDENTAL OR CONSEQUENTIAL DAMAGES, FOR ANY REASON WHATSOEVER.  *
 *                                                                     *
  **********************************************************************/

 /**********************************************************************
 *                                                                     * 
 *    Author: Smart Power Soutions, LLP                                * 
 *                                                                     *
 *    Filename:       flash_routines.c	                                *
 *    Date:           4/21/04                                         *
 *    File Version:   4.00                                             *
 *    Project:        53                                               *
 *    Drawing:        2                                                *
 *                                                                     *
 *    Tools used:    MPLAB C30 Compiler v 1.20.01                      *
 *                                                                     *
 *    Linker File:    p30f4012.gld                                   *
 *                                                                     *
 *                                                                     *
 ***********************************************************************
 *	Code Description
 *  
 *  This file contains routines that will erase and reprogram one row
 *  (32 instruction words) of user Flash memory.  The user Flash is 
 *  used in the application to store startup and run parameters in the
 *  BLDC application.
 *  
 **********************************************************************/

#include "general.h"

void erase_flash_row(unsigned int );
void program_flash(unsigned int, unsigned int);


// This function erases one row (32 instruction words)
// of Flash program memory.
// The address supplied is assumed to be derived via PSV
void erase_flash_row(unsigned int address) 
{ 
	// Set up control register to enable an erase of a 
	// program row
	asm("mov	#0x4041,W1");
	asm("mov	W1,NVMCON");
	// Now calculate the address setting upper word
	// of address to zero.
	asm("clr	NVMADRU");
	// This function assumes you are using PSV and
	// so need to mask off the MS bit of the address
	// which will be in W0
	asm("bclr 	W0,#15");
	asm("mov		W0,NVMADR");
	// Now activate the flash erase sequence
	asm("	bset NVMCON,#14"); 
	asm("	mov	#0x55,W1");
	asm("	mov	W1,NVMKEY");
	asm("	mov	#0xaa,W1");  
	asm("	mov	W1,NVMKEY");
	asm("	bset NVMCON,#15");
	Nop();
	Nop();
	return;  
} 

// This routine writes to 32 consequtive program locations
// The destination is the base address of the 32 locations
// (assuming PSV access) therefore requiring masking of MS bit.
// The values to be written are assumed to be stored in 32 
// consecutive locations with the base address given by values
// Note that the top 8 bytes of program memory are not used and
// are set to zero.
void program_flash(unsigned int destination, unsigned int values)
{
	// Set up control register to enable programming of 
	// 4 program instructions
	asm("mov		#0x4001,W3");
	asm("mov		W3,NVMCON");
	// Mask off MS bit of destination address as assuming PSV
	// is being used.
	asm("bclr	W0,#15");
	// Set TBLPAG to zero as using PSV.
	asm("clr	TBLPAG");
	// Set W3 to zero so we can write 0 into the MS byte
	asm("clr	W3");
	// Load up the 32 values
	asm("tblwtl.w	[w1++],	[w0]");
	asm("tblwth.w	W3,		[w0++]");		// word 1
	asm("tblwtl.w	[w1++],	[w0]");
	asm("tblwth.w	W3,		[w0++]");		// word 2
	asm("tblwtl.w	[w1++],	[w0]");
	asm("tblwth.w	W3,		[w0++]");		// word 3
	asm("tblwtl.w	[w1++],	[w0]");
	asm("tblwth.w	W3,		[w0++]");		// word 4
	asm("tblwtl.w	[w1++],	[w0]");
	asm("tblwth.w	W3,		[w0++]");		// word 5
	asm("tblwtl.w	[w1++],	[w0]");
	asm("tblwth.w	W3,		[w0++]");		// word 6
	asm("tblwtl.w	[w1++],	[w0]");
	asm("tblwth.w	W3,		[w0++]");		// word 7
	asm("tblwtl.w	[w1++],	[w0]");
	asm("tblwth.w	W3,		[w0++]");		// word 8
	asm("tblwtl.w	[w1++],	[w0]");
	asm("tblwth.w	W3,		[w0++]");		// word 9
	asm("tblwtl.w	[w1++],	[w0]");
	asm("tblwth.w	W3,		[w0++]");		// word 10
	asm("tblwtl.w	[w1++],	[w0]");
	asm("tblwth.w	W3,		[w0++]");		// word 11
	asm("tblwtl.w	[w1++],	[w0]");
	asm("tblwth.w	W3,		[w0++]");		// word 12
	asm("tblwtl.w	[w1++],	[w0]");
	asm("tblwth.w	W3,		[w0++]");		// word 13
	asm("tblwtl.w	[w1++],	[w0]");
	asm("tblwth.w	W3,		[w0++]");		// word 14
	asm("tblwtl.w	[w1++],	[w0]");
	asm("tblwth.w	W3,		[w0++]");		// word 15
	asm("tblwtl.w	[w1++],	[w0]");
	asm("tblwth.w	W3,		[w0++]");		// word 16
	asm("tblwtl.w	[w1++],	[w0]");
	asm("tblwth.w	W3,		[w0++]");		// word 17
	asm("tblwtl.w	[w1++],	[w0]");
	asm("tblwth.w	W3,		[w0++]");		// word 18
	asm("tblwtl.w	[w1++],	[w0]");
	asm("tblwth.w	W3,		[w0++]");		// word 19
	asm("tblwtl.w	[w1++],	[w0]");
	asm("tblwth.w	W3,		[w0++]");		// word 20
	asm("tblwtl.w	[w1++],	[w0]");
	asm("tblwth.w	W3,		[w0++]");		// word 21
	asm("tblwtl.w	[w1++],	[w0]");
	asm("tblwth.w	W3,		[w0++]");		// word 22
	asm("tblwtl.w	[w1++],	[w0]");
	asm("tblwth.w	W3,		[w0++]");		// word 23
	asm("tblwtl.w	[w1++],	[w0]");
	asm("tblwth.w	W3,		[w0++]");		// word 24
	asm("tblwtl.w	[w1++],	[w0]");
	asm("tblwth.w	W3,		[w0++]");		// word 25
	asm("tblwtl.w	[w1++],	[w0]");
	asm("tblwth.w	W3,		[w0++]");		// word 26
	asm("tblwtl.w	[w1++],	[w0]");
	asm("tblwth.w	W3,		[w0++]");		// word 27
	asm("tblwtl.w	[w1++],	[w0]");
	asm("tblwth.w	W3,		[w0++]");		// word 28
	asm("tblwtl.w	[w1++],	[w0]");
	asm("tblwth.w	W3,		[w0++]");		// word 29
	asm("tblwtl.w	[w1++],	[w0]");
	asm("tblwth.w	W3,		[w0++]");		// word 30
	asm("tblwtl.w	[w1++],	[w0]");
	asm("tblwth.w	W3,		[w0++]");		// word 31
	asm("tblwtl.w	[w1++],	[w0]");
	asm("tblwth.w	W3,		[w0++]");		// word 32
	
	// Write the key sequence to start the programming
	asm("	bset NVMCON,#14"); 
	asm("	mov	#0x55,W1");
	asm("	mov	W1,NVMKEY");
	asm("	mov	#0xaa,W1");  
	asm("	mov	W1,NVMKEY");
	asm("	bset NVMCON,#15");
	Nop();
	Nop();
	return;
}
