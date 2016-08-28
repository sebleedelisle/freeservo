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
 *    Filename:       hardware.h       	                             *
 *    Date:           12/10/03                                         *
 *    File Version:   3.01                                             *
 *    Project:        53                                               *
 *    Drawing:        2                                                *
 *                                                                     *
 *    Tools used:    MPLAB C30 Compiler v 1.10.02                      *
 *                                                                     *
 *    Linker File:    p30f3010.gld                                   *
 *                                                                     *
 *                                                                     *
 ***********************************************************************
 *	Code Description
 *  
 * This file contins details of the hardware for the project
 * PCB : dsPIC30F Motor Control Development PCB 02-01648 REVB
 *
 **********************************************************************/

// This is the clock frequency in Hz including any PLL factor
#define CLOCK_FREQ 10000000 // Assuming 5.00MHz and X8 PLL

// NB Using LAT Registers rather than PORT registers
// to prevent any read/modify/write issues with outputs

// These are the pushbutton pin connections



// This is the control line for the driver IC for the firing signals
// It must be low to activate the signals to the power module.
#define	PWM_OUTPUT_DISABLE LATDbits.LATD11

// Also declare some useful shortcuts
#define DISABLE_FIRING PWMCON1 = 0x0700
#define ENABLE_FIRING  PWMCON1 = 0x0777



