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
 *    Filename:       slow_event.c     	                             *
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
 *  This is the code for the slow event handler, which is executed 
 *  every 100 msec in this application.  The slow event handler updates
 *  the display screens, processes button input,and stores system
 *  parameters into user Flash.
 *  
 **********************************************************************/


#include "general.h"
#include "hardware.h"
#include "defs.h"
#include "extern_globals.h"

void slow_event_handler(void);

#ifdef DEVELOPMODE
extern void send_run(void);
extern void send_fault(void);
#endif

extern void process_switches(void);

void slow_event_handler(void)
{
	// SLOW_EVENT_RATE is set to 100ms in defs.h
	if (slow_event_count > SLOW_EVENT_RATE) 
	{
		// Provide simple filtering of feedback values
		// for display purposes only
		filtered_vdc=(filtered_vdc+vdc)/2;
		filtered_pot=(filtered_pot+pot)/2;
		filtered_rpm=(filtered_rpm+rpm)/2;
		filtered_ibus=(filtered_ibus+ibus)/2;

		process_switches();
		slow_event_count=0;
#ifdef DEVELOPMODE
		if (run_state == FAULT)
			send_fault();
		if (run_state == RUNNING)
			send_run();
#endif
	}
	return;
}


