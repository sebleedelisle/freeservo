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
 *    Filename:       main.c           	                             *
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
 *  This is the main function for the BLDC application.  This code 
 *  calls the setup routines, initializes system variables and
 *  parameters, enables interrupts, and then goes into a polling loop.
 *  The main polling loop clears the WDT and runs the medium (10msec)
 *  and slow (100msec) event handlers when appropriate.
 *  
 **********************************************************************/

#include "general.h"
#include "hardware.h"
#include "defs.h"
#include "extern_globals.h"

/*********************************************************************/

extern void setup_ports(void);
extern void setup_motor_pwms(void);
extern void setup_adc(void);
extern void setup_qei(void);
extern void setup_timers(void);

#ifdef DEVELOPMODE
extern void setup_uart(void);
extern void serial_handler(void);
extern void process_parameters(void);
#endif

extern void medium_event_handler(void);
extern void slow_event_handler(void);






int main()
{
	unsigned char i;
	

	run_state=INITIALIZING;
	// Set up peripherals - see setup.c for source code
	setup_ports();
	// Note interrupts must not be running during setup_motor_pwms()
	// as config registers are written to set firing polarity
	setup_motor_pwms();
	setup_adc();
	setup_qei();
	setup_timers();
	setup_uart();

	// Reset Power Module using delay due to screen initialization 
	// to ensure don't violate min pulse width of 2us at 12C671 PIC
	// in the power module
		

	IFS2bits.FLTAIF=0;

	// Enable All types ofMath Error Traps
	INTCON1bits.OVATE=TRUE;
	INTCON1bits.OVBTE=TRUE;
	INTCON1bits.COVTE=TRUE;
	// Make the FAULT A Interrupt highest priority=7
	// Leave all others at the default of 4
	IPC10bits.FLTAIP=7;
	// Enable interrupts
	// NB T2 used for Output Compare is not enabled
	// here but only when commutation is required.
	// The same goes for T1 which is used as a guard
	// timer to catch a missed zero crossing event.
	IEC2bits.PWMIE = 1;
	IEC2bits.FLTAIE=1;
	IEC0bits.ADIE = 1;
	// Set ACQUIRE2 flag high to force capture of all three
	// phase voltage channels so offsets can be calibrated
	// out in process_parameters
	control_flags.ACQUIRE2=TRUE;
	// Now actually initialize display which takes sufficient
	// time to easily ensure have valid ADC readings
	//OpenXLCD(FOUR_BIT&LINES_5X7);
	// Clear ACQUIRE2 flag
	control_flags.ACQUIRE2=FALSE;
	

	// Now force any variables which depend on user_parameters
	// to be calculated by calling process_parameters for every
	// parameter. 
	
	for (i=0;i<NO_PARAMETERS;i++)
	{
		param=i;
		process_parameters();
	}
	// Set param back to start of menu
	param=0;
	// Main background loop starts here		
	while(1)
	{
		ClrWdt();
		medium_event_handler();								
		slow_event_handler();
#ifdef DEVELOPMODE
		if (run_state == STANDBY)
			serial_handler();
#endif

	}	
}


